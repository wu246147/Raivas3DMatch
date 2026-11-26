#include "surface_match/Tool.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <stdexcept>

namespace sm
{
using namespace HalconCpp;
namespace
{
/** @brief 校验有限向量。
 * @param value 待检查 XYZ。
 * @throws std::invalid_argument 存在非有限值。 */
void finitePoint(const cv::Vec3d &value)
{
    for (double v : value.val)
        if (!std::isfinite(v))
            throw std::invalid_argument("坐标必须为有限值");
}
/** @brief 校验七元 pose 或连续 pose 列表。
 * @param pose HALCON pose 元组。
 * @param multiple true 允许为空及七项的整数倍，false 要求恰好七项。
 * @throws std::invalid_argument 元素数不合法或存在非有限值。 */
void checkPose(const HTuple &pose, bool multiple = false)
{
    if ((multiple && pose.Length() % 7 != 0) || (!multiple && pose.Length() != 7))
        throw std::invalid_argument("HALCON pose 必须包含完整的七项位姿");
    for (Hlong i = 0; i < pose.Length(); ++i)
        if (!std::isfinite(pose[i].D()))
            throw std::invalid_argument("位姿必须为有限值");
}
/** @brief 统一转换七元位姿到 abg。
 * @param pose 有效七元 pose。
 * @return Rp+T/abg/point 位姿。 */
HTuple abg(const HTuple &pose)
{
    HTuple out;
    ConvertPoseType(pose, "Rp+T", "abg", "point", &out);
    return out;
}
/** @brief 校验并转换深度图到双精度，保持原尺寸。
 * @param depth 非空单通道 uint16/int16/float32/float64。
 * @return 独立双精度深度图。
 * @throws std::invalid_argument 输入不支持。 */
cv::Mat depth64(const cv::Mat &depth)
{
    if (depth.empty() || depth.channels() != 1 ||
        (depth.depth() != CV_16U && depth.depth() != CV_16S && depth.depth() != CV_32F && depth.depth() != CV_64F))
        throw std::invalid_argument("深度图须为非空单通道 uint16/int16/float32/float64");
    cv::Mat out;
    depth.convertTo(out, CV_64F);
    return out;
}
/** @brief 校验并准备权重，避免负权重、零和与长度错误。
 * @param size 数据数量，必须大于零。
 * @param weights 用户权重，空表示等权。
 * @return 已校验权重列表。 */
std::vector<double> checkedWeights(size_t size, const std::vector<double> &weights)
{
    auto result = weights.empty() ? std::vector<double>(size, 1.0) : weights;
    if (!size || result.size() != size)
        throw std::invalid_argument("数据和权重不能为空且数量须一致");
    for (double w : result)
        if (!std::isfinite(w) || w < 0)
            throw std::invalid_argument("权重须为有限非负数");
    double sum = std::accumulate(result.begin(), result.end(), 0.0);
    if (!std::isfinite(sum) || sum <= 0)
        throw std::invalid_argument("权重总和必须为有限正数");
    for (auto &w : result)
        w /= sum;
    return result;
}
/** @brief 校验兼容入口的固定点数并委派公共算法。
 * @param points 调用方的有限 XYZ 点集，不修改。
 * @param expected 必须满足的固定数量。
 * @return 公共算法输出的七元 abg pose。
 * @throws std::invalid_argument 数量错误或几何结构退化。 */
HTuple fixedPointPose(const std::vector<cv::Vec3d> &points, size_t expected)
{
    if (points.size() != expected)
        throw std::invalid_argument("定位点数与函数要求不一致");
    return Tool::mulPointToPose(points);
}
} // 匿名命名空间

void CameraIntrinsics::validate() const
{
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 || !std::isfinite(cx) || !std::isfinite(cy) ||
        !std::isfinite(depthScale) || depthScale <= 0 || !std::isfinite(depthOffset))
        throw std::invalid_argument("焦距和深度比例须为正数，内参须为有限值");
}
void Bounds3D::validate() const
{
    for (int i = 0; i < 3; ++i)
        if (std::isnan(min[i]) || std::isnan(max[i]) || min[i] > max[i])
            throw std::invalid_argument("裁剪范围上下界无效");
}
void MatchParameters::validate() const
{
    if (numMatches <= 0 || !std::isfinite(minScore) || minScore < 0 || minScore > 1 ||
        !std::isfinite(maxOverlapDistance) || maxOverlapDistance < 0 || !std::isfinite(modelSampling) ||
        modelSampling <= 0 || modelSampling > 1 || !std::isfinite(sceneSampling) || sceneSampling <= 0 ||
        sceneSampling > 1 || !std::isfinite(keyPointFraction) || keyPointFraction <= 0 || keyPointFraction > 1 ||
        !std::isfinite(poseRefScoringDistance) || poseRefScoringDistance <= 0 ||
        (modelSmoothK != 0 && modelSmoothK < 3) || (sceneSmoothK != 0 && sceneSmoothK < 3) ||
        !std::isfinite(downsampleDistance) || downsampleDistance < 0 || roiExpansion < 0 ||
        !std::isfinite(angleTolerance) || angleTolerance < 0 || angleTolerance > 180 ||
        (sceneNormals != "mls" && sceneNormals != "fast"))
        throw std::invalid_argument("表面匹配参数超出有效范围");
}
cv::Mat Tool::readImage(const std::string &path, bool depth)
{
    std::ifstream stream(std::filesystem::u8path(path), std::ios::binary);
    if (!stream)
        throw std::runtime_error("无法打开图像文件：" + path);
    std::vector<uchar> bytes((std::istreambuf_iterator<char>(stream)), {});
    if (bytes.empty())
        throw std::runtime_error("图像文件为空：" + path);
    cv::Mat image = cv::imdecode(bytes, depth ? cv::IMREAD_UNCHANGED : cv::IMREAD_COLOR);
    if (image.empty())
        throw std::runtime_error("无法解码图像：" + path);
    return image;
}
cv::Mat Tool::LimitImage(const cv::Mat &image, const cv::Rect &roi)
{
    return LimitImage(image, std::vector<cv::Rect>{roi});
}
cv::Mat Tool::LimitImage(const cv::Mat &image, const std::vector<cv::Rect> &rois)
{
    if (image.empty())
        throw std::invalid_argument("输入图像为空");
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U), out = cv::Mat::zeros(image.size(), image.type());
    for (const auto &roi : rois)
    {
        auto r = roi & cv::Rect(0, 0, image.cols, image.rows);
        if (!r.empty())
            mask(r).setTo(255);
    }
    image.copyTo(out, mask);
    return out;
}
void Tool::DrawRotatedRect(cv::Mat &image, const cv::RotatedRect &rect, double radians, const cv::Scalar &color,
                           int thickness)
{
    if (image.empty() || thickness <= 0 || !std::isfinite(radians))
        throw std::invalid_argument("绘图参数无效");
    cv::Point2f p[4];
    rect.points(p);
    for (int i = 0; i < 4; ++i)
        cv::line(image, p[i], p[(i + 1) % 4], color, thickness, cv::LINE_AA);
    double length = std::min(rect.size.width * 0.5, 40.0);
    cv::arrowedLine(image, rect.center,
                    rect.center + cv::Point2f(float(length * std::cos(radians)), float(length * std::sin(radians))),
                    color, thickness, cv::LINE_AA);
    cv::putText(image, cv::format("%.1f", radians * 180 / CV_PI), rect.center + cv::Point2f(5, -5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
}
HImage Tool::MatToHObject(const cv::Mat &image)
{
    if (image.empty())
        throw std::invalid_argument("图像为空");
    cv::Mat data = image.isContinuous() ? image : image.clone();
    HImage result;
    if (data.type() == CV_8UC3 || data.type() == CV_8UC4)
    {
        result.GenImageInterleaved(data.data, data.channels() == 3 ? "bgr" : "bgrx", data.cols, data.rows, 0, "byte",
                                   data.cols, data.rows, 0, 0, 8, 0);
        return result;
    }
    if (data.channels() != 1)
        throw std::invalid_argument("不支持此通道数");
    const char *type = nullptr;
    switch (data.depth())
    {
    case CV_8U:
        type = "byte";
        break;
    case CV_16U:
        type = "uint2";
        break;
    case CV_16S:
        type = "int2";
        break;
    case CV_32F:
        type = "real";
        break;
    case CV_64F:
        data.convertTo(data, CV_32F);
        type = "real";
        break;
    default:
        throw std::invalid_argument("不支持此像素类型");
    }
    result.GenImage1(type, data.cols, data.rows, data.data);
    return result;
}
HImage Tool::MatToHObjectDepth(const cv::Mat &image)
{
    if (image.channels() != 1)
        throw std::invalid_argument("深度图必须为单通道");
    return MatToHObject(image);
}
cv::Vec3d Tool::getPoint3DValue(const cv::Mat &depth, const cv::Point &pixel, const CameraIntrinsics &camera)
{
    camera.validate();
    if (pixel.x < 0 || pixel.y < 0 || pixel.x >= depth.cols || pixel.y >= depth.rows)
        throw std::out_of_range("采样像素超出图像范围");
    if (depth.empty() || depth.channels() != 1)
        throw std::invalid_argument("深度图必须为非空单通道");
    double raw = 0;
    switch (depth.depth())
    {
    case CV_16U:
        raw = depth.at<uint16_t>(pixel);
        break;
    case CV_16S:
        raw = depth.at<int16_t>(pixel);
        break;
    case CV_32F:
        raw = depth.at<float>(pixel);
        break;
    case CV_64F:
        raw = depth.at<double>(pixel);
        break;
    default:
        throw std::invalid_argument("深度图类型不支持");
    }
    double z = raw * camera.depthScale + camera.depthOffset;
    if (!std::isfinite(raw) || raw <= 0 || !std::isfinite(z) || z <= 0)
        throw std::invalid_argument("采样位置的深度无效");
    return {z * (pixel.x - camera.cx) / camera.fx, z * (pixel.y - camera.cy) / camera.fy, z};
}
std::vector<cv::Vec3d> Tool::get3DPoints(const cv::Mat &depth, const std::vector<cv::Point> &pixels,
                                         const CameraIntrinsics &camera)
{
    camera.validate();
    std::vector<cv::Vec3d> out;
    out.reserve(pixels.size());
    for (auto p : pixels)
        out.push_back(getPoint3DValue(depth, p, camera));
    return out;
}
HObjectModel3D Tool::depthImgToObjectModel3D(const cv::Mat &depth, const CameraIntrinsics &camera,
                                             const Bounds3D &bounds, const cv::Mat &mask)
{
    camera.validate();
    bounds.validate();
    auto raw = depth64(depth);
    if (!mask.empty() && (mask.size() != depth.size() || mask.type() != CV_8UC1))
        throw std::invalid_argument("掩膜尺寸或类型不匹配");
    cv::Mat x(depth.size(), CV_32F, cv::Scalar(0)), y = x.clone(), z = x.clone(),
                                                    valid = cv::Mat::zeros(depth.size(), CV_8U);
    for (int row = 0; row < depth.rows; ++row)
    {
        const auto *d = raw.ptr<double>(row);
        auto *xp = x.ptr<float>(row);
        auto *yp = y.ptr<float>(row);
        auto *zp = z.ptr<float>(row);
        auto *vp = valid.ptr<uchar>(row);
        const uchar *mp = mask.empty() ? nullptr : mask.ptr<uchar>(row);
        for (int col = 0; col < depth.cols; ++col)
        {
            if ((mp && !mp[col]) || !std::isfinite(d[col]) || d[col] <= 0)
                continue;
            double zz = d[col] * camera.depthScale + camera.depthOffset, xx = (col - camera.cx) * zz / camera.fx,
                   yy = (row - camera.cy) * zz / camera.fy;
            if (!std::isfinite(zz) || !std::isfinite(xx) || !std::isfinite(yy) || zz <= 0 || xx < bounds.min[0] ||
                xx > bounds.max[0] || yy < bounds.min[1] || yy > bounds.max[1] || zz < bounds.min[2] ||
                zz > bounds.max[2])
                continue;
            if (std::abs(xx) > std::numeric_limits<float>::max() || std::abs(yy) > std::numeric_limits<float>::max() ||
                zz > std::numeric_limits<float>::max())
                continue;
            xp[col] = float(xx);
            yp[col] = float(yy);
            zp[col] = float(zz);
            vp[col] = 255;
        }
    }
    if (cv::countNonZero(valid) == 0)
        return HObjectModel3D();
    HImage validImage = MatToHObjectDepth(valid);
    HRegion domain = validImage.Threshold(1, 255);
    // XYZ 图共享同一有效域，保留图像与点的映射，掩膜外的零值不成为空间点。
    return HObjectModel3D(MatToHObjectDepth(x).ReduceDomain(domain), MatToHObjectDepth(y).ReduceDomain(domain),
                          MatToHObjectDepth(z).ReduceDomain(domain));
}
HObjectModel3D Tool::depthImgToObjectModel3D_S(const cv::Mat &depth, CameraIntrinsics camera, const Bounds3D &bounds)
{
    camera.depthScale = 1;
    camera.depthOffset = 0;
    return depthImgToObjectModel3D(depth, camera, bounds);
}
HObjectModel3D Tool::filter(const HObjectModel3D &object, const Bounds3D &bounds)
{
    bounds.validate();
    auto out = object;
    const char *names[] = {"point_coord_x", "point_coord_y", "point_coord_z"};
    for (int i = 0; i < 3; ++i)
        if (std::isfinite(bounds.min[i]) || std::isfinite(bounds.max[i]))
            out = out.SelectPointsObjectModel3d(names[i], std::isfinite(bounds.min[i]) ? bounds.min[i] : -1e300,
                                                std::isfinite(bounds.max[i]) ? bounds.max[i] : 1e300);
    return out;
}
HObjectModel3D Tool::filterZ(const HObjectModel3D &object, double minZ, double maxZ)
{
    Bounds3D b;
    b.min[2] = minZ;
    b.max[2] = maxZ;
    return filter(object, b);
}
HTuple Tool::gripper2base(const HTuple &pose, const std::array<double, 6> &robotPose)
{
    checkPose(pose);
    for (auto v : robotPose)
        if (!std::isfinite(v))
            throw std::invalid_argument("机器人位姿非有限");
    HTuple robot, out;
    CreatePose(robotPose[0], robotPose[1], robotPose[2], robotPose[3], robotPose[4], robotPose[5], "Rp+T", "abg",
               "point", &robot);
    PoseCompose(robot, pose, &out);
    return abg(out);
}
HTuple Tool::transformPose(const HTuple &pose, const HTuple &cameraToGripper, const HTuple &targetOffset)
{
    checkPose(pose);
    HTuple combined = pose, mat, out;
    if (targetOffset.Length())
    {
        checkPose(targetOffset);
        PoseCompose(pose, targetOffset, &combined);
    }
    if (cameraToGripper.Length() != 12)
        throw std::invalid_argument("手眼矩阵必须为十二项");
    PoseToHomMat3d(combined, &mat);
    HomMat3dCompose(cameraToGripper, mat, &mat);
    HomMat3dToPose(mat, &out);
    return abg(out);
}
HTuple Tool::getHTarget2Gripper(const HTuple &target, const HTuple &base)
{
    checkPose(target);
    checkPose(base);
    HTuple inv, out;
    PoseInvert(base, &inv);
    PoseCompose(inv, target, &out);
    return abg(out);
}
HTuple Tool::transformPose2Target(const HTuple &pose, const HTuple &localTransform)
{
    checkPose(pose);
    HTuple offset, out;
    HomMat3dToPose(localTransform, &offset);
    PoseCompose(pose, offset, &out);
    return abg(out);
}
HTuple Tool::poseRotate(const HTuple &pose, const std::string &axis)
{
    checkPose(pose);
    HTuple out;
    if (axis == "y")
        return poseRotateYToZero(pose);
    if (axis != "x" && axis != "z")
        throw std::invalid_argument("旋转轴必须为 x/y/z");
    ConvertPoseType(pose, "Rp+T", axis == "x" ? "abg" : "gba", "point", &out);
    out[5] = 0.0;
    return abg(out);
}
HTuple Tool::poseRotateYToZero(const HTuple &pose)
{
    checkPose(pose);
    HTuple h, out;
    PoseToHomMat3d(pose, &h);
    double angle = std::atan2(-h[10].D(), h[8].D()) - CV_PI / 2;
    HomMat3dRotateLocal(h, angle, "y", &h);
    HomMat3dToPose(h, &out);
    return abg(out);
}
HTuple Tool::pose_translate_local(const HTuple &pose, const cv::Vec3d &translation)
{
    checkPose(pose);
    finitePoint(translation);
    HTuple h, out;
    PoseToHomMat3d(pose, &h);
    HomMat3dTranslateLocal(h, translation[0], translation[1], translation[2], &h);
    HomMat3dToPose(h, &out);
    return abg(out);
}
HObjectModel3D Tool::pointTransform(const HObjectModel3D &object, const HTuple &cameraToGripper,
                                    const std::array<double, 6> &robotPose, bool eyeInHand)
{
    HTuple p;
    HomMat3dToPose(cameraToGripper, &p);
    if (eyeInHand)
        p = gripper2base(p, robotPose);
    return object.RigidTransObjectModel3d(HPose(p));
}
HObjectModel3D Tool::pointCloudSplice(const std::vector<HObjectModel3D> &objects, const std::vector<HTuple> &transforms,
                                      const std::vector<std::array<double, 6>> &robotPoses, bool eyeInHand)
{
    if (objects.empty() || objects.size() != transforms.size() || objects.size() != robotPoses.size())
        throw std::invalid_argument("点云、手眼和拍照位姿列表数量须相同且非空");
    std::vector<HObjectModel3D> parts;
    for (size_t i = 0; i < objects.size(); ++i)
        parts.push_back(pointTransform(objects[i], transforms[i], robotPoses[i], eyeInHand));
    return HObjectModel3D::UnionObjectModel3d(HObjectModel3DArray(parts.data(), Hlong(parts.size())), "points_surface");
}
std::vector<int> Tool::getHighestModel(const HTuple &poses, const cv::Vec3d &catchPoint, const HTuple &cameraToGripper)
{
    checkPose(poses, true);
    finitePoint(catchPoint);
    std::vector<std::pair<int, double>> items;
    for (Hlong i = 0; i < poses.Length() / 7; ++i)
    {
        HTuple h, x, y, z;
        PoseToHomMat3d(transformPose(poses.TupleSelectRange(i * 7, i * 7 + 6), cameraToGripper), &h);
        AffineTransPoint3d(h, catchPoint[0], catchPoint[1], catchPoint[2], &x, &y, &z);
        items.emplace_back(int(i), z.D());
    }
    std::stable_sort(items.begin(), items.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
    std::vector<int> out;
    for (auto item : items)
        out.push_back(item.first);
    return out;
}
std::vector<int> Tool::getNoTouchModel(const HObjectModel3D &object, const HObjectModel3D &scene, const HTuple &poses,
                                       double distanceLimit, int touchLimit, bool rotationalSymmetry)
{
    checkPose(poses, true);
    if (!std::isfinite(distanceLimit) || distanceLimit <= 0 || touchLimit <= 0)
        throw std::invalid_argument("接触门限必须为正数");
    auto filtered = scene.SelectPointsObjectModel3d("neighbor_distance 10", 0, 1);
    std::vector<int> out;
    for (Hlong i = 0; i < poses.Length() / 7; ++i)
    {
        auto p = poses.TupleSelectRange(i * 7, i * 7 + 6);
        if (rotationalSymmetry)
            p = poseRotate(p, "z");
        auto moved = object.RigidTransObjectModel3d(HPose(p));
        moved.DistanceObjectModel3d(filtered, HPose(), HTuple(0), HTuple(), HTuple());
        auto distances = moved.GetObjectModel3dParams("&distance");
        int count = 0;
        for (Hlong j = 0; j < distances.Length(); ++j)
            if (distances[j].D() < distanceLimit)
                ++count;
        if (count < touchLimit)
            out.push_back(int(i));
    }
    return out;
}
Line3D Tool::FitLine(const std::vector<cv::Vec3d> &points)
{
    if (points.size() < 2)
        throw std::invalid_argument("直线拟合至少需要两个点");
    cv::Mat data(int(points.size()), 3, CV_64F);
    for (int i = 0; i < data.rows; ++i)
    {
        finitePoint(points[i]);
        for (int j = 0; j < 3; ++j)
            data.at<double>(i, j) = points[i][j];
    }
    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
    if (pca.eigenvalues.at<double>(0) <= 1e-20)
        throw std::invalid_argument("拟合点全部重合");
    Line3D out;
    for (int j = 0; j < 3; ++j)
    {
        out.point[j] = pca.mean.at<double>(0, j);
        out.direction[j] = pca.eigenvectors.at<double>(0, j);
    }
    if (out.direction.dot(points.front() - out.point) < 0)
        out.direction = -out.direction;
    return out;
}
std::vector<double> Tool::GetDistancesToLine(const std::vector<cv::Vec3d> &points, const Line3D &line)
{
    finitePoint(line.point);
    finitePoint(line.direction);
    double n = cv::norm(line.direction);
    if (n < 1e-15)
        throw std::invalid_argument("直线方向不能为零");
    std::vector<double> out;
    for (auto p : points)
    {
        finitePoint(p);
        out.push_back(cv::norm((p - line.point).cross(line.direction)) / n);
    }
    return out;
}
cv::Vec4d Tool::RotationMatrixToQuaternion(const cv::Matx33d &rotation)
{
    cv::Vec3d r;
    cv::Rodrigues(rotation, r);
    double angle = cv::norm(r);
    if (angle < 1e-15)
        return {0, 0, 0, 1};
    double scale = std::sin(angle / 2) / angle;
    return {r[0] * scale, r[1] * scale, r[2] * scale, std::cos(angle / 2)};
}
cv::Vec4d Tool::TransformPoseToQuaternions(const HTuple &pose)
{
    checkPose(pose);
    HTuple h;
    PoseToHomMat3d(pose, &h);
    cv::Matx33d r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r(i, j) = h[i * 4 + j].D();
    return RotationMatrixToQuaternion(r);
}
HTuple Tool::TransformQuaternionsToPose(const cv::Vec3d &translation, const cv::Vec4d &quaternion)
{
    finitePoint(translation);
    for (auto v : quaternion.val)
        if (!std::isfinite(v))
            throw std::invalid_argument("四元数非有限");
    double n = cv::norm(quaternion);
    if (!std::isfinite(n) || n < 1e-15)
        throw std::invalid_argument("四元数不能为零");
    auto q = quaternion / n;
    double x = q[0], y = q[1], z = q[2], w = q[3];
    double a[] = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),     translation[0],
                  2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),     translation[1],
                  2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y), translation[2]};
    HTuple h, out;
    for (auto v : a)
        h.Append(v);
    HomMat3dToPose(h, &out);
    return abg(out);
}
cv::Vec4d Tool::WeightedAverageQuaternions(const std::vector<cv::Vec4d> &quaternions,
                                           const std::vector<double> &weights)
{
    auto w = checkedWeights(quaternions.size(), weights);
    std::vector<cv::Vec4d> qs;
    for (auto q : quaternions)
    {
        for (auto v : q.val)
            if (!std::isfinite(v))
                throw std::invalid_argument("四元数非有限");
        double n = cv::norm(q);
        if (!std::isfinite(n) || n < 1e-15)
            throw std::invalid_argument("四元数退化");
        qs.push_back(q / n);
    }
    auto reference = qs[size_t(std::max_element(w.begin(), w.end()) - w.begin())];
    cv::Vec4d sum{0, 0, 0, 0};
    for (size_t i = 0; i < qs.size(); ++i)
        sum += qs[i] * (w[i] * (reference.dot(qs[i]) < 0 ? -1 : 1));
    double n = cv::norm(sum);
    if (n < 1e-15)
        throw std::invalid_argument("四元数平均退化");
    return sum / n;
}
HTuple Tool::WeightedAveragePoses(const std::vector<HTuple> &poses, const std::vector<double> &weights)
{
    auto w = checkedWeights(poses.size(), weights);
    cv::Vec3d t{0, 0, 0};
    std::vector<cv::Vec4d> qs;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        checkPose(poses[i]);
        qs.push_back(TransformPoseToQuaternions(poses[i]));
        for (int j = 0; j < 3; ++j)
            t[j] += poses[i][j].D() * w[i];
    }
    return TransformQuaternionsToPose(t, WeightedAverageQuaternions(qs, w));
}
double Tool::get_image_circle_z(const cv::Mat &depth, cv::Point2d center, double low, double high, double outerDiameter,
                                double innerDiameter)
{
    if (depth.empty() || depth.channels() != 1 || !std::isfinite(center.x) || !std::isfinite(center.y) ||
        !std::isfinite(low) || !std::isfinite(high) || low > high || !std::isfinite(outerDiameter) ||
        !std::isfinite(innerDiameter) || innerDiameter < 0 || outerDiameter <= innerDiameter)
        throw std::invalid_argument("圆环统计参数无效");
    HObject outer, inner, ring, reduced, region, opened, connected, largest;
    HTuple area, row, col, mean, deviation;
    auto image = MatToHObjectDepth(depth);
    GenCircle(&outer, center.y, center.x, outerDiameter / 2);
    if (innerDiameter > 0)
    {
        GenCircle(&inner, center.y, center.x, innerDiameter / 2);
        Difference(outer, inner, &ring);
    }
    else
        ring = outer;
    ReduceDomain(image, ring, &reduced);
    Threshold(reduced, &region, low, high);
    OpeningCircle(region, &opened, 3);
    Connection(opened, &connected);
    AreaCenter(connected, &area, &row, &col);
    if (!area.Length())
        throw std::runtime_error("圆环内没有有效深度区域");
    SelectObj(connected, &largest, area.TupleSortIndex()[area.Length() - 1].I() + 1);
    Intensity(largest, reduced, &mean, &deviation);
    return mean.D();
}
std::vector<cv::Rect> Tool::get_region_center(const HImage &image)
{
    HObject region, connected, selected;
    HTuple r1, c1, r2, c2;
    Threshold(image, &region, 0, 10);
    Connection(region, &connected);
    SelectShape(connected, &selected, "circularity", "and", 0.5, 1);
    SelectShape(selected, &selected, "area", "and", 100, 99999);
    SmallestRectangle1(selected, &r1, &c1, &r2, &c2);
    std::vector<cv::Rect> out;
    for (Hlong i = 0; i < r1.Length(); ++i)
        out.emplace_back(int(c1[i].D()), int(r1[i].D()), int(c2[i].D() - c1[i].D() + 1),
                         int(r2[i].D() - r1[i].D() + 1));
    return out;
}
void Tool::CalculatePlanePoseWithEuler(const std::vector<cv::Vec3d> &points, cv::Vec3d &position,
                                       cv::Vec3d &eulerDegrees)
{
    if (points.size() != 3)
        throw std::invalid_argument("平面定位必须输入三个点");
    for (auto p : points)
        finitePoint(p);
    auto x = points[1] - points[0], z = x.cross(points[2] - points[0]);
    if (cv::norm(x) < 1e-12 || cv::norm(z) < 1e-12)
        throw std::invalid_argument("平面定位点重合或共线");
    x /= cv::norm(x);
    z /= cv::norm(z);
    auto y = z.cross(x);
    position = (points[0] + points[1] + points[2]) / 3;
    HTuple h, p;
    for (int i = 0; i < 3; ++i)
    {
        h.Append(x[i]);
        h.Append(y[i]);
        h.Append(z[i]);
        h.Append(position[i]);
    }
    HomMat3dToPose(h, &p);
    p = abg(p);
    for (int i = 0; i < 3; ++i)
        eulerDegrees[i] = p[3 + i].D();
}
double Tool::GetAngleWithXoYPlaneSigned(const cv::Vec3d &vector)
{
    finitePoint(vector);
    if (cv::norm(vector) < 1e-15)
        throw std::invalid_argument("向量不能为零");
    return std::atan2(vector[2], std::hypot(vector[0], vector[1])) * 180 / CV_PI;
}
double Tool::GetAngleXWithY(const cv::Vec3d &vector)
{
    finitePoint(vector);
    if (cv::norm(vector) < 1e-15)
        throw std::invalid_argument("向量不能为零");
    return std::atan2(vector[1], vector[0]) * 180 / CV_PI;
}
std::array<std::vector<cv::Vec3d>, 2> Tool::SeparatePoints(std::vector<cv::Vec3d> points, int totalSize)
{
    if (totalSize < 4 || totalSize % 2 || points.size() != size_t(totalSize))
        throw std::invalid_argument("分列点数须为偶数且与配置一致");
    for (auto p : points)
        finitePoint(p);
    std::stable_sort(points.begin(), points.end(), [](auto a, auto b) { return a[0] < b[0]; });
    auto middle = points.begin() + totalSize / 2;
    std::array<std::vector<cv::Vec3d>, 2> out{std::vector<cv::Vec3d>(points.begin(), middle),
                                              std::vector<cv::Vec3d>(middle, points.end())};
    for (auto &column : out)
        std::stable_sort(column.begin(), column.end(), [](auto a, auto b) { return a[1] < b[1]; });
    return out;
}
HTuple Tool::mulPointToPose(const std::vector<cv::Vec3d> &points)
{
    size_t n = points.size();
    if (n != 1 && n != 2 && n != 3 && n != 4 && n != 8 && n != 10 && n != 12)
        throw std::invalid_argument("不支持该定位点数");
    for (auto p : points)
        finitePoint(p);
    cv::Vec3d position, angles{0, 0, 0};
    if (n == 1)
        position = points[0];
    else if (n == 3)
        CalculatePlanePoseWithEuler(points, position, angles);
    else
    {
        Line3D line;
        if (n <= 4)
        {
            auto sorted = points;
            std::stable_sort(sorted.begin(), sorted.end(), [](auto a, auto b) { return a[1] < b[1]; });
            line = FitLine(sorted);
        }
        else
        {
            auto columns = SeparatePoints(points, int(n));
            auto a = FitLine(columns[0]), b = FitLine(columns[1]);
            if (a.direction.dot(b.direction) < 0)
                b.direction = -b.direction;
            line.point = (a.point + b.point) / 2;
            line.direction = a.direction + b.direction;
        }
        position = line.point;
        angles = {0, -GetAngleWithXoYPlaneSigned(line.direction), -GetAngleXWithY(line.direction) - 90};
    }
    HTuple out;
    CreatePose(position[0], position[1], position[2], angles[0], angles[1], angles[2], "Rp+T", "abg", "point", &out);
    return out;
}
HTuple Tool::mulPointToPose2Coarse(const std::vector<cv::Vec3d> &points)
{
    return fixedPointPose(points, 2);
}
HTuple Tool::mulPointToPose3Coarse(const std::vector<cv::Vec3d> &points)
{
    return fixedPointPose(points, 3);
}
HTuple Tool::mulPointToPose1(const std::vector<cv::Vec3d> &points)
{
    return fixedPointPose(points, 1);
}
HTuple Tool::mulPointToPose2(const std::vector<cv::Vec3d> &points)
{
    return fixedPointPose(points, 2);
}
HTuple Tool::mulPointToPose4(const std::vector<cv::Vec3d> &points)
{
    return fixedPointPose(points, 4);
}
HTuple Tool::mulPointToPose8(const std::vector<cv::Vec3d> &points)
{
    return fixedPointPose(points, 8);
}
HTuple Tool::mulPointToPose10(const std::vector<cv::Vec3d> &points)
{
    return fixedPointPose(points, 10);
}
HTuple Tool::mulPointToPose12(const std::vector<cv::Vec3d> &points)
{
    return fixedPointPose(points, 12);
}
} // 命名空间 sm
