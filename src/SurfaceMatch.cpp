#include "surface_match/SurfaceMatch.h"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>
namespace sm
{
using namespace HalconCpp;
namespace
{
/** @brief 校验并统一七元位姿。
 * @param p 有限七元 pose。
 * @return abg 位姿。
 * @throws std::invalid_argument 数量或数值无效。 */
HTuple normalized(const HTuple &p)
{
    if (p.Length() != 7)
        throw std::invalid_argument("转换必须使用单个七元 HALCON pose");
    for (Hlong i = 0; i < 7; ++i)
        if (!std::isfinite(p[i].D()))
            throw std::invalid_argument("位姿含有非有限值");
    HTuple out;
    ConvertPoseType(p, "Rp+T", "abg", "point", &out);
    return out;
}
/** @brief 按左乘顺序组合两个 pose，统一输出 abg。
 * @param left 左侧参考变换。
 * @param right 右侧局部变换。
 * @return left*right。 */
HTuple compose(const HTuple &left, const HTuple &right)
{
    auto l = normalized(left), r = normalized(right);
    HTuple out;
    PoseCompose(l, r, &out);
    return normalized(out);
}
/** @brief 计算位姿逆变换。
 * @param p 七元 pose。
 * @return 归一化逆 pose。 */
HTuple inverse(const HTuple &p)
{
    HTuple out;
    PoseInvert(normalized(p), &out);
    return normalized(out);
}
/** @brief 变换模型空间点，用于保持对称补偿前后的实际中心位置。
 * @param pose 七元空间变换。
 * @param point 有限模型 XYZ。
 * @return 变换后 XYZ。 */
cv::Vec3d movePoint(const HTuple &pose, const cv::Vec3d &point)
{
    for (double v : point.val)
        if (!std::isfinite(v))
            throw std::invalid_argument("模型中心非有限");
    HTuple h, x, y, z;
    PoseToHomMat3d(pose, &h);
    AffineTransPoint3d(h, point[0], point[1], point[2], &x, &y, &z);
    return {x.D(), y.D(), z.D()};
}
} // 匿名命名空间
void SurfaceMatch::setPara(MatchParameters &parameters, MatchingAccuracy accuracy)
{
    auto p = parameters;
    switch (accuracy)
    {
    case MatchingAccuracy::Low:
        p.modelSampling = p.sceneSampling = 0.03;
        p.keyPointFraction = 0.1;
        break;
    case MatchingAccuracy::Normal:
        p.modelSampling = p.sceneSampling = 0.02;
        p.keyPointFraction = 0.3;
        break;
    case MatchingAccuracy::High:
        p.modelSampling = p.sceneSampling = 0.01;
        p.keyPointFraction = 0.2;
        break;
    case MatchingAccuracy::VeryHigh:
        p.modelSampling = p.sceneSampling = 0.008;
        p.keyPointFraction = 0.2;
        break;
    default:
        throw std::invalid_argument("无效的精度档位");
    }
    p.validate();
    parameters = p;
}
std::shared_ptr<const ModelData> SurfaceMatch::initModel3D(const HObjectModel3D &object,
                                                           const MatchParameters &parameters)
{
    parameters.validate();
    if (object.GetObjectModel3dParams("num_points").L() < 3)
        throw std::invalid_argument("模板至少需要三个空间点");
    auto model = std::make_shared<ModelData>();
    model->object = object;
    if (parameters.modelSmoothK)
        model->object = model->object.SmoothObjectModel3d("mls", "mls_kNN", parameters.modelSmoothK);
    // 深度模板保留像素映射并按相机视线定向法线；CAD 已有法线则保留其方向。
    // 任意重算 MLS 法线会破坏与场景法线的朝向一致性，可能导致翻面误匹配。
    if (std::string(model->object.GetObjectModel3dParams("has_xyz_mapping").S().Text()) == "true")
        model->object = model->object.SurfaceNormalsObjectModel3d("xyz_mapping", HTuple(), HTuple());
    else if (std::string(model->object.GetObjectModel3dParams("has_point_normals").S().Text()) != "true")
        model->object = model->object.SurfaceNormalsObjectModel3d("mls", "mls_force_inwards", "true");
    model->surface = HSurfaceModel(model->object, parameters.modelSampling, HTuple(), HTuple());
    return model;
}
std::shared_ptr<const ModelData> SurfaceMatch::initModel(const std::string &path, const MatchParameters &parameters,
                                                         const std::string &fileScale)
{
    parameters.validate();
    if (path.empty())
        throw std::invalid_argument("模型路径为空");
    auto extension = std::filesystem::u8path(path).extension().u8string();
    for (auto &c : extension)
        c = char(std::tolower(static_cast<unsigned char>(c)));
    if (extension == ".sfm")
    {
        auto model = std::make_shared<ModelData>();
        model->surface = HSurfaceModel(path.c_str());
        HTuple handle = model->surface.GetSurfaceModelParam("sampled_model");
        model->object = HObjectModel3D(handle[0].H());
        return model;
    }
    HTuple status;
    HObjectModel3D object(HString(path.c_str()), HTuple(fileScale.c_str()), HTuple(), HTuple(), &status);
    return initModel3D(object, parameters);
}
void SurfaceMatch::saveModel(const ModelData &model, const std::string &path)
{
    model.surface.WriteSurfaceModel(path.c_str());
}
MatchResult SurfaceMatch::run3DModel(const ModelData &model, const HObjectModel3D &scene,
                                     const MatchParameters &parameters)
{
    parameters.validate();
    MatchResult out;
    if (scene.GetObjectModel3dParams("num_points").L() == 0)
        return out;
    auto processed = scene;
    if (parameters.downsampleDistance > 0)
        processed = processed.SampleObjectModel3d("fast", parameters.downsampleDistance, HTuple(), HTuple());
    if (parameters.sceneSmoothK)
        processed = processed.SmoothObjectModel3d("mls", "mls_kNN", parameters.sceneSmoothK);
    HTuple names, values;
    names.Append("num_matches");
    values.Append(parameters.numMatches);
    names.Append("max_overlap_dist_rel");
    values.Append(parameters.maxOverlapDistance);
    names.Append("scene_normal_computation");
    values.Append(parameters.sceneNormals.c_str());
    names.Append("pose_ref_scoring_dist_rel");
    values.Append(parameters.poseRefScoringDistance);
    HSurfaceMatchingResultArray debug;
    auto poses = model.surface.FindSurfaceModel(processed, parameters.sceneSampling, parameters.keyPointFraction,
                                                HTuple(parameters.minScore), HString("false"), names, values,
                                                &out.scores, &debug);
    out.poses = poses.ConvertToTuple();
    out.radians2D.resize(size_t(out.scores.Length()), 0);
    out.scenes.push_back(processed);
    return out;
}
MatchResult SurfaceMatch::runModel(const ModelData &model, const cv::Mat &image, const cv::Mat &depth,
                                   const CameraIntrinsics &camera, const Bounds3D &bounds,
                                   const MatchParameters &parameters, const DetectorOptions *detector, int attitudeId)
{
    const auto start = std::chrono::steady_clock::now();
    parameters.validate();
    camera.validate();
    bounds.validate();
    MatchResult out;
    if (!detector)
        out = run3DModel(model, Tool::depthImgToObjectModel3D(depth, camera, bounds), parameters);
    else
    {
        if (image.size() != depth.size())
            throw std::invalid_argument("2D 与深度图必须已经配准且尺寸一致");
        out.detections = NativeBackend::run2DModel(image, *detector);
        for (const auto &d : out.detections)
        {
            if (attitudeId >= 0 && d.classId != attitudeId)
                continue;
            // 保留原先的保守边界判定：按最远 Z 投影框角，整框须落在可检测 XY 范围内。
            cv::Point2f originalCorners[4];
            d.box.points(originalCorners);
            bool inside = true;
            if (std::isfinite(bounds.max[2]))
                for (auto corner : originalCorners)
                {
                    double x = bounds.max[2] * (corner.x - camera.cx) / camera.fx,
                           y = bounds.max[2] * (corner.y - camera.cy) / camera.fy;
                    if (x < bounds.min[0] || x > bounds.max[0] || y < bounds.min[1] || y > bounds.max[1])
                    {
                        inside = false;
                        break;
                    }
                }
            if (!inside)
                continue;
            auto expanded = d.box;
            expanded.size.width += 2 * parameters.roiExpansion;
            expanded.size.height += 2 * parameters.roiExpansion;
            cv::Point2f corners[4];
            expanded.points(corners);
            std::vector<cv::Point> polygon;
            for (auto c : corners)
                polygon.emplace_back(cvRound(c.x), cvRound(c.y));
            cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8U);
            cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
            if (!d.mask.empty())
            {
                if (d.mask.type() != CV_8UC1 || d.mask.size() != depth.size())
                    throw std::runtime_error("分割掩膜与深度图不匹配");
                cv::bitwise_and(mask, d.mask, mask);
            }
            auto p = parameters;
            p.numMatches = 1;
            auto part = run3DModel(model, Tool::depthImgToObjectModel3D(depth, camera, bounds, mask), p);
            if (!part.scores.Length())
                continue;
            if (d.hasOrientation)
            {
                auto pose = normalized(part.poses.TupleSelectRange(0, 6));
                double difference = std::abs(std::remainder(pose[5].D() - d.radians * 180 / CV_PI, 360.0));
                if (difference >= parameters.angleTolerance)
                    continue;
            }
            out.poses = out.poses.TupleConcat(part.poses);
            out.scores = out.scores.TupleConcat(part.scores);
            out.radians2D.push_back(d.radians);
            out.scenes.insert(out.scenes.end(), part.scenes.begin(), part.scenes.end());
            if (out.scores.Length() >= parameters.numMatches)
                break;
        }
    }
    out.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return out;
}
HTuple SurfaceMatch::transformPose(const HTuple &matchPose, bool eyeInHand, const HTuple &handEyePose,
                                   const HTuple *robotPose)
{
    auto cameraInBase = normalized(handEyePose);
    if (eyeInHand)
    {
        if (!robotPose)
            throw std::invalid_argument("眼在手上必须提供拍照时的机器人位姿");
        cameraInBase = compose(*robotPose, cameraInBase);
    }
    // 偏移是同一空间运动的换基，必须右乘相机位姿的逆；仅左乘会把标定位置错误地叠加到零偏移。
    return compose(compose(cameraInBase, matchPose), inverse(cameraInBase));
}
GraspResult SurfaceMatch::transformPose(const HTuple &matchPose, const GraspTransform &c)
{
    GraspResult out;
    auto current = compose(c.cameraInBase, matchPose);
    auto reference = normalized(c.taughtModelInBase);
    auto refInv = inverse(reference);
    if (c.rotationalSymmetry)
    {
        auto local = compose(refInv, current), adjusted = Tool::poseRotate(local, "z");
        // 去除对称旋转时保持模型中心实际位置，否则偏心模板会产生错误的 TCP 平移。
        auto difference = movePoint(local, c.modelCenter) - movePoint(adjusted, c.modelCenter);
        for (int i = 0; i < 3; ++i)
            adjusted[i] = adjusted[i].D() + difference[i];
        current = compose(reference, adjusted);
    }
    out.modelInBase = current;
    out.motionInBase = compose(current, refInv);
    out.tcpInBase = compose(out.motionInBase, c.taughtTcpInBase);
    if (c.mode == OffsetMode::Absolute)
        out.outputPose = out.tcpInBase;
    else if (c.mode == OffsetMode::Fanuc || c.mode == OffsetMode::Nachi)
    {
        auto origin = normalized(c.offsetReferenceInBase);
        out.outputPose = compose(out.tcpInBase, inverse(origin));
        if (c.mode == OffsetMode::Nachi)
            for (int i = 0; i < 3; ++i)
                out.outputPose[i] = out.tcpInBase[i].D() - origin[i].D();
    }
    else
        throw std::invalid_argument("无效的抓手偏移输出模式");
    return out;
}
HShapeModel SurfaceMatch::initShapeModel(const std::string &path)
{
    return HShapeModel(path.c_str());
}
HShapeModel SurfaceMatch::initModel2D(const std::string &path)
{
    return initShapeModel(path);
}
std::vector<Detection2D> SurfaceMatch::findShapeModel(const HShapeModel &model, const cv::Mat &image, double threshold,
                                                      int numMatches)
{
    if (image.empty() || image.depth() != CV_8U || !std::isfinite(threshold) || threshold < 0 || threshold > 1 ||
        numMatches <= 0)
        throw std::invalid_argument("形状匹配输入无效");
    cv::Mat gray;
    if (image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else if (image.channels() == 4)
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
    else
        throw std::invalid_argument("不支持的图像通道数");
    HObject mean;
    auto h = Tool::MatToHObject(gray);
    MeanImage(h, &mean, 9, 9);
    HTuple rows, columns, angles, scales, scores;
    model.FindScaledShapeModel(HImage(mean), -CV_PI, 2 * CV_PI, 0.8, 1.2, threshold, numMatches, 0.5,
                               "least_squares_high", 0, 0.4, &rows, &columns, &angles, &scales, &scores);
    std::vector<Detection2D> out;
    for (Hlong i = 0; i < scores.Length(); ++i)
    {
        Detection2D d;
        d.score = float(scores[i].D());
        d.box =
            cv::RotatedRect({float(columns[i].D()), float(rows[i].D())}, {1, 1}, float(angles[i].D() * 180 / CV_PI));
        d.radians = float(angles[i].D());
        d.hasOrientation = true;
        out.push_back(d);
    }
    return out;
}
LocationResult SurfaceMatch::locationImage(const cv::Mat &depth, const std::vector<Detection2D> &detections,
                                           const CameraIntrinsics &camera, const Bounds3D &bounds, double outerDiameter,
                                           double innerDiameter, double grayLow, double grayHigh, int attitudeId)
{
    camera.validate();
    bounds.validate();
    LocationResult out;
    for (const auto &d : detections)
    {
        if (attitudeId >= 0 && d.classId != attitudeId)
            continue;
        double raw = Tool::get_image_circle_z(depth, d.box.center, grayLow, grayHigh, outerDiameter, innerDiameter);
        double z = raw * camera.depthScale + camera.depthOffset;
        cv::Vec3d p{(d.box.center.x - camera.cx) * z / camera.fx, (d.box.center.y - camera.cy) * z / camera.fy, z};
        bool valid = raw > 0 && z > 0;
        for (int i = 0; i < 3; ++i)
            valid = valid && std::isfinite(p[i]) && p[i] >= bounds.min[i] && p[i] <= bounds.max[i];
        if (!valid)
            continue;
        HTuple pose;
        CreatePose(p[0], p[1], p[2], 0, 0, 0, "Rp+T", "abg", "point", &pose);
        out.poses = out.poses.TupleConcat(pose);
        out.scores.Append(d.score);
        out.detections.push_back(d);
    }
    return out;
}
LocationResult SurfaceMatch::coarseLocation(const std::vector<LocationResult> &parts,
                                            const std::vector<HTuple> &cameraInBase)
{
    if (parts.size() != 3 || cameraInBase.size() != 3)
        throw std::invalid_argument("粗定位必须提供三帧中心及相机到基座位姿");
    LocationResult out;
    std::vector<cv::Vec3d> points;
    for (size_t i = 0; i < 3; ++i)
    {
        if (parts[i].poses.Length() != 7 || parts[i].scores.Length() != 1)
            throw std::invalid_argument("每帧粗定位必须恰好得到一个中心");
        auto p = compose(cameraInBase[i], parts[i].poses);
        points.emplace_back(p[0].D(), p[1].D(), p[2].D());
        out.poses = out.poses.TupleConcat(parts[i].poses);
        out.scores = out.scores.TupleConcat(parts[i].scores);
        out.detections.insert(out.detections.end(), parts[i].detections.begin(), parts[i].detections.end());
    }
    out.combinedPose = Tool::mulPointToPose(points);
    return out;
}
LocationResult SurfaceMatch::fineLocation(const LocationResult &parts)
{
    if (parts.poses.Length() != parts.scores.Length() * 7 || parts.scores.Length() == 3)
        throw std::invalid_argument("精定位位姿数量无效");
    std::vector<cv::Vec3d> points;
    for (Hlong i = 0; i < parts.scores.Length(); ++i)
        points.emplace_back(parts.poses[i * 7].D(), parts.poses[i * 7 + 1].D(), parts.poses[i * 7 + 2].D());
    auto out = parts;
    out.combinedPose = Tool::mulPointToPose(points);
    return out;
}
MatchResult SurfaceMatch::runMulObjModel(const ModelData &model, const std::vector<HObjectModel3D> &scenes,
                                         const std::vector<HTuple> &transforms, const MatchParameters &parameters,
                                         bool mergeFirst)
{
    if (scenes.empty() || scenes.size() != transforms.size())
        throw std::invalid_argument("多帧点云与变换数量不一致");
    parameters.validate();
    std::vector<HObjectModel3D> transformed;
    for (size_t i = 0; i < scenes.size(); ++i)
    {
        HTuple p;
        HomMat3dToPose(transforms[i], &p);
        transformed.push_back(scenes[i].RigidTransObjectModel3d(HPose(p)));
    }
    if (mergeFirst)
    {
        auto merged = HObjectModel3D::UnionObjectModel3d(
            HObjectModel3DArray(transformed.data(), Hlong(transformed.size())), "points_surface");
        return run3DModel(model, merged, parameters);
    }
    std::vector<HTuple> poses;
    double score = 0;
    MatchResult out;
    auto p = parameters;
    p.numMatches = 1;
    for (const auto &scene : transformed)
    {
        auto part = run3DModel(model, scene, p);
        if (!part.scores.Length())
            return {};
        poses.push_back(part.poses);
        score += part.scores[0].D();
        out.scenes.insert(out.scenes.end(), part.scenes.begin(), part.scenes.end());
    }
    out.poses = Tool::WeightedAveragePoses(poses);
    out.scores = score / poses.size();
    out.radians2D.push_back(0);
    return out;
}
int SurfaceMatch::getHighestModel(const HObjectModel3D &object, const HTuple &poses)
{
    if (poses.Length() % 7)
        throw std::invalid_argument("匹配位姿长度必须为七的倍数");
    int index = -1;
    double minZ = std::numeric_limits<double>::infinity();
    for (Hlong i = 0; i < poses.Length() / 7; ++i)
    {
        auto moved = object.RigidTransObjectModel3d(HPose(poses.TupleSelectRange(i * 7, i * 7 + 6)));
        auto z = moved.GetObjectModel3dParams("point_coord_z");
        if (z.Length() && z.TupleMin().D() < minZ)
        {
            minZ = z.TupleMin().D();
            index = int(i);
        }
    }
    return index;
}
} // 命名空间 sm
