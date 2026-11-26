#include "surface_match/SurfaceMatch.h"
#include <cmath>
#include <iostream>
#include <filesystem>
#include <stdexcept>
using namespace HalconCpp;
namespace
{
/** @brief 失败即抛出含场景说明的测试异常，Release 构建也生效。
 * @param condition 必须满足的测试条件。
 * @param message 失败时显示的中文场景。
 * @throws std::runtime_error condition 为 false。 */
void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
/** @brief 比较浮点结果是否在容差内。
 * @param actual 实际值。
 * @param expected 期望值。
 * @param tolerance 最大绝对误差。
 * @param message 失败场景。
 * @throws std::runtime_error 非有限或误差超限。 */
void expectNear(double actual, double expected, double tolerance, const char *message)
{
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string(message) + "，实际=" + std::to_string(actual) +
                                 "，期望=" + std::to_string(expected));
}
/** @brief 创建测试用 abg pose。
 * @param x X 平移。
 * @param y Y 平移。
 * @param z Z 平移。
 * @param rz 绕 Z 角度，度，默认零。
 * @return 七元 HALCON pose。 */
HTuple pose(double x = 0, double y = 0, double z = 0, double rz = 0)
{
    HTuple p;
    CreatePose(x, y, z, 0, 0, rz, "Rp+T", "abg", "point", &p);
    return p;
}
/** @brief 检查 ROI 拷贝、像素反投影、无效深度、偏移背景与空点云分支。
 * @details 使用独立数值公式验证，不依赖实现函数生成预期结果。 */
void testDepthAndImages()
{
    sm::CameraIntrinsics c;
    c.fx = 2;
    c.fy = 4;
    c.cx = 1;
    c.cy = 1;
    c.depthScale = 0.5;
    c.depthOffset = 10;
    cv::Mat d(3, 4, CV_16U, cv::Scalar(100));
    d.at<uint16_t>(0, 0) = 0;
    auto p = sm::Tool::getPoint3DValue(d, {3, 2}, c);
    expectNear(p[0], 60, 1e-9, "反投影 X");
    expectNear(p[1], 15, 1e-9, "反投影 Y");
    expectNear(p[2], 60, 1e-9, "深度比例偏移");
    bool rejected = false;
    try
    {
        sm::Tool::getPoint3DValue(d, {4, 2}, c);
    }
    catch (const std::out_of_range &)
    {
        rejected = true;
    }
    require(rejected, "像素越界必须拒绝");
    auto object = sm::Tool::depthImgToObjectModel3D(d, c);
    require(object.GetObjectModel3dParams("num_points").L() == 11, "零深度不能被偏移变成空间点");
    cv::Mat mask = cv::Mat::zeros(d.size(), CV_8U);
    mask.at<uchar>(2, 3) = 255;
    auto one = sm::Tool::depthImgToObjectModel3D(d, c, {}, mask);
    require(one.GetObjectModel3dParams("num_points").L() == 1, "掩膜保留单点");
    expectNear(one.GetObjectModel3dParams("point_coord_x").D(), 60, 1e-5, "掩膜不改变原始像素坐标");
    cv::Mat zeros = cv::Mat::zeros(d.size(), d.type());
    require(sm::Tool::depthImgToObjectModel3D(zeros, c).GetObjectModel3dParams("num_points").L() == 0,
            "空深度生成空模型");
    cv::Mat invalid(2, 2, CV_32F, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    invalid.at<float>(0, 0) = std::numeric_limits<float>::infinity();
    require(sm::Tool::depthImgToObjectModel3D(invalid, c).GetObjectModel3dParams("num_points").L() == 0,
            "NaN 与 Inf 必须被剔除");
    auto roi = sm::Tool::LimitImage(d, cv::Rect(2, 1, 10, 10));
    require(cv::countNonZero(roi) == 4, "ROI 越界裁剪");
    auto noncontinuous = d(cv::Rect(1, 0, 2, 3));
    auto h = sm::Tool::MatToHObjectDepth(noncontinuous);
    d.setTo(0);
    expectNear(h.GetGrayval(1, 1).D(), 100, 0, "HALCON 图像必须独立于 Mat 内存");
    std::cout << "PASS 深度、掩膜、ROI、图像所有权\n";
}
/** @brief 检查 PCA、平面位姿、分列不丢点和四元数跨角度平均。 */
void testGeometry()
{
    auto line = sm::Tool::FitLine({{0, 0, 0}, {0, 1, 0}, {0, 2, 0}});
    expectNear(std::abs(line.direction[1]), 1, 1e-9, "PCA 主方向");
    auto ds = sm::Tool::GetDistancesToLine({{3, 1, 0}}, line);
    expectNear(ds[0], 3, 1e-9, "点到直线距离");
    auto columns = sm::Tool::SeparatePoints({{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {10, 3, 0}}, 4);
    require(columns[0].size() == 2 && columns[1].size() == 2, "不均匀分布仍必须保留所有点");
    cv::Vec3d position, angles;
    sm::Tool::CalculatePlanePoseWithEuler({{0, 0, 0}, {3, 0, 0}, {0, 3, 0}}, position, angles);
    expectNear(position[0], 1, 1e-9, "三点平面质心");
    expectNear(cv::norm(angles), 0, 1e-9, "三点平面朝向");
    auto avg = sm::Tool::WeightedAveragePoses({pose(1, 0, 0, 179), pose(3, 0, 0, -179)});
    expectNear(avg[0].D(), 2, 1e-9, "平移平均");
    auto q = sm::Tool::TransformPoseToQuaternions(avg);
    expectNear(std::abs(q[2]), 1, 1e-6, "跨 180 度四元数平均");
    bool rejected = false;
    try
    {
        sm::Tool::WeightedAveragePoses({pose()}, {0});
    }
    catch (const std::invalid_argument &)
    {
        rejected = true;
    }
    require(rejected, "零权重必须拒绝");
    rejected = false;
    try
    {
        sm::Tool::mulPointToPose({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}});
    }
    catch (const std::invalid_argument &)
    {
        rejected = true;
    }
    require(rejected, "共线点必须拒绝");
    std::cout << "PASS PCA、平面、分列、四元数\n";
}
/** @brief 验证原抓取链路的旋转杠杆臂、示教参考、控制器偏移及对称中心补偿。 */
void testGrasp()
{
    sm::GraspTransform c;
    c.cameraInBase = pose(100, 0, 0);
    c.taughtModelInBase = pose(100, 0, 0);
    c.taughtTcpInBase = pose(110, 0, 0);
    c.offsetReferenceInBase = c.taughtTcpInBase;
    auto g = sm::SurfaceMatch::transformPose(pose(0, 0, 0, 90), c);
    expectNear(g.tcpInBase[0].D(), 100, 1e-8, "模型旋转应改变 TCP X");
    expectNear(g.tcpInBase[1].D(), 10, 1e-8, "模型旋转应改变 TCP Y");
    c.mode = sm::OffsetMode::Fanuc;
    g = sm::SurfaceMatch::transformPose(pose(0, 0, 0, 90), c);
    expectNear(g.outputPose[0].D(), 100, 1e-8, "Fanuc 完整矩阵偏移 X");
    expectNear(g.outputPose[1].D(), -100, 1e-8, "Fanuc 完整矩阵偏移 Y");
    c.mode = sm::OffsetMode::Nachi;
    g = sm::SurfaceMatch::transformPose(pose(0, 0, 0, 90), c);
    expectNear(g.outputPose[0].D(), -10, 1e-8, "Nachi 直接位置差 X");
    expectNear(g.outputPose[1].D(), 10, 1e-8, "Nachi 直接位置差 Y");
    c.mode = sm::OffsetMode::Absolute;
    g = sm::SurfaceMatch::transformPose(pose(), c);
    expectNear(g.tcpInBase[0].D(), 110, 1e-8, "单位匹配保持示教 TCP");
    c.rotationalSymmetry = true;
    c.modelCenter = {10, 0, 0};
    g = sm::SurfaceMatch::transformPose(pose(0, 0, 0, 90), c);
    expectNear(g.tcpInBase[0].D(), 100, 1e-8, "对称补偿保持偏心模板中心 X");
    expectNear(g.tcpInBase[1].D(), 10, 1e-8, "对称补偿保持偏心模板中心 Y");
    expectNear(g.tcpInBase[5].D(), 0, 1e-8, "去除对称旋转");
    bool rejected = false;
    try
    {
        sm::SurfaceMatch::transformPose(HTuple(), c);
    }
    catch (const std::invalid_argument &)
    {
        rejected = true;
    }
    require(rejected, "空候选位姿必须拒绝");
    std::cout << "PASS 抓手转换、Fanuc/Nachi、对称中心补偿\n";
}
/** @brief 验证四参数相对偏移接口的换基、旋转杠杆臂、两种相机安装及输入边界。
 * @details 预期平移来自独立几何计算；验证零偏移不叠加标定平移，并验证返回矩阵能左乘参考 TCP。
 * 使用真实 HALCON 组合，不依赖模型、文件或机器人。
 * @throws std::runtime_error 几何结果或无效输入拒绝行为不符合约定。
 * @note 测试不分配外部句柄、不写文件，HALCON 运算失败继续向测试入口传播。 */
void testRelativeGrasp()
{
    auto fixedCamera = pose(100, 200, 50, 90);
    auto delta = sm::SurfaceMatch::transformPose(pose(), false, fixedCamera);
    for (int i = 0; i < 6; ++i)
        expectNear(std::remainder(delta[i].D(), 360.0), 0, 1e-8, "零偏移必须保持单位变换");
    delta = sm::SurfaceMatch::transformPose(pose(10, 20, 30), false, fixedCamera, nullptr);
    expectNear(delta[0].D(), -20, 1e-8, "固定相机旋转后的 X 偏移");
    expectNear(delta[1].D(), 10, 1e-8, "固定相机旋转后的 Y 偏移");
    expectNear(delta[2].D(), 30, 1e-8, "固定相机的 Z 偏移");
    delta = sm::SurfaceMatch::transformPose(pose(0, 0, 0, 90), false, pose(100, 0, 0));
    expectNear(delta[0].D(), 100, 1e-8, "绕相机原点旋转应补偿基座 X 杠杆臂");
    expectNear(delta[1].D(), -100, 1e-8, "绕相机原点旋转应补偿基座 Y 杠杆臂");
    expectNear(delta[5].D(), 90, 1e-8, "返回旋转应保留");
    HTuple tcp;
    PoseCompose(delta, pose(110, 0, 0), &tcp);
    expectNear(tcp[0].D(), 100, 1e-8, "相对偏移应左乘参考 TCP 的 X");
    expectNear(tcp[1].D(), 10, 1e-8, "相对偏移应左乘参考 TCP 的 Y");
    auto robot = pose(100, 20, 0, 90);
    delta = sm::SurfaceMatch::transformPose(pose(5, 0, 0), true, pose(10, 0, 0), &robot);
    expectNear(delta[0].D(), 0, 1e-8, "眼在手上应包含拍照机器人旋转 X");
    expectNear(delta[1].D(), 5, 1e-8, "眼在手上应包含拍照机器人旋转 Y");
    delta = sm::SurfaceMatch::transformPose(pose(0, 0, 0, 90), true, pose(10, 0, 0), &robot);
    expectNear(delta[0].D(), 130, 1e-8, "眼在手上相机原点应为 (100,30,0)，旋转补偿 X");
    expectNear(delta[1].D(), -70, 1e-8, "眼在手上相机原点应为 (100,30,0)，旋转补偿 Y");
    HTuple handEye;
    CreatePose(10, 20, 30, 0, 90, 0, "Rp+T", "abg", "point", &handEye);
    CreatePose(50, -30, 80, 90, 0, 0, "Rp+T", "abg", "point", &robot);
    delta = sm::SurfaceMatch::transformPose(pose(10, 20, 30), true, handEye, &robot);
    // Ry(90) 后再 Rx(90)：(10,20,30) -> (30,20,-10) -> (30,10,20)。
    expectNear(delta[0].D(), 30, 1e-8, "三维手眼和机器人旋转组合的 X");
    expectNear(delta[1].D(), 10, 1e-8, "三维手眼和机器人旋转组合的 Y");
    expectNear(delta[2].D(), 20, 1e-8, "三维手眼和机器人旋转组合的 Z");
    HTuple empty;
    delta = sm::SurfaceMatch::transformPose(pose(1, 2, 3), false, pose(), &empty);
    expectNear(delta[0].D(), 1, 1e-8, "固定相机必须忽略未使用的机器人参数");
    int rejected = 0;
    try
    {
        sm::SurfaceMatch::transformPose(pose(), true, pose(), nullptr);
    }
    catch (const std::invalid_argument &)
    {
        ++rejected;
    }
    try
    {
        sm::SurfaceMatch::transformPose(pose(), true, pose(), &empty);
    }
    catch (const std::invalid_argument &)
    {
        ++rejected;
    }
    try
    {
        sm::SurfaceMatch::transformPose(pose(), false, empty);
    }
    catch (const std::invalid_argument &)
    {
        ++rejected;
    }
    try
    {
        sm::SurfaceMatch::transformPose(pose().TupleConcat(pose()), false, pose());
    }
    catch (const std::invalid_argument &)
    {
        ++rejected;
    }
    auto nonFinite = pose();
    nonFinite[0] = std::numeric_limits<double>::quiet_NaN();
    try
    {
        sm::SurfaceMatch::transformPose(nonFinite, false, pose());
    }
    catch (const std::invalid_argument &)
    {
        ++rejected;
    }
    require(rejected == 5, "缺失机器人、空位姿、批量匹配和非有限输入必须拒绝");
    std::cout << "PASS 四参数相对抓手转换、眼在手上/手外、三维旋转及无效输入\n";
}
/** @brief 用不对称合成曲面训练并验证真实 HALCON 表面匹配及 runModel 深度主流程。
 * @details 模板和场景相同，因此已知真值是单位变换；不模拟 FindSurfaceModel 返回值。 */
void testSurface()
{
    sm::CameraIntrinsics c;
    c.fx = c.fy = 220;
    c.cx = 80;
    c.cy = 60;
    cv::Mat depth = cv::Mat::zeros(120, 160, CV_32F);
    for (int y = 15; y < 105; ++y)
        for (int x = 20; x < 140; ++x)
            depth.at<float>(y, x) =
                float(400 + 45 * std::exp(-((x - 65.0) * (x - 65.0) / 180 + (y - 45.0) * (y - 45.0) / 120)) +
                      28 * std::exp(-((x - 112.0) * (x - 112.0) / 80 + (y - 82.0) * (y - 82.0) / 200)) +
                      (x < 48 && y > 65 ? 32 : 0) + 0.15 * x + 0.04 * y);
    auto object = sm::Tool::depthImgToObjectModel3D(depth, c);
    sm::MatchParameters p;
    p.numMatches = 1;
    p.minScore = 0.6;
    p.modelSampling = 0.02;
    p.sceneSampling = 0.02;
    p.keyPointFraction = 0.3;
    auto model = sm::SurfaceMatch::initModel3D(object, p);
    auto result = sm::SurfaceMatch::runModel(*model, {}, depth, c, {}, p);
    require(result.scores.Length() == 1, "真实 runModel 应找到合成模板");
    expectNear(result.poses[0].D(), 0, 1, "单位变换 X");
    expectNear(result.poses[1].D(), 0, 1, "单位变换 Y");
    expectNear(result.poses[2].D(), 0, 1, "单位变换 Z");
    HTuple movement = pose(12, -7, 4, 6);
    auto scene = object.RigidTransObjectModel3d(HPose(movement));
    auto translated = sm::SurfaceMatch::run3DModel(*model, scene, p);
    require(translated.scores.Length() == 1, "旋转平移场景应匹配");
    expectNear(translated.poses[0].D(), 12, 1, "非单位匹配 X");
    expectNear(translated.poses[1].D(), -7, 1, "非单位匹配 Y");
    expectNear(translated.poses[2].D(), 4, 1, "非单位匹配 Z");
    auto empty = sm::SurfaceMatch::run3DModel(*model, HObjectModel3D(), p);
    require(empty.scores.Length() == 0, "空场景必须返回空结果");
    sm::SurfaceMatch::saveModel(*model, "test_roundtrip.sfm");
    auto loaded = sm::SurfaceMatch::initModel("test_roundtrip.sfm", p);
    require(loaded->object.GetObjectModel3dParams("num_points").L() > 0, "SFM 往返保留模型点云");
    std::cout << "PASS 实际 HALCON 建模、runModel、已知非单位运动、空场景、SFM 往返\n";
}
/** @brief 验证指定原生 SDK 的真实检测与深度滤波调用。
 * @param runtime RaiVas DLL 完整运行目录，包含 ONNX/OpenVINO 依赖。
 * @param data 模型与测试图像目录，包含 det/best.onnx 和 obb/yolo11s-obb_leike.onnx。
 * @param depthRuntime 原生深度滤波 DLL 目录。
 * @details 实际调用原始头文件定义的 OBB/Det 两个驱动，不构造模拟 DLL。 */
void testNative(const std::string &runtime, const std::string &data, const std::string &depthRuntime)
{
    auto root = std::filesystem::u8path(data);
    for (auto driver : {sm::DetectorDriver::Onnx, sm::DetectorDriver::OpenVino})
        for (auto kind : {sm::DetectorKind::Obb, sm::DetectorKind::Detection})
        {
            sm::DetectorOptions options;
            options.runtimeDirectory = runtime;
            options.driver = driver;
            options.kind = kind;
            options.threshold = 0.25;
            options.modelPath =
                (root / (kind == sm::DetectorKind::Obb ? "obb/yolo11s-obb_leike.onnx" : "det/best.onnx")).u8string();
            auto image = sm::Tool::readImage(
                (root / (kind == sm::DetectorKind::Obb ? "obb/TEST1.png" : "det/3.jpg")).u8string(), false);
            auto detections = sm::NativeBackend::run2DModel(image, options);
            require(!detections.empty(), "原生检测应在配套测试图中找到目标");
            for (const auto &d : detections)
                require(d.score > 0 && d.box.size.area() > 0, "原生检测结果字段须有效");
            std::cerr << "PASS 原生后端 driver=" << int(driver) << " kind=" << int(kind)
                      << " count=" << detections.size() << '\n';
        }
    sm::CameraIntrinsics c;
    cv::Mat depth(40, 40, CV_32F, cv::Scalar(500));
    depth.at<float>(20, 20) = 900;
    auto filtered = sm::NativeBackend::DepthRadiusFilter(depth, c, depthRuntime, 5, 4, 1);
    require(filtered.size() == depth.size() && filtered.type() == CV_32F, "原生滤波保持坐标映射");
    expectNear(filtered.at<float>(10, 10), 500, 0.01, "原生滤波保留平面");
    require(filtered.at<float>(20, 20) <= 0 || !std::isfinite(filtered.at<float>(20, 20)),
            "原生滤波应去除孤立异常深度");
    std::cerr << "PASS 原生 DepthRadiusFilter\n";
}
} // 匿名命名空间
/** @brief 依次运行数值与 HALCON 集成验证，任何失败返回非零状态。
 * @param argc 系统参数数量；可选 --native 后接 SDK 目录、数据目录和深度滤波目录。
 * @param argv 调用期间有效的系统参数列表。
 * @return 0 全部通过，1 标准异常，2 HALCON 运行错误（包含许可证错误）。 */
int main(int argc, char *argv[])
{
    try
    {
        if (argc == 5 && std::string(argv[1]) == "--native")
            testNative(argv[2], argv[3], argv[4]);
        else
        {
            testDepthAndImages();
            testGeometry();
            testGrasp();
            testRelativeGrasp();
            testSurface();
        }
        std::cout << "全部算法验证通过\n";
        return 0;
    }
    catch (const HException &e)
    {
        std::cerr << "HALCON " << e.ErrorCode() << " / " << e.ProcName().Text() << ": " << e.ErrorMessage().Text()
                  << '\n';
        return 2;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
