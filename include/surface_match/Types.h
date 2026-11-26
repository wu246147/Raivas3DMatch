#pragma once
#include <HalconCpp.h>
#include <opencv2/core.hpp>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace sm
{
/** @brief 针孔相机内参与深度换算配置，供图像反投影使用。
 * @details 输入图像须已去畸变，2D 与深度图须配准。所有长度统一到调用方选择的单位；
 * z=raw*depthScale+depthOffset，模型、滤波范围、机器人平移必须使用同一单位。
 * 配置是值类型，可复制；运行时不允许另一线程同时修改同一实例。 */
struct CameraIntrinsics
{
    double fx = 1000;       ///< X 方向焦距，像素，有限且大于零。
    double fy = 1000;       ///< Y 方向焦距，像素，有限且大于零。
    double cx = 0;          ///< 主点列坐标，像素，有限值。
    double cy = 0;          ///< 主点行坐标，像素，有限值。
    double depthScale = 1;  ///< 原始深度比例，有限且大于零，直接深度图使用 1。
    double depthOffset = 0; ///< 深度偏移，单位与输出点云一致。
    /** @brief 校验内参与深度变换，不修改配置。
     * @throws std::invalid_argument 焦距、比例不为正或任一字段非有限值。 */
    void validate() const;
};
/** @brief 相机坐标系的闭区间裁剪盒，长度单位与点云一致。
 * @details 默认不裁剪有限 X/Y，Z 必须为正；每轴下界不得大于上界，不允许 NaN。
 * 纯值类型，不持有外部资源。 */
struct Bounds3D
{
    cv::Vec3d min{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                  1e-6}; ///< X/Y/Z 下界。
    cv::Vec3d max{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                  9999}; ///< X/Y/Z 上界。
    /** @brief 校验各轴上下界的顺序和 NaN。
     * @throws std::invalid_argument 任一上下界无效。 */
    void validate() const;
};
/** @brief 匹配精度档位，控制训练/场景的相对采样间距和关键点比例。
 * @details 默认 Normal，数值与原 C# 枚举对应，不作为位标志组合。 */
enum class MatchingAccuracy
{
    Low = 0,     ///< 采样间距 0.03，关键点比例 0.1。
    Normal = 1,  ///< 默认档位：采样间距 0.02，关键点比例 0.3。
    High = 2,    ///< 采样间距 0.01，关键点比例 0.2。
    VeryHigh = 3 ///< 采样间距 0.008，关键点比例 0.2。
};
/** @brief 候选抓取排序策略，具体方向由坐标系决定。
 * @details Highest 对应机器人 Z 从大到小，Best 对应匹配分数从高到低，NoTouch 进行接触筛选。 */
enum class MatchType
{
    Highest = 0, ///< 按机器人 Z 高度降序。
    Best = 1,    ///< 按匹配分数降序。
    NoTouch = 2  ///< 对候选进行接触筛选。
};
/** @brief 表面匹配参数；不持有模型，可在不同任务中按值复制。
 * @details 修改训练采样或模型平滑参数后需要重新建模。模型法线统一由 HALCON 计算。
 * 所有绝对距离与点云同单位，相对距离是模型包围盒对角线的比例。 */
struct MatchParameters
{
    int numMatches = 10;                  ///< 最多匹配个数，必须大于零。
    double minScore = 0.6;                ///< 最低分数，[0,1]。
    double maxOverlapDistance = 0.2;      ///< 重叠抑制相对距离，非负。
    double modelSampling = 0.02;          ///< 模型相对采样间距，(0,1]。
    double sceneSampling = 0.02;          ///< 场景相对采样间距，(0,1]。
    double keyPointFraction = 0.3;        ///< 场景关键点比例，(0,1]。
    double poseRefScoringDistance = 0.02; ///< 位姿精化评分的相对距离，大于零。
    int modelSmoothK = 0;                 ///< 模型 MLS 邻居数，0 关闭，否则至少 3。
    int sceneSmoothK = 0;                 ///< 场景 MLS 邻居数，0 关闭，否则至少 3。
    double downsampleDistance = 0;        ///< 场景降采样绝对距离，0 关闭。
    int roiExpansion = 20;                ///< 2D 候选框向外扩张量，像素，非负。
    double angleTolerance = 45;           ///< OBB 与 3D 的 Z 角容差，度，[0,180]。
    std::string sceneNormals = "mls";     ///< HALCON 场景法线方法，mls 或 fast。
    /** @brief 检查全部数值范围，失败时保留配置原值。
     * @throws std::invalid_argument 配置超出支持范围。 */
    void validate() const;
};
/** @brief 2D 原生推理算法，默认 OBB；非 OBB 不进行方向一致性判断。 */
enum class DetectorKind
{
    Obb,          ///< 旋转框检测，包含方向角。
    Segmentation, ///< 实例分割，包含全图掩膜。
    Detection     ///< 普通轴对齐检测框。
};
/** @brief 原生推理驱动，默认 ONNX；选择 OpenVINO 时需要对应 Vino DLL。 */
enum class DetectorDriver
{
    Onnx,    ///< ONNX Runtime 原生封装。
    OpenVino ///< 名称带 Vino 后缀的原生封装。
};
/** @brief 可选原生 DLL 配置，路径按 Windows 宽字符加载，模型路径遵循原 SDK UTF-8 约定。
 * @details 不持有句柄；由 NativeBackend 管理动态库。YOLO11 与 YOLO26 分开选择。 */
struct DetectorOptions
{
    std::string modelPath;                        ///< 非空模型文件路径，UTF-8。
    std::string runtimeDirectory;                 ///< 原 DLL 与依赖目录，UTF-8，必须存在。
    DetectorKind kind = DetectorKind::Obb;        ///< 算法任务。
    DetectorDriver driver = DetectorDriver::Onnx; ///< 推理驱动。
    double threshold = 0.5;                       ///< 输出置信度，[0,1]。
    bool useGpu = false;                          ///< 传给原生模型初始化函数的 GPU 选项。
    bool yolo26 = false;                          ///< 是否选择 26 后缀接口。
};
/** @brief 单个 2D 检测结果，框和掩膜均为原始图像坐标。
 * @details mask 通过 clone 独立持有；空掩膜表示没有分割数据，禁止并发修改同一 Mat。 */
struct Detection2D
{
    int classId = 0;             ///< 类别编号。
    float score = 0;             ///< 置信度。
    cv::RotatedRect box;         ///< 中心、尺寸为像素，显示角为度。
    float radians = 0;           ///< OBB 方向角，弧度。
    bool hasOrientation = false; ///< 仅 OBB 有可靠方向。
    cv::Mat mask;                ///< 可选的全图 CV_8UC1 掩膜，非零表示前景。
};
/** @brief 一次匹配的结果及其场景资源，HALCON 句柄通过 C++ 对象自动管理。
 * @details poses 每个候选占连续 7 项 HALCON pose，scores 与 radians2D 按候选对应。
 * 不创建 HALCON 匹配结果句柄以节省内存。scenes 保留实际参与计算的点云。
 * 对象可复制（句柄引用计数），只读共享；修改或释放期间禁止并发访问。 */
struct MatchResult
{
    HalconCpp::HTuple poses;      ///< 每个候选七元位姿，平移使用点云单位，角度由 HALCON pose 类型定义。
    HalconCpp::HTuple scores;     ///< 每个候选的分数。
    std::vector<float> radians2D; ///< 对应 2D 方向角，纯 3D 填零。
    std::vector<HalconCpp::HObjectModel3D> scenes; ///< 平滑后的场景，拥有引用计数句柄。
    std::vector<Detection2D> detections;           ///< 本次推理结果，含未通过 3D 检验的候选，供可视化。
    double elapsedMs = 0;                          ///< 整个 runModel 的耗时，毫秒。
};
/** @brief 显式模型上下文，替代原 C# SurfaceMatch 的隐式可变成员。
 * @details object 是模型点云，surface 是训练表面模型，二者应匹配；由 HALCON 引用计数释放。
 * 通过 shared_ptr<const ModelData> 供后台只读使用；不对共享句柄调用 Clear 系列算子。 */
struct ModelData
{
    HalconCpp::HObjectModel3D object; ///< 带法线的模型点云。
    HalconCpp::HSurfaceModel surface; ///< 可用于匹配的表面模型，默认未初始化。
};
} // 命名空间 sm
