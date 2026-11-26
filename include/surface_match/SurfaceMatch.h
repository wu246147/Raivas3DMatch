#pragma once
#include "Tool.h"
#include "NativeBackend.h"

namespace sm
{
/** @brief 机器人偏移输出规则，对应 MainWindow.GetCatchSendPose。
 * @details 无控制器通信副作用；只计算 HALCON abg 位姿，不转换机器人专有角度格式。 */
enum class OffsetMode
{
    Absolute, ///< 直接输出基座系绝对 TCP。
    Fanuc,    ///< 完整矩阵偏移：最终 TCP × 偏移基准的逆。
    Nachi     ///< 旋转保留矩阵偏移，XYZ 改为同一基座坐标中的位置差。
};
/** @brief 将 runModel 的模型运动恢复为抓手目标所需的显式标定数据。
 * @details 所有 pose 均为七元 HALCON 位姿，统一长度单位，角度由第七项约定。
 * 对应 MainWindow 的 GetCurrentCameraInBase / RestoreCatchPose / GetCatchSendPose，
 * 无文件隐式读取和全局状态，调用期间只能只读共享。 */
struct GraspTransform
{
    HalconCpp::HTuple cameraInBase;          ///< 当前拍照相机到基座 pose；眼在手上时由拍照法兰 pose × 手眼得到。
    HalconCpp::HTuple taughtModelInBase;     ///< 示教时模板到基座 pose；固定相机原始相机系模板可取建模时 cameraInBase。
    HalconCpp::HTuple taughtTcpInBase;       ///< 初始示教抓手 TCP 到基座 pose，不是模板局部坐标。
    HalconCpp::HTuple offsetReferenceInBase; ///< 控制器偏移基准 pose；Absolute 模式可为空。
    OffsetMode mode = OffsetMode::Absolute;  ///< 输出规则。
    bool rotationalSymmetry = false;         ///< 是否在参考模型系去除局部 Z 对称旋转。
    cv::Vec3d modelCenter{0, 0, 0};          ///< 模板坐标下 XYZ 中心；对称补偿时保持该点匹配后的位置不变。
};
/** @brief 一次抓手转换的各阶段结果，均为独立七元 HALCON abg pose。
 * @details 供界面核对模型运动、绝对 TCP 和发送偏移，不负责 IK/碰撞检查及机器人通信。 */
struct GraspResult
{
    HalconCpp::HTuple modelInBase;  ///< 当前模板在基座中的位姿，含可选对称补偿。
    HalconCpp::HTuple motionInBase; ///< 当前模板 × 示教模板逆，基座系左乘运动。
    HalconCpp::HTuple tcpInBase;    ///< motionInBase × 示教 TCP，绝对抓手目标。
    HalconCpp::HTuple outputPose;   ///< 根据 OffsetMode 得到的最终抓手位姿或偏移。
};
/** @brief 通过二维中心与深度统计得到的定位结果，不包含三维表面验证。
 * @details 每个中心对应一个七元平移 pose 和置信度，combinedPose 由 coarse/fine 计算。
 * 自有值数据，可移动；共享后不得并发修改 HALCON 元组与 OpenCV 掩膜。 */
struct LocationResult
{
    HalconCpp::HTuple poses;             ///< 中心的相机系七元位姿列表，方向为零。
    HalconCpp::HTuple scores;            ///< 对应置信度。
    HalconCpp::HTuple combinedPose;      ///< 组合后的最终位姿；未组合时为空。
    std::vector<Detection2D> detections; ///< 通过类别和空间范围的检测目标。
};
/** @brief 无实例状态的 HALCON/OpenCV 表面匹配服务，替代 SurfaceMatch.cs。
 * @details 模型、参数、相机和可选 2D 配置显式传入，支持后台任务只读共享模型。
 * HALCON 使用 C++ 引用计数管理资源，不手工 Clear 共享句柄；异常由调用方记录或展示。
 * 函数不操作 UI、不发送机器人消息、不隐式写入模型缓存。 */
class SurfaceMatch final
{
  public:
    /** @brief 设置与原 C# 相同的精度档位，保留其他配置。
     * @param parameters 待修改参数，仅更新模型/场景采样及关键点比例。
     * @param accuracy Low/Normal/High/VeryHigh。
     * @throws std::invalid_argument 枚举无效或最终配置不合法。 */
    static void setPara(MatchParameters &parameters, MatchingAccuracy accuracy);
    /** @brief 从点云计算法线并训练表面模型，不写隐式 .sfm 缓存。
     * @param object 有效非空模板点云，单位须与运行场景一致。
     * @param parameters 训练采样和 MLS 配置。
     * @return 只读共享模型上下文，最后一个引用销毁时释放资源。 */
    static std::shared_ptr<const ModelData> initModel3D(const HalconCpp::HObjectModel3D &object,
                                                        const MatchParameters &parameters);
    /** @brief 读取 .sfm，或读取点云/CAD 并训练，避免同名缓存掩盖参数修改。
     * @param path UTF-8 .sfm/.om3/.ply/.obj/.stl/.dxf 等 HALCON 支持的文件路径。
     * @param parameters 非 .sfm 时用于训练，.sfm 直接使用其原训练参数。
     * @param fileScale HALCON 输入文件单位，默认 m，支持 mm/cm/m 等合法值。
     * @return 模型上下文，.sfm 从模型参数恢复模型点云。 */
    static std::shared_ptr<const ModelData> initModel(const std::string &path, const MatchParameters &parameters,
                                                      const std::string &fileScale = "m");
    /** @brief 显式保存表面模型到磁盘，覆盖目标由调用方确认。
     * @param model 有效模型上下文。
     * @param path 输出 .sfm 文件 UTF-8 路径。
     * @note 有文件写入副作用；失败抛 HALCON 异常。 */
    static void saveModel(const ModelData &model, const std::string &path);
    /** @brief 场景可选降采样/平滑后执行 FindSurfaceModel，空场景返回空结果。
     * @param model 已初始化表面模型，只读借用。
     * @param scene 非空句柄，点数可以为零。
     * @param parameters 查找与预处理配置。
     * @return pose/分数和平滑场景，不持有额外匹配调试句柄。 */
    static MatchResult run3DModel(const ModelData &model, const HalconCpp::HObjectModel3D &scene,
                                  const MatchParameters &parameters);
    /** @brief 完整运行深度转点云、裁剪和表面匹配，可先用原生 2D 框限制搜索。
     * @details 保持原类别过滤、扩张旋转框、每框单目标匹配与 OBB 方向差门限；Seg/Det 不强加角度门限。
     * @param model 模板上下文。
     * @param image 已与深度配准的 8 位 2D 图，纯 3D 模式可为空。
     * @param depth 原始单通道 uint16/int16/float32/float64 深度图。
     * @param camera 内参与深度换算。
     * @param bounds 相机系 XYZ 裁剪范围。
     * @param parameters 匹配参数。
     * @param detector 可选原生 2D 配置，nullptr 表示纯 3D；调用期间有效、不转移所有权。
     * @param attitudeId 类别筛选，-1 接受全部，其他值只接受该类。
     * @return 结果及耗时；无目标为正常空结果，失败抛异常。 */
    static MatchResult runModel(const ModelData &model, const cv::Mat &image, const cv::Mat &depth,
                                const CameraIntrinsics &camera, const Bounds3D &bounds,
                                const MatchParameters &parameters, const DetectorOptions *detector = nullptr,
                                int attitudeId = -1);
    /** @brief 将相机系匹配偏移换算为机器人基座系的抓手相对位移，不计算绝对抓取点。
     * @details 先求相机到基座 C，再计算 D_base=C*D_camera*inverse(C)，完整保留旋转的杠杆臂效应。
     * 眼在手外 C=handEyePose；眼在手上 C=robotPose*handEyePose。
     * @param matchPose 单个七元 HALCON 目标偏移 D_camera；模板须保留参考相机坐标，
     * 若模板已移到局部原点或参考相机已改变，须先用真实参考将匹配绝对位姿换成相机系偏移。
     * @param eyeInHand true 表示眼在手上，false 表示固定相机。
     * @param handEyePose 七元手眼标定位姿；眼在手上为相机到机器人末端，眼在手外为相机到基座。
     * @param robotPose 拍照时标定所对应的末端到基座七元位姿，仅 eyeInHand=true 时必填；
     * 固定相机允许 nullptr 并忽略此参数。指针只在调用期间借用，不保存、不释放。
     * @return 独立七元 Rp+T/abg/point 相对 pose，角度为度，平移与输入同单位；
     * 作用方式为 T_new_tcp=D_base*T_reference_tcp，XYZ 不是机器人当前位置的分量差，欧拉角也不可直接相加。
     * @throws std::invalid_argument 必填位姿不是七项、含非有限值或眼在手上未提供机器人位姿。
     * @note 所有平移单位须一致；不修改输入、不读写文件、不操作机器人，可并发调用但输入不可并发修改。
     * HALCON 的位姿类型/数值错误继续抛出 HException，由调用方处理。 */
    static HalconCpp::HTuple transformPose(const HalconCpp::HTuple &matchPose, bool eyeInHand,
                                           const HalconCpp::HTuple &handEyePose,
                                           const HalconCpp::HTuple *robotPose = nullptr);
    /** @brief 兼容旧版示教抓取链路，返回绝对 TCP 及按控制器规则生成的偏移。
     * @details B_M=C_B*M_C；D=B_M*inv(B_M_teach)；TCP=D*TCP_teach。
     * 对称补偿在示教模板系执行并保持中心位置；发送偏移按 Fanuc/Nachi/Absolute 分支处理。
     * @param matchPose runModel 中单个七元模型到当前相机 pose，不允许传入整批列表。
     * @param calibration 显式相机/模板/抓手示教与偏移参考，单位须统一。
     * @return 基座模型运动、绝对 TCP 和最终发送偏移，不操作机器人。
     * @throws std::invalid_argument 必要 pose 缺失、非有限或模式无效。 */
    static GraspResult transformPose(const HalconCpp::HTuple &matchPose, const GraspTransform &calibration);
    /** @brief 读取原 HALCON 形状模型，自动管理句柄。
     * @param path UTF-8 .shm/.hmd 路径。
     * @return 形状模型；不读取原未参与计算的 .txt 附属数据。 */
    static HalconCpp::HShapeModel initShapeModel(const std::string &path);
    /** @brief 兼容原 initModel2D 命名，读取同一个 HALCON 形状模型。
     * @param path 完整 UTF-8 模型文件路径。
     * @return 自动管理的形状模型。 */
    static HalconCpp::HShapeModel initModel2D(const std::string &path);
    /** @brief 在灰度图上按原缩放/旋转范围搜索形状模型。
     * @param model 已初始化形状模型。
     * @param image 非空 8 位灰度/BGR/BGRA 图，内部转灰度并均值滤波。
     * @param threshold 最低匹配分数，[0,1]。
     * @param numMatches 最大结果数，正整数。
     * @return 检测列表，box 中心为匹配中心，方向保留弧度。 */
    static std::vector<Detection2D> findShapeModel(const HalconCpp::HShapeModel &model, const cv::Mat &image,
                                                   double threshold = 0.5, int numMatches = 1);
    /** @brief 对已有 2D 检测中心进行圆环深度统计并反投影，供粗/精定位共用。
     * @param depth 原始深度图。
     * @param detections 原图坐标检测结果。
     * @param camera 相机配置。
     * @param bounds XYZ 闭区间。
     * @param outerDiameter 外圆直径，像素。
     * @param innerDiameter 内圆直径，像素。
     * @param grayLow 原始灰度下限。
     * @param grayHigh 原始灰度上限。
     * @param attitudeId -1 全部类别，否则只接受指定类别。
     * @return 中心平移 pose 与分数；无区域时抛异常以暴露数据问题。 */
    static LocationResult locationImage(const cv::Mat &depth, const std::vector<Detection2D> &detections,
                                        const CameraIntrinsics &camera, const Bounds3D &bounds, double outerDiameter,
                                        double innerDiameter, double grayLow, double grayHigh, int attitudeId = -1);
    /** @brief 将三幅图的中心定位组合成机器人基座系平面位姿。
     * @param parts 恰好三份定位结果，每份必须仅有一个中心 pose。
     * @param cameraInBase 每幅图拍照时相机到基座 pose，数量恰好为三。
     * @return 合并的原始中心数据和基座平面 combinedPose。 */
    static LocationResult coarseLocation(const std::vector<LocationResult> &parts,
                                         const std::vector<HalconCpp::HTuple> &cameraInBase);
    /** @brief 根据已定位中心集合执行原点数对应的精定位算法。
     * @param parts 定位结果，中心数量须为 1/2/4/8/10/12，按原图坐标排序规则组合。
     * @return 保留中心数据，并填入相机坐标系 combinedPose。 */
    static LocationResult fineLocation(const LocationResult &parts);
    /** @brief 多帧点云变换到统一坐标后合并匹配，或逐帧单目标匹配并平均姿态。
     * @param model 同一模板上下文，不支持混用不同物体。
     * @param scenes 非空相机点云列表。
     * @param transforms 每帧对应相机到统一坐标的十二元矩阵。
     * @param parameters 匹配参数，逐帧模式强制每帧一个目标以避免跨物体平均。
     * @param mergeFirst true 合并后匹配；false 每帧匹配后等权平均，任一帧无匹配则返回空。
     * @return 统一坐标系结果。 */
    static MatchResult runMulObjModel(const ModelData &model, const std::vector<HalconCpp::HObjectModel3D> &scenes,
                                      const std::vector<HalconCpp::HTuple> &transforms,
                                      const MatchParameters &parameters, bool mergeFirst);
    /** @brief 按原 SurfaceMatch.getHighestModel 规则选择相机系最小 Z 的模型。
     * @param object 模板点云，须非空。
     * @param poses 连续七元候选 pose。
     * @return 候选索引，空列表返回 -1；该规则与机器人基座 Z 排序不同。 */
    static int getHighestModel(const HalconCpp::HObjectModel3D &object, const HalconCpp::HTuple &poses);
};
} // 命名空间 sm
