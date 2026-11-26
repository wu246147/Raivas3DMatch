#pragma once
#include "Types.h"

namespace sm
{
/** @brief PCA 拟合的空间直线，纯值类型，不依赖原始点集的生命周期。
 * @details direction 始终为单位向量，方向朝向首点一侧；point 为输入点的质心。 */
struct Line3D
{
    cv::Vec3d point{0, 0, 0};     ///< 直线上一点，长度单位与输入点一致。
    cv::Vec3d direction{1, 0, 0}; ///< 归一化方向，退化点集不生成该结果。
};
/** @brief 无状态静态工具集，迁移 Tool.cs 的图像、深度、位姿和点云算法。
 * @details 输入以 const 引用借用，返回值拥有自己的 OpenCV 数据或 HALCON 引用计数资源。
 * 除 DrawRotatedRect 外不修改输入；不同对象可并行处理，同一可变句柄需调用方同步。
 * HALCON/OpenCV 异常向上传递，不弹窗、不写隐式调试文件。 */
class Tool final
{
  public:
    /** @brief 通过文件字节解码图像，支持 Windows 中文路径，不损失深度位宽。
     * @param path UTF-8 图像路径，不能为空。
     * @param depth true 保留原位宽/通道；false 解码成 8 位 BGR。
     * @return 独立持有数据的非空图像。
     * @throws std::runtime_error 文件无法读取或图像解码失败。 */
    static cv::Mat readImage(const std::string &path, bool depth);
    /** @brief 按矩形保留原图对应像素，区域外置零；矩形会裁剪到图像边界。
     * @param image 非空输入图像，支持任意 OpenCV 像素类型。
     * @param roi 保留矩形，像素坐标；空矩形得到全零图。
     * @return 与输入大小/类型相同的独立图像。 */
    static cv::Mat LimitImage(const cv::Mat &image, const cv::Rect &roi);
    /** @brief 保留多个矩形的并集，空集合得到全零图，不改变坐标原点。
     * @param image 非空输入图像。
     * @param rois 任意数量像素矩形，越界部分自动裁剪。
     * @return 与输入大小/类型相同的独立图像。 */
    static cv::Mat LimitImage(const cv::Mat &image, const std::vector<cv::Rect> &rois);
    /** @brief 原位绘制旋转矩形、方向箭头和角度数值。
     * @param image 可写非空显示图，调用方持有数据并同步并发访问。
     * @param rect 待绘制矩形，像素坐标、显示角为度。
     * @param radians 箭头方向，弧度。
     * @param color OpenCV BGR 颜色。
     * @param thickness 正整数线宽，默认 2。 */
    static void DrawRotatedRect(cv::Mat &image, const cv::RotatedRect &rect, double radians, const cv::Scalar &color,
                                int thickness = 2);
    /** @brief 复制 OpenCV 图像到 HALCON，非连续 ROI 会先整理内存。
     * @param image 非空 8 位灰度/BGR/BGRA 或单通道 uint16/int16/float32/float64 图。
     * @return 自己持有数据的 HImage，不依赖输入 Mat 生命周期；float64 转 float32。
     * @throws std::invalid_argument 不支持的类型。 */
    static HalconCpp::HImage MatToHObject(const cv::Mat &image);
    /** @brief 将深度 Mat 复制为 HALCON 单通道图，拒绝多通道输入。
     * @param image 非空单通道深度图，类型同 MatToHObject。
     * @return 拥有数据的 HALCON 图像。 */
    static HalconCpp::HImage MatToHObjectDepth(const cv::Mat &image);
    /** @brief 按针孔模型反投影一个像素，拒绝越界、零深度和非有限值。
     * @param depth 单通道 uint16/int16/float32/float64 原始深度图。
     * @param pixel 原图像素坐标，x 是列，y 是行；必须在图内。
     * @param camera 有效内参和深度比例/偏移。
     * @return 相机坐标系 XYZ，长度单位由深度换算定义。
     * @throws std::invalid_argument 输入类型/深度无效；std::out_of_range 像素越界。 */
    static cv::Vec3d getPoint3DValue(const cv::Mat &depth, const cv::Point &pixel, const CameraIntrinsics &camera);
    /** @brief 批量反投影，保持输入顺序，任一无效点即报错，不生成虚构坐标。
     * @param depth 原始单通道深度图。
     * @param pixels 待采样像素列表，可为空。
     * @param camera 有效内参。
     * @return 与像素列表等长的 XYZ 列表。 */
    static std::vector<cv::Vec3d> get3DPoints(const cv::Mat &depth, const std::vector<cv::Point> &pixels,
                                              const CameraIntrinsics &camera);
    /** @brief 深度转有序 XYZ 图后交给 HALCON 生成点云，保留原图像素坐标约定。
     * @details 无效原始深度（<=0、NaN、Inf）先剔除，避免偏移将背景零值变成有效点。
     * @param depth 原始单通道 uint16/int16/float32/float64 深度图。
     * @param camera 相机内参与 z=raw*scale+offset。
     * @param bounds XYZ 闭区间；空点云是合法结果。
     * @param mask 可选 CV_8UC1 全图掩膜，与深度图同尺寸，非零有效。
     * @return 自动管理资源的 HALCON 点云；不修改任何输入。 */
    static HalconCpp::HObjectModel3D depthImgToObjectModel3D(const cv::Mat &depth, const CameraIntrinsics &camera,
                                                             const Bounds3D &bounds = {}, const cv::Mat &mask = {});
    /** @brief 迁移 S 相机直接深度转换，沿用原实现忽略低/高灰度映射的行为。
     * @param depth 已为物理深度的单通道图。
     * @param camera 只使用 fx/fy/cx/cy，换算强制为比例 1、偏移 0。
     * @param bounds 物理长度裁剪范围。
     * @return 自动管理的 HALCON 点云。 */
    static HalconCpp::HObjectModel3D depthImgToObjectModel3D_S(const cv::Mat &depth, CameraIntrinsics camera,
                                                               const Bounds3D &bounds = {});
    /** @brief 依次按 X/Y/Z 闭区间筛选点云，不改变原句柄。
     * @param object 有效 HALCON 点云。
     * @param bounds 各轴裁剪范围。
     * @return 新点云句柄，可能为空点云。 */
    static HalconCpp::HObjectModel3D filter(const HalconCpp::HObjectModel3D &object, const Bounds3D &bounds);
    /** @brief 只筛选 Z 闭区间。
     * @param object 有效点云。
     * @param minZ Z 下界，与点云同单位。
     * @param maxZ Z 上界，必须不小于 minZ。
     * @return 新点云句柄。 */
    static HalconCpp::HObjectModel3D filterZ(const HalconCpp::HObjectModel3D &object, double minZ, double maxZ);
    /** @brief 将法兰坐标系位姿左乘拍照时法兰到基座位姿。
     * @param pose 七元 HALCON 工件到法兰 pose。
     * @param robotPose 六元 x/y/z/rx/ry/rz，角度为度，Rp+T/abg/point。
     * @return 工件到基座七元 pose。 */
    static HalconCpp::HTuple gripper2base(const HalconCpp::HTuple &pose, const std::array<double, 6> &robotPose);
    /** @brief 将模型位姿转换到手眼坐标，再右乘可选抓取偏移。
     * @param pose 七元工件到相机 pose。
     * @param cameraToGripper 十二元 3x4 行主序手眼矩阵。
     * @param targetOffset 可选七元抓取点到工件 pose，空表示单位变换。
     * @return 抓取点到法兰（或固定相机标定坐标）pose。 */
    static HalconCpp::HTuple transformPose(const HalconCpp::HTuple &pose, const HalconCpp::HTuple &cameraToGripper,
                                           const HalconCpp::HTuple &targetOffset = {});
    /** @brief 求两次法兰位姿之间的相对变换，inv(base)*target。
     * @param target 七元目标法兰到基座 pose。
     * @param base 七元参考法兰到基座 pose。
     * @return 目标法兰到参考法兰 pose。 */
    static HalconCpp::HTuple getHTarget2Gripper(const HalconCpp::HTuple &target, const HalconCpp::HTuple &base);
    /** @brief 将抓取偏移右乘到当前位姿，统一输出为 abg。
     * @param pose 当前七元 HALCON pose。
     * @param localTransform 十二元局部变换矩阵。
     * @return 新七元 pose，不修改输入。 */
    static HalconCpp::HTuple transformPose2Target(const HalconCpp::HTuple &pose,
                                                  const HalconCpp::HTuple &localTransform);
    /** @brief 按原业务约定消除指定轴角度，用于旋转对称工件。
     * @param pose 七元 HALCON pose。
     * @param axis x/y/z；x 对应原 abg 最末角，z 对应 gba 最末角，y 使用局部 Y 旋转。
     * @return 新七元 abg pose。
     * @throws std::invalid_argument 轴名称无效。 */
    static HalconCpp::HTuple poseRotate(const HalconCpp::HTuple &pose, const std::string &axis);
    /** @brief 按原局部 Y 最优旋转规则消除 Y 方向分量，使用精确 pi/2。
     * @param pose 七元 HALCON pose。
     * @return 新七元 pose。 */
    static HalconCpp::HTuple poseRotateYToZero(const HalconCpp::HTuple &pose);
    /** @brief 沿位姿自身坐标轴平移，调用 HALCON local 平移而非全局相加。
     * @param pose 七元 HALCON pose。
     * @param translation 局部 XYZ 平移，与 pose 同单位。
     * @return 新七元 pose。 */
    static HalconCpp::HTuple pose_translate_local(const HalconCpp::HTuple &pose, const cv::Vec3d &translation);
    /** @brief 将点云转换到标定坐标，眼在手上时再转到机器人基座。
     * @param object 相机点云。
     * @param cameraToGripper 十二元手眼矩阵。
     * @param robotPose 拍照时法兰到基座六元位姿，度。
     * @param eyeInHand true 表示眼在手上。
     * @return 新坐标系点云。 */
    static HalconCpp::HObjectModel3D pointTransform(const HalconCpp::HObjectModel3D &object,
                                                    const HalconCpp::HTuple &cameraToGripper,
                                                    const std::array<double, 6> &robotPose, bool eyeInHand);
    /** @brief 逐帧变换并合并点云，不将空句柄混入 union。
     * @param objects 同一单位的相机点云列表，不可为空。
     * @param transforms 每帧对应十二元手眼矩阵，数量须一致。
     * @param robotPoses 每帧六元机器人位姿，数量须一致。
     * @param eyeInHand 是否眼在手上。
     * @return 融合点云，独立管理资源。 */
    static HalconCpp::HObjectModel3D pointCloudSplice(const std::vector<HalconCpp::HObjectModel3D> &objects,
                                                      const std::vector<HalconCpp::HTuple> &transforms,
                                                      const std::vector<std::array<double, 6>> &robotPoses,
                                                      bool eyeInHand);
    /** @brief 按映射抓取点在标定坐标系的 Z 从大到小返回候选索引。
     * @param poses 连续七元 HALCON pose，允许空。
     * @param catchPoint 模板抓取点 XYZ，与模型同单位。
     * @param cameraToGripper 十二元手眼矩阵。
     * @return 排序后的候选索引，等高保持输入顺序。 */
    static std::vector<int> getHighestModel(const HalconCpp::HTuple &poses, const cv::Vec3d &catchPoint,
                                            const HalconCpp::HTuple &cameraToGripper);
    /** @brief 按模型与场景最近距离统计接触点，返回通过接触数门限的候选。
     * @param object 模板碰撞模型，调用中仅变换其副本。
     * @param scene 场景点云；先按原逻辑过滤第十邻居距离 [0,1]。
     * @param poses 连续七元候选 pose。
     * @param distanceLimit 接触距离，正数，单位与点云一致。
     * @param touchLimit 接触点数量上限，正整数，严格小于才接受。
     * @param rotationalSymmetry 是否先按 Z 旋转对称处理。
     * @return 通过筛选的候选索引，可能为空。 */
    static std::vector<int> getNoTouchModel(const HalconCpp::HObjectModel3D &object,
                                            const HalconCpp::HObjectModel3D &scene, const HalconCpp::HTuple &poses,
                                            double distanceLimit, int touchLimit, bool rotationalSymmetry);
    /** @brief 对点集进行 PCA 主轴拟合，防止重合点或非有限值造成无意义结果。
     * @param points 至少两个有限 XYZ 点。
     * @return 质心和单位主方向，方向与原实现保持首点一侧约定。
     * @throws std::invalid_argument 点集不足或完全重合。 */
    static Line3D FitLine(const std::vector<cv::Vec3d> &points);
    /** @brief 计算各点到空间直线的垂直距离。
     * @param points 有限 XYZ 列表，可为空。
     * @param line 有限且方向非零的直线，内部归一化方向。
     * @return 与输入等长的距离列表。 */
    static std::vector<double> GetDistancesToLine(const std::vector<cv::Vec3d> &points, const Line3D &line);
    /** @brief 将旋转矩阵转换为单位四元数，使用 Rodrigues 保持大角度稳定性。
     * @param rotation 有效的 3x3 正交旋转矩阵。
     * @return 按 x/y/z/w 排列的四元数。 */
    static cv::Vec4d RotationMatrixToQuaternion(const cv::Matx33d &rotation);
    /** @brief 将 HALCON 位姿旋转分量转换为四元数。
     * @param pose 七元 HALCON pose。
     * @return x/y/z/w 单位四元数。 */
    static cv::Vec4d TransformPoseToQuaternions(const HalconCpp::HTuple &pose);
    /** @brief 归一化四元数后构造矩阵，再由 HALCON 提取 pose，避免欧拉角约定混淆。
     * @param translation 有限 XYZ 平移。
     * @param quaternion 有限、非零的 x/y/z/w 四元数。
     * @return abg 七元 pose。
     * @throws std::invalid_argument 四元数为零或非有限。 */
    static HalconCpp::HTuple TransformQuaternionsToPose(const cv::Vec3d &translation, const cv::Vec4d &quaternion);
    /** @brief 按最大权重四元数进行同半球符号对齐，再加权归一化。
     * @param quaternions 非空、有限、非零的 x/y/z/w 四元数集合。
     * @param weights 非负有限权重，数量一致且总和大于零；空表示等权。
     * @return 单位平均四元数。
     * @throws std::invalid_argument 参数无效或加权结果退化。 */
    static cv::Vec4d WeightedAverageQuaternions(const std::vector<cv::Vec4d> &quaternions,
                                                const std::vector<double> &weights = {});
    /** @brief 分别平均平移和同半球四元数，避免欧拉角跨越 360 度的错误。
     * @param poses 非空七元 pose 集合，必须描述同一工件和坐标系。
     * @param weights 有限非负权重；空表示等权。
     * @return 新七元 abg pose。 */
    static HalconCpp::HTuple WeightedAveragePoses(const std::vector<HalconCpp::HTuple> &poses,
                                                  const std::vector<double> &weights = {});
    /** @brief 在同心圆环内阈值化、开运算并选最大连通域后计算深度均值。
     * @param depth 非空单通道图，统计原始像素值，不应用深度换算。
     * @param center 圆心列/行，像素，有限。
     * @param low 原始灰度下界，有限。
     * @param high 原始灰度上界，必须不小于 low。
     * @param outerDiameter 外圆直径，像素，正数。
     * @param innerDiameter 内圆直径，像素，[0,outerDiameter)。
     * @return 最大区域的原始深度均值。
     * @throws std::runtime_error 无有效连通区域。 */
    static double get_image_circle_z(const cv::Mat &depth, cv::Point2d center, double low, double high,
                                     double outerDiameter, double innerDiameter);
    /** @brief 查找低灰度圆形区域的外接矩形，迁移原阈值 0..10、圆度 0.5..1、面积 100..99999。
     * @param image 非空单通道 HALCON 图像。
     * @return 各区域的像素外接矩形，可能为空。 */
    static std::vector<cv::Rect> get_region_center(const HalconCpp::HImage &image);
    /** @brief 由三点构造右手平面坐标系，X 沿第一条边、Z 沿叉乘法线。
     * @param points 恰好三个有限、不共线 XYZ 点。
     * @param position 输出三点质心。
     * @param eulerDegrees 输出与 HALCON abg 一致的欧拉角，单位度。
     * @throws std::invalid_argument 点数或几何结构不合法。 */
    static void CalculatePlanePoseWithEuler(const std::vector<cv::Vec3d> &points, cv::Vec3d &position,
                                            cv::Vec3d &eulerDegrees);
    /** @brief 求向量相对 XY 平面的有符号仰角。
     * @param vector 有限非零三维向量。
     * @return [-90,90] 度。 */
    static double GetAngleWithXoYPlaneSigned(const cv::Vec3d &vector);
    /** @brief 求向量在 XY 平面相对 X 的方位角。
     * @param vector 有限非零三维向量。
     * @return [-180,180] 度。 */
    static double GetAngleXWithY(const cv::Vec3d &vector);
    /** @brief 按 X 排序等分左右两列，再各按 Y 排序，修复原实现分列不均丢点问题。
     * @param points 非空有限点集，数量必须为 totalSize。
     * @param totalSize 偶数且至少为 4。
     * @return 左右两列，每列独立持有数据。 */
    static std::array<std::vector<cv::Vec3d>, 2> SeparatePoints(std::vector<cv::Vec3d> points, int totalSize);
    /** @brief 合并原 1/2/4/8/10/12 点精定位和 3 点粗定位算法。
     * @details 1 点仅平移；3 点建平面；2/4 点拟合单线；8/10/12 点两列拟合平均。
     * @param points 点数须为 1/2/3/4/8/10/12；点必须有限且几何结构非退化。
     * @return abg 七元 pose；原未实现的两点粗定位由两点精定位实现覆盖。
     * @throws std::invalid_argument 点数或几何结构无效。 */
    static HalconCpp::HTuple mulPointToPose(const std::vector<cv::Vec3d> &points);
    /** @brief 原两点粗定位入口，补全原 C# 未实现的计算，采用两点精定位规则。
     * @param points 恰好两个不重合的有限 XYZ 点。
     * @return abg 七元 pose；无效点数或退化输入抛 std::invalid_argument。 */
    static HalconCpp::HTuple mulPointToPose2Coarse(const std::vector<cv::Vec3d> &points);
    /** @brief 原三点粗定位入口，构造右手平面坐标系。
     * @param points 恰好三个不共线的有限 XYZ 点。
     * @return abg 七元 pose；点数或几何结构无效时抛 std::invalid_argument。 */
    static HalconCpp::HTuple mulPointToPose3Coarse(const std::vector<cv::Vec3d> &points);
    /** @brief 原单点精定位入口，仅返回该点位置、方向为零。
     * @param points 恰好一个有限 XYZ 点。
     * @return abg 七元 pose；无效输入抛 std::invalid_argument。 */
    static HalconCpp::HTuple mulPointToPose1(const std::vector<cv::Vec3d> &points);
    /** @brief 原两点精定位入口，按 Y 排序后拟合主方向。
     * @param points 恰好两个不重合的有限 XYZ 点。
     * @return abg 七元 pose；无效输入抛 std::invalid_argument。 */
    static HalconCpp::HTuple mulPointToPose2(const std::vector<cv::Vec3d> &points);
    /** @brief 原四点精定位入口，按单列 PCA 计算位姿。
     * @param points 恰好四个非完全重合的有限 XYZ 点。
     * @return abg 七元 pose；无效输入抛 std::invalid_argument。 */
    static HalconCpp::HTuple mulPointToPose4(const std::vector<cv::Vec3d> &points);
    /** @brief 原八点精定位入口，分成两列拟合后平均方向。
     * @param points 恰好八个有限 XYZ 点，两列均不得完全重合。
     * @return abg 七元 pose；无效输入抛 std::invalid_argument。 */
    static HalconCpp::HTuple mulPointToPose8(const std::vector<cv::Vec3d> &points);
    /** @brief 原十点精定位入口，分成两列拟合后平均方向。
     * @param points 恰好十个有限 XYZ 点，两列均不得完全重合。
     * @return abg 七元 pose；无效输入抛 std::invalid_argument。 */
    static HalconCpp::HTuple mulPointToPose10(const std::vector<cv::Vec3d> &points);
    /** @brief 原十二点精定位入口，分成两列拟合后平均方向。
     * @param points 恰好十二个有限 XYZ 点，两列均不得完全重合。
     * @return abg 七元 pose；无效输入抛 std::invalid_argument。 */
    static HalconCpp::HTuple mulPointToPose12(const std::vector<cv::Vec3d> &points);
};
} // 命名空间 sm
