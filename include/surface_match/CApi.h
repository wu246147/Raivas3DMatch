#pragma once
#include <stdint.h>
#ifdef _WIN32
#ifdef SM_C_API_EXPORTS
#define SM_API __declspec(dllexport)
#else
#define SM_API __declspec(dllimport)
#endif
#define SM_CALL __cdecl
#else
#define SM_API
#define SM_CALL
#endif
#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 原生会话的不透明句柄，隔离 C++/HALCON/OpenCV 类型。
 * @details 由 SmCreate 分配、SmDestroy 唯一释放，调用方不得解引用、复制所有权或使用已释放句柄。
 * 同一会话的操作内部串行，销毁不得与调用并发；跨会话可并发，原生 RaiVas 后端仍按原全局锁串行。 */
    typedef struct SmContext SmContext;
    /** @brief C ABI 返回状态，固定 int32_t，不使用 C++ 异常跨越 DLL 边界。 */
    typedef enum SmStatus
    {
        SM_OK = 0,               ///< 成功，也包含没有匹配目标的正常运行。
        SM_INVALID_ARGUMENT = 1, ///< 参数、尺寸或配置无效。
        SM_ERROR = 2,            ///< HALCON、OpenCV、文件或原生后端错误。
        SM_NOT_READY = 3,        ///< 缺少模型或深度等前置状态。
        SM_BUFFER_TOO_SMALL = 4  ///< 输出容量不足，written 返回需要的元素数量。
    } SmStatus;
    /** @brief 可直接封送的七元 HALCON abg 位姿，按八字节对齐。
 * @details 平移长度单位须与点云一致，Rx/Ry/Rz 为度；Type 应为 0（Rp+T/abg/point）。
 * 该值结构不持有资源，可由 C# 按 Sequential/Pack=8 传入和接收。 */
    typedef struct SmPose
    {
        double x;    ///< X 平移。
        double y;    ///< Y 平移。
        double z;    ///< Z 平移。
        double rx;   ///< X 旋转，度。
        double ry;   ///< Y 旋转，度。
        double rz;   ///< Z 旋转，度。
        double type; ///< HALCON pose 类型，本 ABI 统一为 0。
    } SmPose;
    /** @brief C ABI 相机、范围和匹配配置，固定大小值结构，不包含指针。
 * @details 通过 SmGetDefaultConfig 初始化，再修改需要的字段。XYZ 范围为闭区间，
 * 默认允许无穷 X/Y；训练采样/平滑改变后须重新加载或创建模型。 */
    typedef struct SmConfig
    {
        double fx;                 ///< X 焦距，像素，有限正数。
        double fy;                 ///< Y 焦距，像素，有限正数。
        double cx;                 ///< 主点列，像素，有限值。
        double cy;                 ///< 主点行，像素，有限值。
        double depthScale;         ///< 深度比例，有限正数。
        double depthOffset;        ///< 深度偏移，有限值。
        double minX;               ///< 相机系 X 下界。
        double maxX;               ///< 相机系 X 上界。
        double minY;               ///< 相机系 Y 下界。
        double maxY;               ///< 相机系 Y 上界。
        double minZ;               ///< 相机系 Z 下界。
        double maxZ;               ///< 相机系 Z 上界。
        double minScore;           ///< 匹配阈值，[0,1]。
        double maxOverlapDistance; ///< 重叠相对距离，非负。
        double modelSampling;      ///< 模型相对采样，(0,1]。
        double sceneSampling;      ///< 场景相对采样，(0,1]。
        double keyPointFraction;   ///< 关键点比例，(0,1]。
        double downsampleDistance; ///< 降采样绝对间距，0 关闭。
        double angleTolerance;     ///< OBB 角度容差，度，[0,180]。
        int32_t numMatches;        ///< 最大匹配数量，正整数。
        int32_t modelSmoothK;      ///< 模型 MLS 邻居数，0 或至少 3。
        int32_t sceneSmoothK;      ///< 场景 MLS 邻居数，0 或至少 3。
        int32_t roiExpansion;      ///< 框扩张像素，非负整数。
    } SmConfig;
    /** @brief 单个候选的可封送结果，候选索引与 runModel 输出顺序一致。 */
    typedef struct SmMatch
    {
        SmPose pose;      ///< 模型到当前相机的七元 abg pose。
        double score;     ///< 匹配分数。
        double radians2D; ///< 2D 方向弧度，纯 3D 模式为零。
    } SmMatch;
    /** @brief 抓手转换配置，与 SurfaceMatch::transformPose 的坐标约定一致。
 * @details 全部位姿使用统一长度单位和 abg 度；眼在手上时调用方先组合拍照法兰与手眼位姿。 */
    typedef struct SmGraspConfig
    {
        SmPose cameraInBase;          ///< 当前相机到基座。
        SmPose taughtModelInBase;     ///< 示教模板到基座。
        SmPose taughtTcpInBase;       ///< 示教抓手 TCP 到基座。
        SmPose offsetReferenceInBase; ///< 控制器偏移基准；绝对模式忽略。
        double centerX;               ///< 模板中心 X，仅对称补偿时使用。
        double centerY;               ///< 模板中心 Y，仅对称补偿时使用。
        double centerZ;               ///< 模板中心 Z，仅对称补偿时使用。
        int32_t mode;                 ///< 0 绝对 TCP，1 Fanuc 矩阵偏移，2 Nachi 位置差。
        int32_t rotationalSymmetry;   ///< 0 保留旋转，1 去除局部 Z 对称角并补偿中心。
    } SmGraspConfig;
    /** @brief 抓手变换结果，无资源所有权，所有 pose 均为 abg。 */
    typedef struct SmGraspResult
    {
        SmPose modelInBase;  ///< 当前模板到基座。
        SmPose motionInBase; ///< 基座系模板运动。
        SmPose tcpInBase;    ///< 绝对抓手 TCP。
        SmPose outputPose;   ///< 最终发送位姿或偏移。
    } SmGraspResult;

    /** @brief 查询此 DLL 的 C ABI 版本，不分配资源。
 * @return 当前为 1；封送方应拒绝不支持的版本。 */
    SM_API uint32_t SM_CALL SmGetAbiVersion(void);
    /** @brief 获取本线程最后错误的 UTF-8 文本，不清除错误状态。
 * @return 借用的零结尾文本指针，成功时为空串；仅有效到同线程下一次 C API 调用，调用方不得释放。 */
    SM_API const char *SM_CALL SmGetLastErrorMessage(void);
    /** @brief 初始化默认配置并验证调用方结构尺寸。
 * @param config 非空输出结构，调用方分配，不转移所有权。
 * @param configSize 调用方 sizeof(SmConfig)，必须与 ABI 1 一致。
 * @return SmStatus；失败原因由 SmGetLastErrorMessage 获取。 */
    SM_API int32_t SM_CALL SmGetDefaultConfig(SmConfig *config, uint32_t configSize);
    /** @brief 创建独立算法会话，失败时输出置空。
 * @param context 非空句柄输出地址；成功后调用方必须恰好调用一次 SmDestroy。
 * @return SmStatus。 */
    SM_API int32_t SM_CALL SmCreate(SmContext **context);
    /** @brief 释放会话中的模型、图像和缓存；nullptr 是无操作。
 * @param context 仍有效的唯一会话句柄，不得与其他调用并发，不得重复释放。
 * @return SmStatus；本函数不将 C++ 异常传播给调用方。 */
    SM_API int32_t SM_CALL SmDestroy(SmContext *context);
    /** @brief 校验并替换会话配置，成功后清除旧匹配，保留图像和已训练模型。
 * @param context 有效会话句柄。
 * @param config 调用期间有效的只读配置结构。
 * @param configSize sizeof(SmConfig)，用于拒绝错误版本或封送布局。
 * @return SmStatus；模型训练参数改变后调用方须重新建模。 */
    SM_API int32_t SM_CALL SmSetConfig(SmContext *context, const SmConfig *config, uint32_t configSize);
    /** @brief 读取配准图像文件，深度保留原位宽；成功后清除旧匹配。
 * @param context 有效会话句柄。
 * @param imagePath 可选 UTF-8 2D 路径，nullptr 或空串表示纯 3D 输入。
 * @param depthPath 非空 UTF-8 深度文件路径。
 * @return SmStatus；失败保留原图像。 */
    SM_API int32_t SM_CALL SmLoadImages(SmContext *context, const char *imagePath, const char *depthPath);
    /** @brief 从原始像素缓冲区复制一张图像，不保留调用方内存引用。
 * @param context 有效会话句柄。
 * @param imageKind 0 表示 2D 图，1 表示深度图。
 * @param pixels 调用期间有效的输入地址，非空；返回后可释放或解除固定。
 * @param bufferBytes 输入缓冲区可访问的总字节数，必须包含全部行。
 * @param width 像素宽度，正整数。
 * @param height 像素高度，正整数。
 * @param stride 每行字节数，正数，至少为 width 乘单像素大小，允许尾部填充。
 * @param pixelFormat 1=Gray8，2=BGR8，3=DepthU16，4=DepthF32，5=DepthF64；须与 imageKind 一致。
 * @return SmStatus；图像被复制到 DLL 管理的 Mat，成功后清除旧匹配。 */
    SM_API int32_t SM_CALL SmSetImage(SmContext *context, int32_t imageKind, const void *pixels, uint64_t bufferBytes,
                                      int32_t width, int32_t height, uint64_t stride, int32_t pixelFormat);
    /** @brief 从文件读取并训练模板，.sfm 直接加载；成功后替换旧模型并清除旧匹配。
 * @param context 有效会话句柄。
 * @param path 非空 UTF-8 模型文件路径。
 * @param fileScale HALCON 文件单位，如 m/mm，nullptr 默认 m；模型须与场景最终单位一致。
 * @return SmStatus；失败保留旧模型。 */
    SM_API int32_t SM_CALL SmLoadModel(SmContext *context, const char *path, const char *fileScale);
    /** @brief 卸载会话的三维模型并清空匹配缓存，保留图像、相机和匹配配置。
 * @details 在会话锁内清空结果，再释放模型引用，由 C++/HALCON 生命周期管理释放模型点云和表面模型；
 * 不对共享句柄手工调用 Clear。尚未加载模型或重复卸载均成功，不销毁会话或删除磁盘模型文件。
 * @param context 非空、尚未销毁的会话句柄，仅在调用期间借用，不转移所有权。
 * @return SM_OK 卸载成功；空句柄返回 SM_INVALID_ARGUMENT，其他失败由错误状态和本线程最后错误说明。
 * @note 与同一会话的模型加载、匹配、结果读取串行，可能等待正在执行的操作；不得与 SmDestroy 并发。
 * 卸载后 SmRunModel 返回 SM_NOT_READY，须重新加载或创建模型；调用方此前复制的候选数据不受影响。 */
    SM_API int32_t SM_CALL SmDisposeModel(SmContext *context);
    /** @brief 以当前深度、内参和范围生成模板并训练，用于快速验证流程。
 * @param context 已设置有效深度和配置的会话。
 * @return SmStatus；成功替换模型并清除旧匹配。 */
    SM_API int32_t SM_CALL SmCreateModelFromDepth(SmContext *context);
    /** @brief 同步执行 runModel 并缓存本次结果，失败时清除旧结果，避免误用上一次候选。
 * @param context 已具备模型和深度的会话；耗时调用建议由 C# 后台任务执行。
 * @param detectorPath 可选 UTF-8 2D 模型路径，nullptr/空串表示纯 3D。
 * @param runtimeDirectory 2D DLL 目录，启用 2D 时非空，否则忽略。
 * @param kind 2D 任务，0=OBB、1=Seg、2=Det。
 * @param driver 0=ONNX、1=OpenVINO。
 * @param threshold 2D 置信度，[0,1]，纯 3D 忽略。
 * @param useGpu 0 CPU，1 GPU，按原 SDK 支持情况执行。
 * @param yolo26 0 YOLO11，1 YOLO26。
 * @param attitudeId -1 接受全部类别，非负值按类别筛选。
 * @return SmStatus；没有候选为 SM_OK，调用 SmCopyMatches 查询数量。 */
    SM_API int32_t SM_CALL SmRunModel(SmContext *context, const char *detectorPath, const char *runtimeDirectory,
                                      int32_t kind, int32_t driver, double threshold, int32_t useGpu, int32_t yolo26,
                                      int32_t attitudeId);
    /** @brief 将缓存候选复制到调用方数组，容量不足时不写入任何候选。
 * @param context 有效会话；同一业务流程的 run/copy 若由多个线程使用，调用方须整体同步。
 * @param matches 输出数组，capacity=0 时允许 nullptr 进行数量查询。
 * @param capacity 数组的 SmMatch 元素容量，不是字节数。
 * @param written 非空输出地址，接收需要的元素数量；成功时等于实际写入数。
 * @return SM_OK 或 SM_BUFFER_TOO_SMALL；其他状态参见最后错误文本。 */
    SM_API int32_t SM_CALL SmCopyMatches(SmContext *context, SmMatch *matches, uint64_t capacity, uint64_t *written);
    /** @brief 将单个 runModel 模型位姿转换为最终抓手位姿/偏移，不依赖会话。
 * @param matchPose 非空单个模型到相机的 abg pose。
 * @param config 非空且有效的示教与标定配置。
 * @param result 非空输出结构，由调用方分配。
 * @return SmStatus，失败时不修改 result；不发送机器人命令。 */
    SM_API int32_t SM_CALL SmTransformPose(const SmPose *matchPose, const SmGraspConfig *config, SmGraspResult *result);

    /** @brief 将相机系目标偏移换算为基座系抓手相对位移，无需创建会话。
 * @details 相机到基座 C 在眼在手上时为 robotPose*handEyePose，否则为 handEyePose；
 * 结果 D=C*matchPose*inverse(C)，使用 T_new_tcp=D*T_reference_tcp，不是 XYZ/欧拉角分量加法。
 * @param matchPose 非空单个相机系目标偏移；原始 runModel 结果须使用参考相机坐标模板。
 * @param eyeInHand 只能为 0（眼在手外）或 1（眼在手上）。
 * @param handEyePose 非空手眼位姿，0 模式为相机到基座，1 模式为相机到标定末端。
 * @param robotPose 1 模式必须非空，为拍照时的标定末端到基座；0 模式允许 nullptr，传入时也忽略。
 * @param relativePose 非空输出地址，调用方分配；全部输入先读取，成功后才写出，允许与输入地址重合。
 * @return SM_OK 成功；无效指针、枚举、位姿返回 SM_INVALID_ARGUMENT，HALCON 失败返回 SM_ERROR；
 * 失败不修改输出，详情从本线程 SmGetLastErrorMessage 获取。
 * @note 所有 SmPose 使用 Type=0、abg 度和统一长度单位；借用输入仅在调用期间有效，不保留指针。
 * 不修改会话、不操作机器人，可并发调用，调用方须保证输入和输出地址不存在并发读写。 */
    SM_API int32_t SM_CALL SmTransformRelativePose(const SmPose *matchPose, int32_t eyeInHand,
                                                   const SmPose *handEyePose, const SmPose *robotPose,
                                                   SmPose *relativePose);

#ifdef __cplusplus
}
#endif
