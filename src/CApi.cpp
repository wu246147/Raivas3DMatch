#include "surface_match/CApi.h"
#include "surface_match/SurfaceMatch.h"
#include <mutex>
#include <string>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <type_traits>

/** @brief DLL 私有会话，实现 C 句柄的资源所有权与串行访问。
 * @details mutex 保护相机、范围、模型、图像与结果；model 使用 HALCON 引用计数，图像为独立 Mat。
 * 生命周期由 SmCreate/SmDestroy 控制，销毁期间调用方必须保证没有活动调用。 */
struct SmContext
{
    std::mutex mutex;                           ///< 同一会话操作的互斥锁。
    sm::CameraIntrinsics camera;                ///< 当前相机配置。
    sm::Bounds3D bounds;                        ///< 当前相机系空间范围。
    sm::MatchParameters parameters;             ///< 当前匹配配置。
    std::shared_ptr<const sm::ModelData> model; ///< 当前只读模型，可为空。
    cv::Mat image, depth;                       ///< DLL 拥有的图像数据。
    sm::MatchResult result;                     ///< 最近一次成功运行结果，失败时清空。
};
static_assert(sizeof(SmPose) == 56 && sizeof(SmMatch) == 72 && sizeof(SmGraspConfig) == 256 &&
                  sizeof(SmGraspResult) == 224,
              "C ABI 布局不符");
namespace
{
thread_local std::string lastError; ///< 仅本线程有效的 UTF-8 最后错误。
/** @brief 保护 C ABI 边界，转换所有异常并记录实际错误。
 * @tparam Operation 无参数、返回 int32_t 状态的可调用类型。
 * @param operation 待执行同步操作，不保存其引用。
 * @return 原状态或转换后的错误状态，异常不跨 DLL 边界。 */
template <class Operation> int32_t guarded(Operation &&operation) noexcept
{
    try
    {
        lastError.clear();
        return operation();
    }
    catch (const HalconCpp::HException &e)
    {
        lastError =
            "HALCON " + std::to_string(e.ErrorCode()) + " / " + e.ProcName().Text() + ": " + e.ErrorMessage().Text();
        return SM_ERROR;
    }
    catch (const std::invalid_argument &e)
    {
        lastError = e.what();
        return SM_INVALID_ARGUMENT;
    }
    catch (const std::logic_error &e)
    {
        lastError = e.what();
        return SM_NOT_READY;
    }
    catch (const std::exception &e)
    {
        lastError = e.what();
        return SM_ERROR;
    }
    catch (...)
    {
        lastError = "未识别的原生异常，C ABI 已阻止其跨越 DLL 边界";
        return SM_ERROR;
    }
}
/** @brief 验证必要指针或配置条件。
 * @param condition 须满足的条件。
 * @param message 失败原因，调用期间有效。
 * @throws std::invalid_argument 条件不满足。 */
void validate(bool condition, const char *message)
{
    if (!condition)
        throw std::invalid_argument(message);
}
/** @brief 将 C 位姿转换成 HALCON abg，拒绝非有限值与非零 pose 类型。
 * @param p 调用方提供的七元值。
 * @return 独立七元 HALCON pose。 */
HalconCpp::HTuple poseIn(const SmPose &p)
{
    validate(p.type == 0, "C ABI 只接受 Type=0 的 abg 位姿");
    const double v[] = {p.x, p.y, p.z, p.rx, p.ry, p.rz};
    for (auto x : v)
        validate(std::isfinite(x), "位姿必须为有限值");
    HalconCpp::HTuple out;
    HalconCpp::CreatePose(p.x, p.y, p.z, p.rx, p.ry, p.rz, "Rp+T", "abg", "point", &out);
    return out;
}
/** @brief 将 HALCON 位姿统一到 C ABI 的 abg 布局。
 * @param pose 七元 HALCON pose。
 * @return 无资源所有权的七元值。 */
SmPose poseOut(const HalconCpp::HTuple &pose)
{
    HalconCpp::HTuple p;
    HalconCpp::ConvertPoseType(pose, "Rp+T", "abg", "point", &p);
    return {p[0].D(), p[1].D(), p[2].D(), p[3].D(), p[4].D(), p[5].D(), 0};
}
/** @brief 验证深度 Mat 的通道和位宽，供文件和内存入口共用。
 * @param depth 待验证 Mat，不修改。
 * @throws std::invalid_argument 图像为空或不是支持的深度格式。 */
void validDepth(const cv::Mat &depth)
{
    validate(
        !depth.empty() && depth.channels() == 1 &&
            (depth.depth() == CV_16U || depth.depth() == CV_16S || depth.depth() == CV_32F || depth.depth() == CV_64F),
        "深度须为单通道 16 位整数或 32/64 位浮点图");
}
}
uint32_t SM_CALL SmGetAbiVersion(void)
{
    return 1;
}
const char *SM_CALL SmGetLastErrorMessage(void)
{
    return lastError.c_str();
}
int32_t SM_CALL SmGetDefaultConfig(SmConfig *config, uint32_t configSize)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(config && configSize == sizeof(SmConfig), "SmConfig 指针或尺寸无效");
            sm::CameraIntrinsics c;
            sm::Bounds3D b;
            sm::MatchParameters p;
            *config = {c.fx,
                       c.fy,
                       c.cx,
                       c.cy,
                       c.depthScale,
                       c.depthOffset,
                       b.min[0],
                       b.max[0],
                       b.min[1],
                       b.max[1],
                       b.min[2],
                       b.max[2],
                       p.minScore,
                       p.maxOverlapDistance,
                       p.modelSampling,
                       p.sceneSampling,
                       p.keyPointFraction,
                       p.downsampleDistance,
                       p.angleTolerance,
                       p.numMatches,
                       p.modelSmoothK,
                       p.sceneSmoothK,
                       p.roiExpansion};
            return SM_OK;
        });
}
int32_t SM_CALL SmCreate(SmContext **context)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context, "会话输出地址为空");
            *context = nullptr;
            *context = new SmContext;
            return SM_OK;
        });
}
int32_t SM_CALL SmDestroy(SmContext *context)
{
    return guarded(
        [&]() -> int32_t
        {
            delete context;
            return SM_OK;
        });
}
int32_t SM_CALL SmSetConfig(SmContext *context, const SmConfig *config, uint32_t configSize)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context && config && configSize == sizeof(SmConfig), "会话、配置或配置尺寸无效");
            sm::CameraIntrinsics c{config->fx, config->fy,         config->cx,
                                   config->cy, config->depthScale, config->depthOffset};
            sm::Bounds3D b;
            b.min = {config->minX, config->minY, config->minZ};
            b.max = {config->maxX, config->maxY, config->maxZ};
            sm::MatchParameters p;
            p.minScore = config->minScore;
            p.maxOverlapDistance = config->maxOverlapDistance;
            p.modelSampling = config->modelSampling;
            p.sceneSampling = config->sceneSampling;
            p.keyPointFraction = config->keyPointFraction;
            p.downsampleDistance = config->downsampleDistance;
            p.angleTolerance = config->angleTolerance;
            p.numMatches = config->numMatches;
            p.modelSmoothK = config->modelSmoothK;
            p.sceneSmoothK = config->sceneSmoothK;
            p.roiExpansion = config->roiExpansion;
            c.validate();
            b.validate();
            p.validate();
            std::lock_guard<std::mutex> lock(context->mutex);
            context->camera = c;
            context->bounds = b;
            context->parameters = p;
            context->result = {};
            return SM_OK;
        });
}
int32_t SM_CALL SmLoadImages(SmContext *context, const char *imagePath, const char *depthPath)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context && depthPath && *depthPath, "会话或深度路径为空");
            auto depth = sm::Tool::readImage(depthPath, true);
            validDepth(depth);
            cv::Mat image;
            if (imagePath && *imagePath)
                image = sm::Tool::readImage(imagePath, false);
            std::lock_guard<std::mutex> lock(context->mutex);
            context->depth = std::move(depth);
            context->image = std::move(image);
            context->result = {};
            return SM_OK;
        });
}
int32_t SM_CALL SmSetImage(SmContext *context, int32_t imageKind, const void *pixels, uint64_t bufferBytes,
                           int32_t width, int32_t height, uint64_t stride, int32_t pixelFormat)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context && pixels && width > 0 && height > 0 && (imageKind == 0 || imageKind == 1),
                     "图像指针、尺寸或种类无效");
            int type = -1;
            uint64_t pixelBytes = 0;
            switch (pixelFormat)
            {
            case 1:
                type = CV_8UC1;
                pixelBytes = 1;
                break;
            case 2:
                type = CV_8UC3;
                pixelBytes = 3;
                break;
            case 3:
                type = CV_16UC1;
                pixelBytes = 2;
                break;
            case 4:
                type = CV_32FC1;
                pixelBytes = 4;
                break;
            case 5:
                type = CV_64FC1;
                pixelBytes = 8;
                break;
            default:
                throw std::invalid_argument("像素格式无效");
            }
            validate((imageKind == 0 && pixelFormat <= 2) || (imageKind == 1 && pixelFormat >= 3),
                     "像素格式与图像种类不一致");
            uint64_t rowBytes = uint64_t(width) * pixelBytes;
            validate(stride >= rowBytes && stride <= std::numeric_limits<std::size_t>::max(), "图像行步长无效");
            validate(uint64_t(height - 1) <= (std::numeric_limits<uint64_t>::max() - rowBytes) / stride,
                     "图像大小溢出");
            validate(bufferBytes >= uint64_t(height - 1) * stride + rowBytes, "输入缓冲区不足以覆盖全部图像行");
            auto copy = cv::Mat(height, width, type, const_cast<void *>(pixels), std::size_t(stride)).clone();
            std::lock_guard<std::mutex> lock(context->mutex);
            if (imageKind == 0)
                context->image = std::move(copy);
            else
                context->depth = std::move(copy);
            context->result = {};
            return SM_OK;
        });
}
int32_t SM_CALL SmLoadModel(SmContext *context, const char *path, const char *fileScale)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context && path && *path, "会话或模型路径为空");
            std::lock_guard<std::mutex> lock(context->mutex);
            auto model =
                sm::SurfaceMatch::initModel(path, context->parameters, fileScale && *fileScale ? fileScale : "m");
            context->model = std::move(model);
            context->result = {};
            return SM_OK;
        });
}
int32_t SM_CALL SmDisposeModel(SmContext *context)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context, "会话为空");
            std::lock_guard<std::mutex> lock(context->mutex);
            // 先清除旧结果持有的场景点云，再释放模型所有权；相机配置和输入图像继续供后续加载使用。
            context->result = {};
            context->model.reset();
            return SM_OK;
        });
}
int32_t SM_CALL SmCreateModelFromDepth(SmContext *context)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context, "会话为空");
            std::lock_guard<std::mutex> lock(context->mutex);
            if (context->depth.empty())
                throw std::logic_error("请先设置深度图");
            auto scene = sm::Tool::depthImgToObjectModel3D(context->depth, context->camera, context->bounds);
            auto model = sm::SurfaceMatch::initModel3D(scene, context->parameters);
            context->model = std::move(model);
            context->result = {};
            return SM_OK;
        });
}
int32_t SM_CALL SmRunModel(SmContext *context, const char *detectorPath, const char *runtimeDirectory, int32_t kind,
                           int32_t driver, double threshold, int32_t useGpu, int32_t yolo26, int32_t attitudeId)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context, "会话为空");
            std::lock_guard<std::mutex> lock(context->mutex);
            context->result = {};
            if (!context->model || context->depth.empty())
                throw std::logic_error("运行前必须先设置深度并加载模型");
            sm::DetectorOptions detector;
            bool use2D = detectorPath && *detectorPath;
            if (use2D)
            {
                validate(runtimeDirectory && *runtimeDirectory && kind >= 0 && kind <= 2 &&
                             (driver == 0 || driver == 1) && (useGpu == 0 || useGpu == 1) &&
                             (yolo26 == 0 || yolo26 == 1) && attitudeId >= -1,
                         "2D 后端配置无效");
                detector.modelPath = detectorPath;
                detector.runtimeDirectory = runtimeDirectory;
                detector.kind = sm::DetectorKind(kind);
                detector.driver = sm::DetectorDriver(driver);
                detector.threshold = threshold;
                detector.useGpu = useGpu != 0;
                detector.yolo26 = yolo26 != 0;
            }
            context->result = sm::SurfaceMatch::runModel(*context->model, context->image, context->depth,
                                                         context->camera, context->bounds, context->parameters,
                                                         use2D ? &detector : nullptr, attitudeId);
            return SM_OK;
        });
}
int32_t SM_CALL SmCopyMatches(SmContext *context, SmMatch *matches, uint64_t capacity, uint64_t *written)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(context && written, "会话或数量输出地址为空");
            std::lock_guard<std::mutex> lock(context->mutex);
            uint64_t count = uint64_t(context->result.scores.Length());
            *written = count;
            if (capacity < count)
            {
                lastError = "输出数组容量不足";
                return SM_BUFFER_TOO_SMALL;
            }
            validate(matches || count == 0, "候选输出地址为空");
            for (uint64_t i = 0; i < count; ++i)
            {
                auto offset = Hlong(i) * 7;
                matches[i] = {poseOut(context->result.poses.TupleSelectRange(offset, offset + 6)),
                              context->result.scores[Hlong(i)].D(), context->result.radians2D.at(std::size_t(i))};
            }
            return SM_OK;
        });
}
int32_t SM_CALL SmTransformPose(const SmPose *matchPose, const SmGraspConfig *config, SmGraspResult *result)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(matchPose && config && result && config->mode >= 0 && config->mode <= 2 &&
                         (config->rotationalSymmetry == 0 || config->rotationalSymmetry == 1),
                     "抓手配置或指针无效");
            sm::GraspTransform c;
            c.cameraInBase = poseIn(config->cameraInBase);
            c.taughtModelInBase = poseIn(config->taughtModelInBase);
            c.taughtTcpInBase = poseIn(config->taughtTcpInBase);
            if (config->mode)
                c.offsetReferenceInBase = poseIn(config->offsetReferenceInBase);
            c.mode = sm::OffsetMode(config->mode);
            c.rotationalSymmetry = config->rotationalSymmetry != 0;
            c.modelCenter = {config->centerX, config->centerY, config->centerZ};
            auto out = sm::SurfaceMatch::transformPose(poseIn(*matchPose), c);
            *result = {poseOut(out.modelInBase), poseOut(out.motionInBase), poseOut(out.tcpInBase),
                       poseOut(out.outputPose)};
            return SM_OK;
        });
}
int32_t SM_CALL SmTransformRelativePose(const SmPose *matchPose, int32_t eyeInHand, const SmPose *handEyePose,
                                        const SmPose *robotPose, SmPose *relativePose)
{
    return guarded(
        [&]() -> int32_t
        {
            validate(matchPose && handEyePose && relativePose && (eyeInHand == 0 || eyeInHand == 1),
                     "相对抓手变换的指针或眼在手上标志无效");
            HalconCpp::HTuple robot;
            if (eyeInHand)
            {
                validate(robotPose != nullptr, "眼在手上必须提供拍照时的机器人位姿");
                robot = poseIn(*robotPose);
            }
            auto out = sm::SurfaceMatch::transformPose(poseIn(*matchPose), eyeInHand != 0, poseIn(*handEyePose),
                                                       eyeInHand ? &robot : nullptr);
            *relativePose = poseOut(out);
            return SM_OK;
        });
}
