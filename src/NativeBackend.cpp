#include "surface_match/NativeBackend.h"
#include <cmath>
#include <filesystem>
#include <map>
#include <mutex>
#include <stdexcept>
#ifdef _WIN32
#include <Windows.h>
#endif
#ifdef SM_ENABLE_RAIVAS
// 原始封装头文件传递包含 ONNX Runtime；本层只推导 ABI，不初始化该运行时。
#define ORT_API_MANUAL_INIT
#include <raivasalgobbdet.h>
#include <raivasalgseg.h>
#include <raivasalgdet.h>
#include <raivasalgobbdetvino.h>
#include <raivasalgsegvino.h>
#include <raivasalgdetvino.h>
#endif
#ifdef SM_ENABLE_DEPTH_FILTER
#include <DepthRadiusFilter.h>
#endif

namespace sm
{
namespace
{
std::mutex nativeMutex; ///< 保护 SDK 全局模型、结果缓存及动态库列表。
#if defined(SM_ENABLE_RAIVAS) || defined(SM_ENABLE_DEPTH_FILTER)
/** @brief 加载并缓存指定目录的 DLL，同时允许其本地依赖解析。
 * @param directory 有效 UTF-8 SDK 运行目录。
 * @param name DLL 文件名。
 * @return 进程生命周期内有效的模块句柄，不交给调用方释放。
 * @throws std::runtime_error DLL 或依赖缺失；调用前须持有 nativeMutex。 */
HMODULE module(const std::string &directory, const std::string &name)
{
    static std::map<std::wstring, HMODULE> modules;
    auto dir = std::filesystem::absolute(std::filesystem::u8path(directory));
    auto path = dir / std::filesystem::u8path(name);
    auto key = path.wstring();
    auto found = modules.find(key);
    if (found != modules.end())
        return found->second;
    // 目录 cookie 保持到进程结束，原 SDK 延迟加载 ONNX/OpenVINO 插件时仍能找到依赖。
    if (!AddDllDirectory(dir.c_str()))
        throw std::runtime_error("无法添加 SDK 依赖目录，Windows 错误：" + std::to_string(GetLastError()));
    auto handle = LoadLibraryExW(path.c_str(), nullptr,
                                 LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS |
                                     LOAD_LIBRARY_SEARCH_USER_DIRS);
    if (!handle)
        throw std::runtime_error("无法加载 " + path.u8string() + "，Windows 错误：" + std::to_string(GetLastError()));
    modules.emplace(key, handle);
    return handle;
}
/** @brief 按原始头文件推导出的函数指针类型查找导出入口。
 * @tparam Function 原生头文件声明对应的 decltype(&函数)。
 * @param handle 已加载模块句柄。
 * @param name 实际导出名称。
 * @return 保留调用约定与参数类型的函数指针。
 * @throws std::runtime_error 入口不存在。 */
template <class Function> Function entry(HMODULE handle, const std::string &name)
{
    auto address = GetProcAddress(handle, name.c_str());
    if (!address)
        throw std::runtime_error("原生 DLL 缺少导出：" + name);
    return reinterpret_cast<Function>(address);
}
/** @brief 将原 SDK 错误转换为携带操作名称的异常。
 * @param code 原返回值，0 为成功。
 * @param operation 用于错误提示的接口名称。
 * @throws std::runtime_error code 不为零。 */
void checkNative(int code, const std::string &operation)
{
    if (code)
        throw std::runtime_error(operation + " 失败，原 SDK 错误码：" + std::to_string(code));
}
#endif
} // 匿名命名空间
std::vector<Detection2D> NativeBackend::run2DModel(const cv::Mat &image, const DetectorOptions &options)
{
#ifdef SM_ENABLE_RAIVAS
    if (image.empty() || image.depth() != CV_8U ||
        (image.channels() != 1 && image.channels() != 3 && image.channels() != 4))
        throw std::invalid_argument("2D 推理需要 8 位灰度/BGR/BGRA 图");
    if (options.modelPath.empty() || options.runtimeDirectory.empty() || !std::isfinite(options.threshold) ||
        options.threshold < 0 || options.threshold > 1)
        throw std::invalid_argument("推理路径或置信度无效");
    std::lock_guard<std::mutex> lock(nativeMutex);
    std::string task = options.kind == DetectorKind::Obb            ? "OBB"
                       : options.kind == DetectorKind::Segmentation ? "Seg"
                                                                    : "Det";
    std::string dllTask = task == "OBB" ? "OBBDet" : task;
    std::string suffix = options.driver == DetectorDriver::OpenVino ? "Vino" : "", version = options.yolo26 ? "26" : "";
    auto handle = module(options.runtimeDirectory, "RaiVasAlg" + dllTask + suffix + ".dll");
    std::string initName = "init" + (task == "OBB" ? std::string("OBBModel") : task + "Model") + version + suffix;
    std::string setName = "set" + task + "Para" + version + suffix, runName = "run" + task + version + suffix,
                getName = "get" + task + "Result" + version + suffix;
    static std::map<HMODULE, std::string> activeModels;
    // YOLO11/26 有独立状态；缓存键保留版本和 GPU，切换配置后必须重新初始化。
    auto key = version + "|" + options.modelPath + "|" + (options.useGpu ? "gpu" : "cpu");
    try
    {
        if (activeModels[handle] != key)
        {
            activeModels.erase(handle);
            checkNative(entry<decltype(&::initOBBModel)>(handle, initName)(options.modelPath.c_str(), options.useGpu),
                        initName);
            activeModels[handle] = key;
        }
        checkNative(entry<decltype(&::setOBBPara)>(handle, setName)(options.threshold), setName);
        cv::Mat input = image.clone();
        int count = 0;
        checkNative(entry<decltype(&::runOBB)>(handle, runName)(input, count), runName);
        if (count < 0 || count > 100000)
            throw std::runtime_error("原 SDK 返回了无效检测数量");
        std::vector<Detection2D> out;
        out.reserve(count);
        for (int i = 0; i < count; ++i)
        {
            Detection2D d;
            float x = 0, y = 0, w = 0, h = 0, a = 0;
            if (options.kind == DetectorKind::Obb)
            {
                checkNative(entry<decltype(&::getOBBResult)>(handle, getName)(i, &d.classId, &d.score, &x, &y, &w, &h,
                                                                              &a, &d.radians),
                            getName);
                d.hasOrientation = true;
            }
            else if (options.kind == DetectorKind::Segmentation)
            {
                cv::Mat mask;
                checkNative(
                    entry<decltype(&::getSegResult)>(handle, getName)(i, &d.classId, &d.score, &x, &y, &w, &h, mask),
                    getName);
                d.mask = mask.clone();
            }
            else
                checkNative(entry<decltype(&::getDetResult)>(handle, getName)(i, &d.classId, &d.score, &x, &y, &w, &h),
                            getName);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) || !std::isfinite(h) || !std::isfinite(a) ||
                !std::isfinite(d.score) || !std::isfinite(d.radians) || w <= 0 || h <= 0)
                throw std::runtime_error("原 SDK 返回了无效检测框");
            d.box = cv::RotatedRect({x, y}, {w, h}, a);
            out.push_back(std::move(d));
        }
        return out;
    }
    catch (int code)
    {
        activeModels.erase(handle);
        throw std::runtime_error("原 SDK 抛出错误码：" + std::to_string(code));
    }
#else
    (void)image;
    (void)options;
    throw std::runtime_error("当前构建未启用 SM_ENABLE_RAIVAS");
#endif
}
cv::Mat NativeBackend::DepthRadiusFilter(const cv::Mat &depth, const CameraIntrinsics &camera,
                                         const std::string &runtimeDirectory, int windowSize, int minNeighbors,
                                         float maxDepthDifference, int threads)
{
#ifdef SM_ENABLE_DEPTH_FILTER
    camera.validate();
    if (depth.empty() || depth.channels() != 1 ||
        (depth.depth() != CV_16U && depth.depth() != CV_16S && depth.depth() != CV_32F && depth.depth() != CV_64F) ||
        windowSize <= 0 || windowSize > 255 || windowSize % 2 == 0 || minNeighbors < 0 ||
        minNeighbors >= windowSize * windowSize || !std::isfinite(maxDepthDifference) || maxDepthDifference <= 0 ||
        threads < 0)
        throw std::invalid_argument("深度滤波参数无效");
    cv::Mat input, output;
    depth.convertTo(input, CV_32F);
    std::lock_guard<std::mutex> lock(nativeMutex);
    auto handle = module(runtimeDirectory, "RaivasAlgDepthFilter.dll");
    try
    {
        checkNative(entry<decltype(&::DepthRadiusFilter)>(handle, "DepthRadiusFilter")(
                        input, output, camera.fx, camera.fy, camera.cx, camera.cy, windowSize, minNeighbors,
                        maxDepthDifference, threads, false),
                    "DepthRadiusFilter");
    }
    catch (int code)
    {
        throw std::runtime_error("深度滤波 SDK 抛出错误码：" + std::to_string(code));
    }
    if (output.size() != depth.size() || output.type() != CV_32FC1)
        throw std::runtime_error("深度滤波 SDK 改变了图像尺寸或类型");
    return output.clone();
#else
    (void)depth;
    (void)camera;
    (void)runtimeDirectory;
    (void)windowSize;
    (void)minNeighbors;
    (void)maxDepthDifference;
    (void)threads;
    throw std::runtime_error("当前构建未启用 SM_ENABLE_DEPTH_FILTER");
#endif
}
} // 命名空间 sm
