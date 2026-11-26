#pragma once
#include "Types.h"
namespace sm
{
/** @brief 可选原生 DLL 接入层，直接包含用户指定 SDK 的原始头文件推导函数类型。
 * @details 使用原头文件的 decltype 保留 __stdcall、cv::Mat& 等 ABI，不从 C# P/Invoke 猜测签名。
 * DLL 按绝对路径按需加载，缺失时明确报错，不影响纯 HALCON 匹配。RaiVas 使用全局模型，
 * 本层在同一互斥锁中完成初始化/推理/读取结果，防止多上下文互相覆盖。
 * 不应由本层以外代码并发调用同组 DLL 的全局接口；DLL 在进程退出时由系统释放。 */
class NativeBackend final
{
  public:
    /** @brief 加载原生推理 DLL、初始化所选模型并获取一批检测结果。
     * @param image 非空 8 位灰度/BGR/BGRA 图；推理使用副本，输入不被修改。
     * @param options 模型、驱动、任务、阈值和 DLL 路径。
     * @return 独立持有掩膜的检测列表，未检测到目标时为空。
     * @throws std::runtime_error 未编译后端、DLL/符号缺失或原 SDK 返回非零错误。
     * @note 模型配置相同时复用已加载模型；全局状态操作串行。 */
    static std::vector<Detection2D> run2DModel(const cv::Mat &image, const DetectorOptions &options);
    /** @brief 调用原 DepthRadiusFilter.h 声明的深度滤波函数，保持图像尺寸和坐标映射。
     * @param depth 非空单通道 uint16/int16/float32/float64 图，转换为 float32 后调用。
     * @param camera 相机内参；不应用比例偏移，滤波门限按原始深度单位解释。
     * @param runtimeDirectory DLL 与其依赖目录，UTF-8。
     * @param windowSize 正奇数窗口，默认 5 像素。
     * @param minNeighbors 最少邻居数，[0,windowSize*windowSize-1]。
     * @param maxDepthDifference 正数最大原始深度差。
     * @param threads 非负线程数，0 保留 SDK 默认。
     * @return 独立 float32 深度图，禁止 SDK 删除无效点以免破坏二维映射。
     * @throws std::runtime_error 后端关闭或原 SDK 调用失败。 */
    static cv::Mat DepthRadiusFilter(const cv::Mat &depth, const CameraIntrinsics &camera,
                                     const std::string &runtimeDirectory, int windowSize = 5, int minNeighbors = 8,
                                     float maxDepthDifference = 0.1f, int threads = 0);
};
} // 命名空间 sm
