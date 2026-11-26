#include "surface_match/CApi.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
/** @brief 检查 C 接口实际状态并在失败时复制错误文本，防止后续清理覆盖原生错误。
 * @param actual 刚完成的 C 调用返回值。
 * @param expected 本场景要求的 SmStatus。
 * @param message 中文场景说明，调用期间有效。
 * @throws std::runtime_error 返回状态不符合预期。 */
void expectStatus(int32_t actual, int32_t expected, const char *message)
{
    if (actual != expected)
        throw std::runtime_error(std::string(message) + "，状态=" + std::to_string(actual) + "，" +
                                 SmGetLastErrorMessage());
}

/** @brief 核对原生结果的数量和内容，Release 下也执行。
 * @param condition 本场景必须满足的条件。
 * @param message 中文失败说明，调用期间有效。
 * @throws std::runtime_error 条件不成立。 */
void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

/** @brief 从真实 DLL 验证模型卸载、缓存清空、重复调用和会话重用，不直接访问 C++ 私有状态。
 * @details 使用独立合成深度进行真实 HALCON 建模与匹配，卸载后通过 SmCopyMatches 直接确认缓存已空。
 * 会话由 unique_ptr 管理，异常时也调用 SmDestroy；不读写用户图像、模型或配置文件。
 * @throws std::runtime_error 原生状态、结果缓存或重建行为不符合预期。 */
void testModelLifecycle()
{
    expectStatus(SmDisposeModel(nullptr), SM_INVALID_ARGUMENT, "空会话不能用于卸载模型");
    SmContext *raw = nullptr;
    expectStatus(SmCreate(&raw), SM_OK, "创建会话");
    std::unique_ptr<SmContext, decltype(&SmDestroy)> context(raw, &SmDestroy);
    expectStatus(SmDisposeModel(context.get()), SM_OK, "未加载模型时卸载");
    expectStatus(SmDisposeModel(context.get()), SM_OK, "重复卸载空模型");
    SmConfig config{};
    expectStatus(SmGetDefaultConfig(&config, sizeof(config)), SM_OK, "读取默认参数");
    config.fx = config.fy = 220;
    config.cx = 80;
    config.cy = 60;
    config.numMatches = 1;
    expectStatus(SmSetConfig(context.get(), &config, sizeof(config)), SM_OK, "设置测试内参");
    std::vector<float> depth(160 * 120, 0);
    for (int y = 15; y < 105; ++y)
        for (int x = 20; x < 140; ++x)
            depth[y * 160 + x] =
                float(400 + 45 * std::exp(-((x - 65.0) * (x - 65) / 180 + (y - 45.0) * (y - 45) / 120)) +
                      28 * std::exp(-((x - 112.0) * (x - 112) / 80 + (y - 82.0) * (y - 82) / 200)) +
                      (x < 48 && y > 65 ? 32 : 0) + 0.15 * x + 0.04 * y);
    expectStatus(
        SmSetImage(context.get(), 1, depth.data(), depth.size() * sizeof(float), 160, 120, 160 * sizeof(float), 4),
        SM_OK, "复制深度图");
    expectStatus(SmCreateModelFromDepth(context.get()), SM_OK, "创建初始模型");
    expectStatus(SmRunModel(context.get(), nullptr, nullptr, 0, 0, 0.5, 0, 0, -1), SM_OK, "初始匹配");
    SmMatch match{};
    uint64_t count = 0;
    expectStatus(SmCopyMatches(context.get(), &match, 1, &count), SM_OK, "复制初始匹配");
    require(count == 1 && match.score > 0.6, "卸载前应有真实匹配结果");
    expectStatus(SmDisposeModel(context.get()), SM_OK, "卸载模型");
    count = 99;
    expectStatus(SmCopyMatches(context.get(), nullptr, 0, &count), SM_OK, "卸载后直接查询缓存");
    require(count == 0, "DisposeModel 应立即清空旧匹配缓存");
    require(match.score > 0.6, "调用方已复制的结果不应被卸载破坏");
    expectStatus(SmDisposeModel(context.get()), SM_OK, "重复卸载同一模型");
    expectStatus(SmRunModel(context.get(), nullptr, nullptr, 0, 0, 0.5, 0, 0, -1), SM_NOT_READY, "未重载模型不能运行");
    // 不重新输入深度或参数：重建成功说明卸载保留了匹配所需的会话状态。
    expectStatus(SmCreateModelFromDepth(context.get()), SM_OK, "使用保留的深度和参数重新建模");
    expectStatus(SmRunModel(context.get(), nullptr, nullptr, 0, 0, 0.5, 0, 0, -1), SM_OK, "重建后匹配");
    expectStatus(SmCopyMatches(context.get(), &match, 1, &count), SM_OK, "复制重建后的匹配");
    require(count == 1 && match.score > 0.6, "卸载后会话应能继续建模和匹配");
}
}

/** @brief 执行模型生命周期 C 接口验证并将异常转成测试失败退出码。
 * @return 全部通过为 0，失败为 1 并输出完整场景与原生错误；会话清理由测试函数的 RAII 完成。 */
int main()
{
    try
    {
        testModelLifecycle();
        std::cout << "PASS C ABI 模型卸载、缓存清空、空模型重复卸载与会话重用\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
