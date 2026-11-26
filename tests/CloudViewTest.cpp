#include "CloudView.h"
#include <QApplication>
#include <QImage>
#include <QMouseEvent>
#include <QWheelEvent>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
/** @brief 在 Release 构建中也执行显示和交互断言。
 * @param condition 预期成立的条件。
 * @param message 失败时的中文说明。
 * @throws std::runtime_error 条件为 false。 */
void check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
/** @brief 比较两个九元相机状态的差异，避免仅依据截图主观判断鼠标是否生效。
 * @param a 第一份位置、焦点和上方向。
 * @param b 第二份位置、焦点和上方向。
 * @return 差异的绝对值之和。 */
double cameraDifference(const std::array<double, 9> &a, const std::array<double, 9> &b)
{
    double d = 0;
    for (std::size_t i = 0; i < a.size(); ++i)
        d += std::abs(a[i] - b[i]);
    return d;
}
/** @brief 通过 Qt 事件通道模拟鼠标拖动，验证真实控件到 VTK 的事件转发。
 * @param view 已初始化的点云控件，不转移所有权。
 * @param button 左键旋转或中键平移。
 * @param from 拖动起点，控件逻辑像素。
 * @param to 拖动终点，控件逻辑像素。 */
void drag(CloudView &view, Qt::MouseButton button, const QPointF &from, const QPointF &to)
{
    QMouseEvent press(QEvent::MouseButtonPress, from, button, button, Qt::NoModifier);
    QApplication::sendEvent(&view, &press);
    QMouseEvent move(QEvent::MouseMove, to, Qt::NoButton, button, Qt::NoModifier);
    QApplication::sendEvent(&view, &move);
    QMouseEvent release(QEvent::MouseButtonRelease, to, button, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(&view, &release);
    QApplication::processEvents();
}
/** @brief 检查 HALCON 到显示快照的数量上限和候选矩阵，随后验证原生 OpenGL 和交互。
 * @details 使用隐藏的 Windows 原生窗口，仍创建真实 OpenGL 上下文；不依赖 offscreen 插件假装渲染。 */
void runTest()
{
    HalconCpp::HTuple x, y, z, pose;
    for (int i = 0; i < 100; ++i)
    {
        x.Append(double(i));
        y.Append(0.0);
        z.Append(1.0);
    }
    HalconCpp::HObjectModel3D object(x, y, z);
    HalconCpp::CreatePose(12, 3, 4, 0, 0, 0, "Rp+T", "abg", "point", &pose);
    auto prepared = CloudView::prepareData(object, &object, pose, 7, 11);
    check(prepared->scenePoints.size() <= 7 && prepared->modelPoints.size() <= 11, "显示抽样须遵守数量上限");
    check(prepared->modelPoses.size() == 1 && prepared->modelPoses[0][3] == 12, "候选矩阵须保留实际平移");
    auto data = std::make_shared<CloudSceneData>();
    for (int row = 0; row < 45; ++row)
        for (int col = 0; col < 65; ++col)
        {
            float px = (col - 32) * 0.05f, py = (row - 22) * 0.05f,
                  pz = 0.2f * std::sin(col * 0.15f) + 0.1f * std::cos(row * 0.2f);
            data->scenePoints.emplace_back(px, py, pz);
            if (col > 18 && col < 46 && row > 10 && row < 35)
                data->modelPoints.emplace_back(px, py, pz);
        }
    std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    data->modelPoses.push_back(identity);
    CloudView view;
    view.resize(850, 600);
    view.setAttribute(Qt::WA_DontShowOnScreen);
    view.show();
    view.setData(data);
    view.setSelectedCandidate(0);
    QApplication::processEvents();
    check(view.saveScreenshot("cloud_initial.png"), "原生 OpenGL 初始截图失败");
    QImage initial("cloud_initial.png");
    int colored = 0;
    for (int yy = 0; yy < initial.height(); ++yy)
        for (int xx = 0; xx < initial.width(); ++xx)
        {
            auto c = initial.pixelColor(xx, yy);
            if (c.red() > 180 && c.green() > 80 && c.green() < 220 && c.blue() < 100)
                ++colored;
        }
    check(colored > 50, "截图应含选中候选的橙色点，不能只有背景");
    auto before = view.cameraState();
    drag(view, Qt::LeftButton, {400, 300}, {480, 350});
    auto rotated = view.cameraState();
    check(cameraDifference(before, rotated) > 0.01, "左键拖动必须改变三维相机");
    view.setSelectedCandidate(-1);
    check(cameraDifference(rotated, view.cameraState()) < 1e-9, "候选高亮不得重置用户视角");
    drag(view, Qt::MiddleButton, {400, 300}, {440, 320});
    auto panned = view.cameraState();
    check(cameraDifference(rotated, panned) > 0.01, "中键拖动必须平移相机");
    QWheelEvent wheel(QPointF(400, 300), QPointF(400, 300), QPoint(), QPoint(0, 120), Qt::NoButton, Qt::NoModifier,
                      Qt::NoScrollPhase, false);
    QApplication::sendEvent(&view, &wheel);
    QApplication::processEvents();
    check(cameraDifference(panned, view.cameraState()) > 0.01, "滚轮必须改变三维相机距离");
    check(view.saveScreenshot("cloud_rotated.png"), "旋转后的原生截图失败");
    view.setSceneVisible(false);
    view.setModelsVisible(false);
    view.setPointSize(5);
    view.setModelsVisible(true);
    view.resetCamera();
    view.resize(700, 500);
    QApplication::processEvents();
    check(view.saveScreenshot("cloud_resized.png"), "尺寸变化后截图失败");
    QImage resized("cloud_resized.png");
    check(resized.width() == int(view.width() * view.devicePixelRatioF()), "截图宽度须匹配设备像素尺寸");
    view.setData({});
    view.close();
    std::cout << "PASS 点云快照、真实 OpenGL 渲染、候选高亮、旋转、平移、滚轮缩放、重置与尺寸同步\n";
}
}
/** @brief 在 Qt 事件环境下验证 PCL 点云控件，失败返回非零状态。
 * @param argc 命令行参数数量。
 * @param argv 调用期间有效的参数列表。
 * @return 0 全部通过，1 标准异常，2 HALCON 异常。
 * @note 由 CTest 设置 QT_QPA_PLATFORM=windows，测试窗口不在桌面显示。 */
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    try
    {
        runTest();
        return 0;
    }
    catch (const HalconCpp::HException &e)
    {
        std::cerr << e.ErrorMessage().Text() << '\n';
        return 2;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
