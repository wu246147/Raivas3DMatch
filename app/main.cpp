#include "MainWindow.h"
#include <QApplication>
#include <QTimer>
#include <QPixmap>
#include <QFontDatabase>
#ifdef SM_ENABLE_PCL_VIEWER
#include "CloudView.h"
#endif
/** @brief 启动 Qt 测试程序，支持无交互 smoke 渲染检查。
 * @param argc 系统命令行参数数量。
 * @param argv 系统参数列表，--smoke 时隐藏窗口、生成合成图并保存截图后退出。
 * @return Qt 事件循环状态，截图或匹配失败返回 2，超时返回 3。
 * @note GUI 与 HALCON 资源由栈对象和 Qt 父子关系自动释放。 */
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    if (app.arguments().contains("--smoke"))
    {
        QFontDatabase::addApplicationFont("C:/Windows/Fonts/msyh.ttc");
        app.setFont(QFont("Microsoft YaHei", 9));
    }
    MainWindow window;
    if (app.arguments().contains("--smoke"))
        window.setAttribute(Qt::WA_DontShowOnScreen);
    window.show();
    if (app.arguments().contains("--smoke"))
    {
        window.loadSyntheticImages();
        window.runSmokeWorkflow();
        auto *poll = new QTimer(&window);
        poll->setInterval(200);
        QObject::connect(poll, &QTimer::timeout, &window,
                         [&window, &app, poll]()
                         {
                             if (window.isBusy())
                                 return;
                             poll->stop();
                             bool ok = window.grab().save("surface_match_smoke.png");
#ifdef SM_ENABLE_PCL_VIEWER
                             if (QApplication::platformName() == "windows")
                             {
                                 auto *cloud = dynamic_cast<CloudView *>(window.findChild<QWidget *>("pclCloudView"));
                                 ok = ok && cloud && cloud->saveScreenshot("surface_match_native_ui.png");
                             }
#endif
                             app.exit(ok && window.hasMatches() ? 0 : 2);
                         });
        poll->start();
        QTimer::singleShot(60000, &app, [&app]() { app.exit(3); });
    }
    return app.exec();
}
