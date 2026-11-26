#include "MainWindow.h"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QTemporaryDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QPlainTextEdit>
#include <QEventLoop>
#include <QTimer>
#include <QPushButton>
#include <QFileDialog>
#include <QDialog>
#include <QLabel>
#include <QTableWidget>
#include <QMouseEvent>
#include <QTabWidget>
#include <QPixmap>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdexcept>

namespace
{
/** @brief 检查真实 UI 与文件行为，失败时抛出中文原因。
 * @param condition 必须成立的条件。
 * @param message 失败场景。
 * @throws std::runtime_error 条件为 false。 */
void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
/** @brief 在 Qt 事件循环中等待自动保存计时器，保持信号和文件任务可执行。 */
void waitForSave()
{
    QEventLoop loop;
    QTimer::singleShot(700, &loop, &QEventLoop::quit);
    loop.exec();
}
/** @brief 通过图像控件的 Qt 事件通道模拟左键点击，覆盖真实取值过滤器。
 * @param label 非空图像显示标签，不转移所有权。
 * @param position 控件局部逻辑像素坐标。 */
void clickImage(QLabel *label, const QPointF &position)
{
    QMouseEvent press(QEvent::MouseButtonPress, position, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(label, &press);
    QApplication::processEvents();
}
/** @brief 根据实际居中 pixmap 计算指定原图像素中心的控件位置，向真实事件通道发送点击。
 * @param label 显示等比例图像的标签，测试中使用居中对齐。
 * @param original 原始图像宽高，正整数。
 * @param pixel 需要验证的原图列/行，须在原图内。 */
void clickOriginalPixel(QLabel *label, const QSize &original, const QPoint &pixel)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    auto pixmap = label->pixmap();
#else
    auto pixmap = label->pixmap(Qt::ReturnByValue);
#endif
    require(!pixmap.isNull(), "点击测试前必须已有显示图像");
    auto size = (QSizeF(pixmap.size()) / pixmap.devicePixelRatio()).toSize();
    int margin = label->margin();
    auto contents = label->contentsRect().adjusted(margin, margin, -margin, -margin);
    // QLabel 分别对两项做整数除法；先相减再除会在奇偶尺寸不同时产生一像素偏差。
    int left = contents.x() + contents.width() / 2 - size.width() / 2,
        top = contents.y() + contents.height() / 2 - size.height() / 2;
    clickImage(label, {left + (pixel.x() + 0.5) * size.width() / original.width(),
                       top + (pixel.y() + 0.5) * size.height() / original.height()});
}
/** @brief 将有明确数值的深度测试图编码到隔离文件，再从实际打开按钮加载。
 * @param window 待验证窗口，文件选择使用 Qt 对话框。
 * @param depth 有效单通道 uint16 或 float32 测试深度。
 * @param path 隔离目录中的 TIFF 路径，允许中文。
 * @throws std::runtime_error 文件编码、写入或打开动作失败。 */
void loadDepthFixture(MainWindow &window, const cv::Mat &depth, const QString &path)
{
    std::vector<uchar> bytes;
    require(cv::imencode(".tiff", depth, bytes), "深度测试图编码失败");
    {
        QFile file(path);
        require(file.open(QIODevice::WriteOnly), "深度测试文件创建失败");
        require(file.write(reinterpret_cast<const char *>(bytes.data()), qint64(bytes.size())) == qint64(bytes.size()),
                "深度测试文件写入不完整");
    }
    bool selected = false;
    QTimer::singleShot(0, &window,
                       [&path, &selected]()
                       {
                           for (auto *widget : QApplication::topLevelWidgets())
                               if (auto *dialog = qobject_cast<QFileDialog *>(widget))
                               {
                                   dialog->selectFile(path);
                                   selected = true;
                                   static_cast<QDialog *>(dialog)->accept();
                                   break;
                               }
                       });
    window.findChild<QPushButton *>("openDepthButton")->click();
    QApplication::processEvents();
    require(selected, "深度加载按钮必须打开文件选择框");
}
/** @brief 验证真实点击读取原图值、留白边界、设备像素比、窗口缩放和深度换算。
 * @details 使用隔离的已知 BGR、uint16、float32 图，验证 65535、NaN、Inf 与零深度不会被显示归一化替代。 */
void testImageValues()
{
    QTemporaryDir directory(QCoreApplication::applicationDirPath() + "/pixel_values_XXXXXX");
    require(directory.isValid(), "无法创建像素测试目录");
    MainWindow window(nullptr, directory.path());
    window.setAttribute(Qt::WA_DontShowOnScreen);
    auto *tabs = window.findChild<QTabWidget *>("viewTabs");
    auto *image = window.findChild<QLabel *>("imageView");
    auto *depthView = window.findChild<QLabel *>("depthView");
    auto *imageValue = window.findChild<QLabel *>("imagePixelValue");
    auto *depthValue = window.findChild<QLabel *>("depthPixelValue");
    require(tabs && image && depthView && imageValue && depthValue, "图像控件和像素读数控件须完整");
    image->setFixedSize(720, 400);
    image->setMargin(6);
    depthView->setFixedSize(720, 400);
    tabs->setCurrentIndex(0);
    window.show();
    window.loadSyntheticImages();
    QApplication::processEvents();
    clickOriginalPixel(image, {320, 240}, {100, 80});
    require(imageValue->text().contains("X=100, Y=80") && imageValue->text().contains("BGR=(100, 80, 180)"),
            "2D 点击应读取原图坐标和 BGR 值");
    clickImage(image, {image->contentsRect().left() + 0.1, image->height() / 2.0});
    require(imageValue->text().contains(QStringLiteral("留白")), "居中图像的留白不得映射为有效像素");
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    auto pixmap = image->pixmap();
#else
    auto pixmap = image->pixmap(Qt::ReturnByValue);
#endif
    auto doubled = pixmap.scaled(pixmap.width() * 2, pixmap.height() * 2);
    doubled.setDevicePixelRatio(2);
    image->setPixmap(doubled);
    clickOriginalPixel(image, {320, 240}, {200, 100});
    require(imageValue->text().contains("X=200, Y=100") && imageValue->text().contains("BGR=(200, 100, 180)"),
            "高 DPI pixmap 坐标映射应准确");
    clickOriginalPixel(image, {320, 240}, {319, 239});
    require(imageValue->text().contains("X=319, Y=239"), "图像最后一个像素须可准确选中");
    tabs->setCurrentIndex(1);
    QApplication::processEvents();
    cv::Mat depth = (cv::Mat_<uint16_t>(2, 3) << 0, 1200, 65535, 100, 200, 300);
    loadDepthFixture(window, depth, directory.filePath(QStringLiteral("原始16位深度.tiff")));
    window.findChild<QDoubleSpinBox *>("scaleSpin")->setValue(0.1);
    window.findChild<QDoubleSpinBox *>("offsetSpin")->setValue(10);
    clickOriginalPixel(depthView, {3, 2}, {1, 0});
    require(depthValue->text().contains("X=1, Y=0") && depthValue->text().contains(QStringLiteral("原始深度=1200")) &&
                depthValue->text().contains("Z=130"),
            "深度点击应保留原值并应用比例偏移");
    window.findChild<QDoubleSpinBox *>("scaleSpin")->setValue(0.2);
    require(depthValue->text().contains("Z=250"), "修改比例后选中像素的深度读数应刷新");
    clickOriginalPixel(depthView, {3, 2}, {2, 0});
    require(depthValue->text().contains(QStringLiteral("原始深度=65535")), "16 位深度不能被归一化成 8 位颜色值");
    clickOriginalPixel(depthView, {3, 2}, {0, 0});
    require(depthValue->text().contains(QStringLiteral("原始深度=0")) &&
                depthValue->text().contains(QStringLiteral("无效深度")),
            "零深度不能被偏移变成有效深度");
    cv::Mat floating = (cv::Mat_<float>(2, 3) << std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity(), -1, 0, 12.5f, 500.25f);
    loadDepthFixture(window, floating, directory.filePath(QStringLiteral("原始浮点深度.tiff")));
    clickOriginalPixel(depthView, {3, 2}, {0, 0});
    require(depthValue->text().contains("NaN") && depthValue->text().contains(QStringLiteral("无效深度")),
            "原始 NaN 深度须明确显示并标记无效");
    clickOriginalPixel(depthView, {3, 2}, {1, 0});
    require(depthValue->text().contains("+Inf") && depthValue->text().contains(QStringLiteral("无效深度")),
            "原始 Inf 深度须明确显示并标记无效");
    clickOriginalPixel(depthView, {3, 2}, {1, 1});
    require(depthValue->text().contains(QStringLiteral("原始深度=12.5")) && depthValue->text().contains("Z=12.5"),
            "浮点深度的原值与换算值须正确");
    image->setMinimumSize(400, 300);
    image->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    tabs->setCurrentIndex(0);
    window.resize(1600, 1020);
    QApplication::processEvents();
    clickOriginalPixel(image, {320, 240}, {150, 120});
    require(imageValue->text().contains("X=150, Y=120") && imageValue->text().contains("BGR=(150, 120, 180)"),
            "窗口尺寸改变后取值坐标仍须准确");
    std::cout << "PASS 图像取值、缩放留白、DPI、16 位深度、浮点无效值及换算刷新\n";
}
/** @brief 在 Qt 事件循环中等待后台任务与完成槽，避免在 UI 线程阻塞信号派发。
 * @param window 正在执行测试任务的窗口，只读借用其忙状态。
 * @throws std::runtime_error 任务三十秒内未结束。 */
void waitForTask(MainWindow &window)
{
    QEventLoop loop;
    QTimer poll;
    poll.setInterval(10);
    QObject::connect(&poll, &QTimer::timeout, &loop,
                     [&window, &loop]()
                     {
                         if (!window.isBusy())
                             loop.quit();
                     });
    QTimer::singleShot(30000, &loop, &QEventLoop::quit);
    poll.start();
    loop.exec();
    QApplication::processEvents();
    require(!window.isBusy(), "后台任务未在测试时限内结束");
}
/** @brief 从真实按钮验证模板预览、PLY 导出和四参数抓手转换，检查候选保留与参数分支。
 * @details 全部输入为测试合成数据，输出位于自动清理的隔离目录；不使用用户图片或配置。 */
void testPointCloudActions()
{
    QTemporaryDir directory(QCoreApplication::applicationDirPath() + "/cloud_actions_XXXXXX");
    require(directory.isValid(), "无法创建点云操作测试目录");
    MainWindow window(nullptr, directory.path());
    require(!window.findChild<QPushButton *>("syntheticButton") &&
                !window.findChild<QPushButton *>("createModelButton"),
            "旧的两个按钮须从 Designer 界面移除");
    auto *save = window.findChild<QPushButton *>("savePointCloudButton");
    auto *show = window.findChild<QPushButton *>("showModelButton");
    require(save && show, "新增点云保存与模型显示按钮须存在");
    window.loadSyntheticImages();
    window.runSmokeWorkflow();
    waitForTask(window);
    require(window.hasMatches(), "操作验证前应完成实际建模匹配");
    auto *table = window.findChild<QTableWidget *>("resultTable");
    int count = table->rowCount();
    auto x = table->item(0, 1)->text();
#ifdef SM_ENABLE_PCL_VIEWER
    show->click();
    waitForTask(window);
    auto *description = window.findChild<QLabel *>("cloudInfoLabel");
    require(description && description->text().startsWith(QStringLiteral("模型预览：")),
            "预览须使用独立说明，不把单位变换计为匹配");
    require(table->rowCount() == count && table->item(0, 1)->text() == x, "模型预览不得改变已有匹配结果");
#endif
    auto path = directory.filePath(QStringLiteral("完整深度点云.ply"));
    bool selected = false;
    // 在 Qt 自己的文件对话框中选择隔离输出，覆盖真实按钮到后台写盘的流程。
    QTimer::singleShot(0, &window,
                       [&path, &selected]()
                       {
                           for (auto *widget : QApplication::topLevelWidgets())
                               if (auto *dialog = qobject_cast<QFileDialog *>(widget))
                               {
                                   dialog->selectFile(path);
                                   selected = true;
                                   static_cast<QDialog *>(dialog)->accept();
                                   break;
                               }
                       });
    save->click();
    require(selected, "点云保存按钮应打开文件对话框");
    waitForTask(window);
    require(QFile::exists(path), "点云保存按钮应生成 PLY 文件");
    HalconCpp::HTuple status;
    HalconCpp::HObjectModel3D reloaded(HalconCpp::HString(path.toStdWString().c_str()), HalconCpp::HTuple("m"),
                                       HalconCpp::HTuple(), HalconCpp::HTuple(), &status);
    require(reloaded.GetObjectModel3dParams("num_points").L() == 28500, "PLY 导出须保留当前深度的完整有效点");
    require(table->rowCount() == count && table->item(0, 1)->text() == x, "保存点云不得改变匹配结果");
    auto *eyeInHand = window.findChild<QCheckBox *>("eyeInHandCheck");
    auto *robot = window.findChild<QLineEdit *>("robotEdit");
    auto *handEye = window.findChild<QLineEdit *>("handEyeEdit");
    auto *grasp = window.findChild<QLabel *>("graspResultLabel");
    require(eyeInHand && robot && handEye && grasp, "四参数抓手控件必须完整");
    eyeInHand->setChecked(false);
    require(!robot->isEnabled(), "眼在手外应禁用不需要的机器人输入");
    handEye->setText("100,0,0,0,0,90");
    robot->setText("null");
    // 新接口不读取旧示教字段；保留字段只为兼容已有 JSON，错误的旧值不能阻止新转换。
    window.findChild<QLineEdit *>("taughtModelEdit")->setText("旧配置无效");
    window.findChild<QLineEdit *>("taughtTcpEdit")->setText("旧配置无效");
    window.findChild<QLineEdit *>("referenceEdit")->setText("旧配置无效");
    window.findChild<QLineEdit *>("centerEdit")->setText("旧配置无效");
    window.findChild<QPushButton *>("transformButton")->click();
    require(grasp->text().startsWith(QStringLiteral("抓手相对位移 Pose（基座系）")),
            "抓手按钮应走四参数相对位移接口且忽略固定相机的机器人文本");
    require(window.findChild<QLineEdit *>("taughtTcpEdit")->isHidden(), "旧版绝对抓取配置应隐藏，避免误导输入");
    eyeInHand->setChecked(true);
    require(robot->isEnabled(), "眼在手上应启用拍照机器人位姿");
    robot->setText("100,20,30,0,0,90");
    window.findChild<QPushButton *>("transformButton")->click();
    auto *log = window.findChild<QPlainTextEdit *>("logEdit");
    require(log && log->toPlainText().split('\n').last().contains(QStringLiteral("抓手相对位移")),
            "眼在手上的四参数调用应成功");
    auto resultPath = directory.filePath(QStringLiteral("相对抓手结果.json"));
    selected = false;
    QTimer::singleShot(0, &window,
                       [&resultPath, &selected]()
                       {
                           for (auto *widget : QApplication::topLevelWidgets())
                               if (auto *dialog = qobject_cast<QFileDialog *>(widget))
                               {
                                   dialog->selectFile(resultPath);
                                   selected = true;
                                   static_cast<QDialog *>(dialog)->accept();
                                   break;
                               }
                       });
    window.findChild<QPushButton *>("exportResultsButton")->click();
    require(selected, "抓手导出应选择目标文件");
    QFile resultFile(resultPath);
    require(resultFile.open(QIODevice::ReadOnly), "相对抓手结果应写入文件");
    auto exported = QJsonDocument::fromJson(resultFile.readAll()).object();
    require(exported["outputPoseConvention"].toString() == "base_relative_left_multiply", "导出须标明相对位移约定");
    auto candidate = exported["candidates"].toArray().first().toObject();
    require(candidate["outputPose"].toArray().size() == 7 && !candidate.contains("tcpInBase"),
            "导出须包含相对七元 pose，不得伪造绝对抓取位姿");
    std::cout << "PASS 新增模型预览与完整 PLY 保存、中文路径、候选结果保留\n";
    std::cout << "PASS 四参数抓手按钮、相机安装分支和相对结果导出\n";
}
/** @brief 读取隔离测试配置，不访问用户的实际参数文件。
 * @param path 测试生成的 JSON 文件绝对路径。
 * @return 已解析的配置对象。
 * @throws std::runtime_error 无法读取或文件不是 JSON 对象。 */
QJsonObject readConfig(const QString &path)
{
    QFile file(path);
    require(file.open(QIODevice::ReadOnly), "配置文件应能读取");
    auto doc = QJsonDocument::fromJson(file.readAll());
    require(doc.isObject(), "配置必须为合法 JSON");
    return doc.object();
}
/** @brief 通过 Designer 控件实际编辑，验证延迟保存、关闭保存、重启恢复及损坏文件保留。 */
void runTest()
{
    QTemporaryDir directory(QCoreApplication::applicationDirPath() + "/settings_test_XXXXXX");
    require(directory.isValid(), "无法创建隔离测试目录");
    auto path = directory.filePath("surface_match_settings.json");
    {
        MainWindow window(nullptr, directory.path());
        auto *fx = window.findChild<QDoubleSpinBox *>("fxSpin");
        auto *driver = window.findChild<QComboBox *>("driverCombo");
        auto *runtime = window.findChild<QLineEdit *>("runtimeEdit");
        auto *use2D = window.findChild<QCheckBox *>("use2DCheck");
        auto *size = window.findChild<QSpinBox *>("pointSizeSpin");
        require(fx && driver && runtime && use2D && size, "Designer 控件名称须完整可定位");
        fx->setValue(1234.56789);
        driver->setCurrentIndex(1);
        runtime->setText(QStringLiteral("E:/测试目录/原生库"));
        use2D->setChecked(true);
        size->setValue(5);
        waitForSave();
        auto saved = readConfig(path);
        require(saved["fx"].toDouble() == fx->value() && saved["driver"].toInt() == 1 &&
                    saved["pointSize"].toInt() == 5,
                "编辑后应自动保存数值、枚举与三维外观");
        require(saved["runtime"].toString() == runtime->text() && saved["use2D"].toBool(), "中文路径和开关须保存");
        fx->setValue(8765.4321);
        window.close();
        require(readConfig(path)["fx"].toDouble() == fx->value(), "关闭时须立即保存未到延迟时间的最后修改");
    }
    {
        MainWindow restored(nullptr, directory.path());
        require(restored.findChild<QDoubleSpinBox *>("fxSpin")->value() == 8765.4321, "重启须恢复上次内参");
        require(restored.findChild<QCheckBox *>("use2DCheck")->isChecked() &&
                    restored.findChild<QSpinBox *>("pointSizeSpin")->value() == 5,
                "重启须恢复开关与点大小");
        require(restored.findChild<QLineEdit *>("runtimeEdit")->text() == QStringLiteral("E:/测试目录/原生库"),
                "重启须恢复中文路径");
    }
    QByteArray invalid = "{broken-json";
    {
        QFile file(path);
        require(file.open(QIODevice::WriteOnly | QIODevice::Truncate), "准备损坏配置失败");
        require(file.write(invalid) == invalid.size(), "写入损坏配置失败");
    }
    {
        MainWindow restored(nullptr, directory.path());
        require(restored.findChild<QDoubleSpinBox *>("fxSpin")->value() == 1000, "损坏配置应保持 Designer 默认值");
    }
    {
        QFile file(path);
        require(file.open(QIODevice::ReadOnly) && file.readAll() == invalid, "没有用户编辑时不得覆盖损坏的原文件");
    }
    std::cout << "PASS Designer 控件绑定、自动保存、关闭落盘、重启恢复、中文路径和损坏配置保留\n";
}
}
/** @brief 在无窗口 Qt 平台验证参数、点云操作和图像点击取值，文件全部写在自动清理的隔离目录。
 * @param argc 系统参数数量。
 * @param argv 系统参数列表，调用期间有效。
 * @return 全部通过为 0，异常为 1。 */
int main(int argc, char *argv[])
{
    QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    QApplication app(argc, argv);
    try
    {
        runTest();
        testPointCloudActions();
        testImageValues();
        return 0;
    }
    catch (const HalconCpp::HException &e)
    {
        std::cerr << e.ProcName().Text() << ": " << e.ErrorMessage().Text() << '\n';
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
