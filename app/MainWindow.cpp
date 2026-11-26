#include "MainWindow.h"
#include "ui_MainWindow.h"
#ifdef SM_ENABLE_PCL_VIEWER
#include "CloudView.h"
#endif
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFile>
#include <QApplication>
#include <QDir>
#include <QTimer>
#include <QScopedValueRollback>
#include <QMouseEvent>
#include <QStyle>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

using namespace HalconCpp;
namespace
{
/** @brief 将 UTF-8 标准字符串转换为 Qt 字符串。
 * @param text UTF-8 文本。
 * @return Qt Unicode 文本。 */
QString qstr(const std::string &text)
{
    return QString::fromUtf8(text.c_str());
}
/** @brief 从文本解析固定数量有限浮点数。
 * @param text 用逗号、空白或分号分隔的数字。
 * @param count 要求的元素数量。
 * @return 按原顺序解析的数字。
 * @throws std::invalid_argument 数量、格式或有限性不满足约定。 */
std::vector<double> values(const QString &text, int count)
{
    auto parts = text.split(QRegularExpression("[,;，\\s]+"), Qt::SkipEmptyParts);
    if (parts.size() != count)
        throw std::invalid_argument("数值数量错误，需要 " + std::to_string(count) + " 项");
    std::vector<double> out;
    for (const auto &part : parts)
    {
        bool ok = false;
        double v = part.toDouble(&ok);
        if (!ok || !std::isfinite(v))
            throw std::invalid_argument("位姿文本含无效数值");
        out.push_back(v);
    }
    return out;
}
/** @brief 将 HALCON pose 格式化为六项可读数值。
 * @param pose 至少六项数值的 pose。
 * @return 三位小数文本，空 pose 返回空串。 */
QString poseText(const HTuple &pose)
{
    QStringList out;
    for (Hlong i = 0; i < std::min<Hlong>(pose.Length(), 6); ++i)
        out << QString::number(pose[i].D(), 'f', 3);
    return out.join(", ");
}
/** @brief 将元组写为 JSON 数组，保留第七项 pose 类型。
 * @param tuple 有限数值元组。
 * @return 完整数值数组。 */
QJsonArray tupleJson(const HTuple &tuple)
{
    QJsonArray out;
    for (Hlong i = 0; i < tuple.Length(); ++i)
        out.append(tuple[i].D());
    return out;
}
/** @brief 将像素数值格式化为可读文本，并明确表示非有限值。
 * @param value 原始或换算后的数值。
 * @param digits 有效数字位数，float32 使用 9，double 使用 17。
 * @return 不依赖显示伪彩色的数值文本；非有限值为 NaN、+Inf 或 -Inf。 */
QString pixelNumber(double value, int digits)
{
    if (std::isnan(value))
        return "NaN";
    if (std::isinf(value))
        return value > 0 ? "+Inf" : "-Inf";
    return QString::number(value, 'g', digits);
}
} // 匿名命名空间

MainWindow::MainWindow(QWidget *parent, const QString &settingsDirectory)
    : QMainWindow(parent), ui_(std::make_unique<Ui::MainWindow>())
{
    ui_->setupUi(this);
    controls_ = ui_->sidebarWidget;
    numbers_["fx"] = ui_->fxSpin;
    numbers_["fy"] = ui_->fySpin;
    numbers_["cx"] = ui_->cxSpin;
    numbers_["cy"] = ui_->cySpin;
    numbers_["scale"] = ui_->scaleSpin;
    numbers_["offset"] = ui_->offsetSpin;
    numbers_["xmin"] = ui_->xminSpin;
    numbers_["xmax"] = ui_->xmaxSpin;
    numbers_["ymin"] = ui_->yminSpin;
    numbers_["ymax"] = ui_->ymaxSpin;
    numbers_["zmin"] = ui_->zminSpin;
    numbers_["zmax"] = ui_->zmaxSpin;
    numbers_["count"] = ui_->countSpin;
    numbers_["score"] = ui_->scoreSpin;
    numbers_["overlap"] = ui_->overlapSpin;
    numbers_["modelSmooth"] = ui_->modelSmoothSpin;
    numbers_["sceneSmooth"] = ui_->sceneSmoothSpin;
    numbers_["downsample"] = ui_->downsampleSpin;
    numbers_["expand"] = ui_->expandSpin;
    numbers_["angle"] = ui_->angleSpin;
    numbers_["fileScale"] = ui_->fileScaleSpin;
    numbers_["threshold2D"] = ui_->threshold2DSpin;
    accuracy_ = ui_->accuracyCombo;
    driver_ = ui_->driverCombo;
    kind_ = ui_->kindCombo;
    offsetMode_ = ui_->offsetModeCombo;
    use2D_ = ui_->use2DCheck;
    gpu_ = ui_->gpuCheck;
    yolo26_ = ui_->yolo26Check;
    eyeInHand_ = ui_->eyeInHandCheck;
    symmetry_ = ui_->symmetryCheck;
    attitude_ = ui_->attitudeSpin;
    model2D_ = ui_->model2DEdit;
    runtime_ = ui_->runtimeEdit;
    depthRuntime_ = ui_->depthRuntimeEdit;
    handEye_ = ui_->handEyeEdit;
    robot_ = ui_->robotEdit;
    taughtModel_ = ui_->taughtModelEdit;
    taughtTcp_ = ui_->taughtTcpEdit;
    reference_ = ui_->referenceEdit;
    center_ = ui_->centerEdit;
    imageView_ = ui_->imageView;
    depthView_ = ui_->depthView;
    resultView_ = ui_->resultView;
    modelLabel_ = ui_->modelStatusLabel;
    graspText_ = ui_->graspResultLabel;
    table_ = ui_->resultTable;
    log_ = ui_->logEdit;
    for (auto *label : {imageView_, depthView_, resultView_})
    {
        label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        label->setScaledContents(false);
        label->installEventFilter(this);
    }
    imageView_->setCursor(Qt::CrossCursor);
    depthView_->setCursor(Qt::CrossCursor);
    connect(numbers_.at("scale"), qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this]() { updatePixelReadout(true); });
    connect(numbers_.at("offset"), qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this]() { updatePixelReadout(true); });
    runtime_->setText(QString::fromUtf8(SM_RAIVAS_RUNTIME_DIR));
    depthRuntime_->setText(QString::fromUtf8(SM_DEPTH_RUNTIME_DIR));
    ui_->mainSplitter->setStretchFactor(1, 1);
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    connect(ui_->open2DButton, &QPushButton::clicked, this, [this]() { openImage(false); });
    connect(ui_->openDepthButton, &QPushButton::clicked, this, [this]() { openImage(true); });
    connect(ui_->filterDepthButton, &QPushButton::clicked, this, &MainWindow::filterDepth);
    connect(ui_->loadModelButton, &QPushButton::clicked, this, [this]() { loadModel(false); });
    connect(ui_->runButton, &QPushButton::clicked, this, &MainWindow::runModel);
    connect(ui_->transformButton, &QPushButton::clicked, this, &MainWindow::transformSelected);
    connect(eyeInHand_, &QCheckBox::toggled, robot_, &QWidget::setEnabled);
    connect(eyeInHand_, &QCheckBox::toggled, ui_->robotEditLabel, &QWidget::setEnabled);
    connect(ui_->importSettingsButton, &QPushButton::clicked, this, [this]() { settingsFile(false); });
    connect(ui_->exportSettingsButton, &QPushButton::clicked, this, [this]() { settingsFile(true); });
    connect(ui_->exportResultsButton, &QPushButton::clicked, this, &MainWindow::exportResults);
    connect(ui_->browseModelButton, &QPushButton::clicked, this,
            [this]()
            {
                auto file = QFileDialog::getOpenFileName(this, QStringLiteral("选择 2D 模型"), {},
                                                         "ONNX (*.onnx);;All files (*)");
                if (!file.isEmpty())
                    model2D_->setText(file);
            });
    connect(ui_->saveModelButton, &QPushButton::clicked, this,
            [this]()
            {
                if (!model_)
                {
                    log(QStringLiteral("请先加载模型"));
                    return;
                }
                auto file =
                    QFileDialog::getSaveFileName(this, QStringLiteral("保存表面模型"), {}, "Surface model (*.sfm)");
                if (file.isEmpty())
                    return;
                auto model = model_;
                startTask(
                    [model, file]()
                    {
                        sm::SurfaceMatch::saveModel(*model, file.toUtf8().toStdString());
                        TaskResult r;
                        r.message = QStringLiteral("表面模型已保存");
                        return r;
                    });
            });
    connect(ui_->previewCloudButton, &QPushButton::clicked, this, &MainWindow::previewPointCloud);
    connect(ui_->savePointCloudButton, &QPushButton::clicked, this, &MainWindow::savePointCloud);
    connect(ui_->showModelButton, &QPushButton::clicked, this, &MainWindow::showModel);
    connect(ui_->saveCloudButton, &QPushButton::clicked, this, &MainWindow::saveCloudScreenshot);
    connect(table_, &QTableWidget::itemSelectionChanged, this, &MainWindow::selectCloudCandidate);
#ifdef SM_ENABLE_PCL_VIEWER
    ui_->cloudPlaceholder->hide();
    cloudView_ = new CloudView(ui_->cloudHostLayout->parentWidget());
    cloudView_->setObjectName("pclCloudView");
    ui_->cloudHostLayout->addWidget(cloudView_);
    connect(ui_->resetCameraButton, &QPushButton::clicked, cloudView_, &CloudView::resetCamera);
    connect(ui_->sceneVisibleCheck, &QCheckBox::toggled, cloudView_, &CloudView::setSceneVisible);
    connect(ui_->matchesVisibleCheck, &QCheckBox::toggled, cloudView_, &CloudView::setModelsVisible);
    connect(ui_->pointSizeSpin, qOverload<int>(&QSpinBox::valueChanged), cloudView_, &CloudView::setPointSize);
#else
    ui_->cloudPlaceholder->setText(QStringLiteral("当前构建未启用 PCL 点云视图，请设置 SM_ENABLE_PCL_VIEWER=ON。"));
    for (auto *button : {ui_->previewCloudButton, ui_->saveCloudButton, ui_->resetCameraButton, ui_->showModelButton})
        button->setEnabled(false);
#endif
    connect(&watcher_, &QFutureWatcher<TaskResult>::finished, this, &MainWindow::finishTask);
    refreshImages();
    log(QStringLiteral("就绪：打开深度图、设置内参后可预览 3D 点云；加载模型后运行匹配。"));
    settingsPath_ = QDir(settingsDirectory.isEmpty() ? QCoreApplication::applicationDirPath() : settingsDirectory)
                        .filePath("surface_match_settings.json");
    settingsEnabled_ = !QCoreApplication::arguments().contains("--smoke") || !settingsDirectory.isEmpty();
    settingsSaveTimer_ = new QTimer(this);
    settingsSaveTimer_->setSingleShot(true);
    settingsSaveTimer_->setInterval(500);
    connect(settingsSaveTimer_, &QTimer::timeout, this, &MainWindow::saveAutomaticSettings);
    if (settingsEnabled_)
        restoreAutomaticSettings();
    robot_->setEnabled(eyeInHand_->isChecked());
    ui_->robotEditLabel->setEnabled(eyeInHand_->isChecked());
    for (auto *box : findChildren<QDoubleSpinBox *>())
        connect(box, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::scheduleSettingsSave);
    for (auto *box : findChildren<QSpinBox *>())
        connect(box, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::scheduleSettingsSave);
    for (auto *box : findChildren<QComboBox *>())
        connect(box, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::scheduleSettingsSave);
    for (auto *box : findChildren<QCheckBox *>())
        connect(box, &QCheckBox::toggled, this, &MainWindow::scheduleSettingsSave);
    for (auto *edit : findChildren<QLineEdit *>())
        connect(edit, &QLineEdit::textChanged, this, &MainWindow::scheduleSettingsSave);
    if (settingsEnabled_)
        log(QStringLiteral("参数自动记录：%1").arg(settingsPath_));
}
MainWindow::~MainWindow()
{
    if (settingsDirty_)
        saveAutomaticSettings();
    watcher_.waitForFinished();
}
void MainWindow::closeEvent(QCloseEvent *event)
{
    if (watcher_.isRunning())
    {
        event->ignore();
        log(QStringLiteral("后台算法仍在执行，请等待完成后关闭窗口。"));
    }
    else
    {
        saveAutomaticSettings();
        event->accept();
    }
}
sm::CameraIntrinsics MainWindow::camera() const
{
    sm::CameraIntrinsics c;
    c.fx = numbers_.at("fx")->value();
    c.fy = numbers_.at("fy")->value();
    c.cx = numbers_.at("cx")->value();
    c.cy = numbers_.at("cy")->value();
    c.depthScale = numbers_.at("scale")->value();
    c.depthOffset = numbers_.at("offset")->value();
    c.validate();
    return c;
}
sm::Bounds3D MainWindow::bounds() const
{
    sm::Bounds3D b;
    b.min = {numbers_.at("xmin")->value(), numbers_.at("ymin")->value(), numbers_.at("zmin")->value()};
    b.max = {numbers_.at("xmax")->value(), numbers_.at("ymax")->value(), numbers_.at("zmax")->value()};
    b.validate();
    return b;
}
sm::MatchParameters MainWindow::parameters() const
{
    sm::MatchParameters p;
    sm::SurfaceMatch::setPara(p, sm::MatchingAccuracy(accuracy_->currentIndex()));
    p.numMatches = int(numbers_.at("count")->value());
    p.minScore = numbers_.at("score")->value();
    p.maxOverlapDistance = numbers_.at("overlap")->value();
    p.modelSmoothK = int(numbers_.at("modelSmooth")->value());
    p.sceneSmoothK = int(numbers_.at("sceneSmooth")->value());
    p.downsampleDistance = numbers_.at("downsample")->value();
    p.roiExpansion = int(numbers_.at("expand")->value());
    p.angleTolerance = numbers_.at("angle")->value();
    p.validate();
    return p;
}
sm::DetectorOptions MainWindow::detector() const
{
    sm::DetectorOptions d;
    d.modelPath = model2D_->text().toUtf8().toStdString();
    d.runtimeDirectory = runtime_->text().toUtf8().toStdString();
    d.kind = sm::DetectorKind(kind_->currentIndex());
    d.driver = sm::DetectorDriver(driver_->currentIndex());
    d.useGpu = gpu_->isChecked();
    d.yolo26 = yolo26_->isChecked();
    d.threshold = numbers_.at("threshold2D")->value();
    return d;
}
void MainWindow::openImage(bool depth)
{
    auto path =
        QFileDialog::getOpenFileName(this, depth ? QStringLiteral("打开原始深度图") : QStringLiteral("打开 2D 图片"),
                                     {}, "Images (*.png *.tif *.tiff *.bmp *.jpg *.jpeg *.exr);;All files (*)");
    if (path.isEmpty())
        return;
    try
    {
        auto mat = sm::Tool::readImage(path.toUtf8().toStdString(), depth);
        if (depth)
        {
            if (mat.channels() != 1 ||
                (mat.depth() != CV_16U && mat.depth() != CV_16S && mat.depth() != CV_32F && mat.depth() != CV_64F))
                throw std::invalid_argument("深度图须为单通道 16 位整数或 32/64 位浮点图");
            depth_ = mat;
            selectedDepthPixel_.reset();
        }
        else
        {
            image_ = mat;
            selectedImagePixel_.reset();
        }
        result_ = {};
        table_->setRowCount(0);
        overlay_.release();
#ifdef SM_ENABLE_PCL_VIEWER
        cloudView_->setData({});
        ui_->cloudInfoLabel->setText(QStringLiteral("输入已改变，请重新显示点云或运行匹配"));
#endif
        refreshImages();
        log(QStringLiteral("已加载 %1，%2 × %3，OpenCV 类型 %4").arg(path).arg(mat.cols).arg(mat.rows).arg(mat.type()));
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
}
void MainWindow::showImage(QLabel *label, const cv::Mat &image)
{
    if (image.empty())
    {
        label->setPixmap({});
        label->setText(QStringLiteral("尚无图像"));
        return;
    }
    cv::Mat display;
    if (image.channels() == 1 && image.depth() != CV_8U)
    {
        cv::Mat source;
        image.convertTo(source, CV_32F);
        cv::patchNaNs(source, 0);
        cv::Mat valid = (source > 0) & (source < std::numeric_limits<float>::infinity());
        double low = 0, high = 0;
        if (cv::countNonZero(valid))
            cv::minMaxLoc(source, &low, &high, nullptr, nullptr, valid);
        cv::Mat normalized = cv::Mat::zeros(source.size(), CV_8U);
        if (high > low)
            source.convertTo(normalized, CV_8U, 255 / (high - low), -low * 255 / (high - low));
        else
            normalized.setTo(128, valid);
        cv::applyColorMap(normalized, display, cv::COLORMAP_TURBO);
        display.setTo(cv::Scalar(0, 0, 0), ~valid);
    }
    else if (image.channels() == 1)
        cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
    else
        display = image;
    cv::Mat rgb;
    cv::cvtColor(display, rgb, cv::COLOR_BGR2RGB);
    QImage qimage(rgb.data, rgb.cols, rgb.rows, int(rgb.step), QImage::Format_RGB888);
    int margin = label->margin();
    auto size = label->contentsRect().adjusted(margin, margin, -margin, -margin).size();
    if (size.width() > 0 && size.height() > 0)
        label->setPixmap(QPixmap::fromImage(qimage.copy()).scaled(size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    auto *label = qobject_cast<QLabel *>(watched);
    if (label == imageView_ || label == depthView_ || label == resultView_)
    {
        if (event->type() == QEvent::Resize)
            showImage(label, label == imageView_ ? image_ : label == depthView_ ? depth_ : overlay_);
        else if (event->type() == QEvent::MouseButtonPress && (label == imageView_ || label == depthView_))
        {
            auto *mouse = static_cast<QMouseEvent *>(event);
            if (mouse->button() == Qt::LeftButton)
            {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
                inspectImagePixel(label == depthView_, mouse->position());
#else
                inspectImagePixel(label == depthView_, mouse->localPos());
#endif
                return true;
            }
        }
    }
    return QMainWindow::eventFilter(watched, event);
}
std::optional<cv::Point> MainWindow::imagePixelAt(const QLabel *label, const cv::Mat &image, const QPointF &position)
{
    if (image.empty())
        return {};
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    auto pixmap = label->pixmap();
#else
    auto pixmap = label->pixmap(Qt::ReturnByValue);
#endif
    if (pixmap.isNull())
        return {};
    int margin = label->margin();
    auto content = label->contentsRect().adjusted(margin, margin, -margin, -margin);
    auto size = (QSizeF(pixmap.size()) / pixmap.devicePixelRatio()).toSize();
    auto displayed = QStyle::alignedRect(label->layoutDirection(), label->alignment(), size, content);
    // 右/下边界为开区间，避免把留白或像素边界映射到最后一列/行之外。
    if (displayed.width() <= 0 || displayed.height() <= 0 || position.x() < displayed.x() ||
        position.y() < displayed.y() || position.x() >= displayed.x() + displayed.width() ||
        position.y() >= displayed.y() + displayed.height())
        return {};
    int x = int(std::floor((position.x() - displayed.x()) * image.cols / displayed.width()));
    int y = int(std::floor((position.y() - displayed.y()) * image.rows / displayed.height()));
    return cv::Point(std::min(x, image.cols - 1), std::min(y, image.rows - 1));
}
double MainWindow::imageChannelValue(const cv::Mat &image, const cv::Point &pixel, int channel)
{
    if (image.empty() || pixel.x < 0 || pixel.y < 0 || pixel.x >= image.cols || pixel.y >= image.rows || channel < 0 ||
        channel >= image.channels())
        throw std::invalid_argument("像素坐标或图像通道无效");
    auto index = std::size_t(pixel.x) * image.channels() + channel;
    switch (image.depth())
    {
    case CV_8U:
        return image.ptr<uchar>(pixel.y)[index];
    case CV_8S:
        return image.ptr<signed char>(pixel.y)[index];
    case CV_16U:
        return image.ptr<uint16_t>(pixel.y)[index];
    case CV_16S:
        return image.ptr<int16_t>(pixel.y)[index];
    case CV_32S:
        return image.ptr<int32_t>(pixel.y)[index];
    case CV_32F:
        return image.ptr<float>(pixel.y)[index];
    case CV_64F:
        return image.ptr<double>(pixel.y)[index];
    default:
        throw std::invalid_argument("该图像类型不支持像素取值");
    }
}
void MainWindow::inspectImagePixel(bool depth, const QPointF &position)
{
    auto *label = depth ? depthView_ : imageView_;
    const auto &image = depth ? depth_ : image_;
    auto &selected = depth ? selectedDepthPixel_ : selectedImagePixel_;
    selected = imagePixelAt(label, image, position);
    if (!selected && !image.empty())
    {
        (depth ? ui_->depthPixelValue : ui_->imagePixelValue)
            ->setText(QStringLiteral("当前点击位于图像留白区域，无像素值。"));
        return;
    }
    updatePixelReadout(depth);
}
void MainWindow::updatePixelReadout(bool depth)
{
    const auto &image = depth ? depth_ : image_;
    auto &selected = depth ? selectedDepthPixel_ : selectedImagePixel_;
    auto *readout = depth ? ui_->depthPixelValue : ui_->imagePixelValue;
    if (image.empty() || !selected || selected->x >= image.cols || selected->y >= image.rows)
    {
        selected.reset();
        readout->setText(depth ? QStringLiteral("左键点击深度图，查看原始深度与换算后的 Z 值。")
                               : QStringLiteral("左键点击图像，查看像素坐标与灰度 / BGR 值。"));
        return;
    }
    try
    {
        QString text = QStringLiteral("像素 (X=%1, Y=%2)").arg(selected->x).arg(selected->y);
        int digits = image.depth() == CV_32F ? std::numeric_limits<float>::max_digits10
                                             : std::numeric_limits<double>::max_digits10;
        if (depth)
        {
            double raw = imageChannelValue(image, *selected, 0);
            double z = raw * numbers_.at("scale")->value() + numbers_.at("offset")->value();
            text += QStringLiteral(" · 原始深度=%1 · 换算深度 Z=%2（与点云同单位）")
                        .arg(pixelNumber(raw, digits), pixelNumber(z, 17));
            if (!std::isfinite(raw) || raw <= 0 || !std::isfinite(z) || z <= 0)
                text += QStringLiteral(" · 无效深度");
        }
        else
        {
            QStringList values;
            for (int channel = 0; channel < image.channels(); ++channel)
                values << pixelNumber(imageChannelValue(image, *selected, channel), digits);
            if (image.channels() == 1)
                text += QStringLiteral(" · 灰度=%1").arg(values.front());
            else
                text += QStringLiteral(" · %1=(%2)").arg(image.channels() == 4 ? "BGRA" : "BGR", values.join(", "));
        }
        readout->setText(text);
    }
    catch (const std::exception &e)
    {
        readout->setText(QStringLiteral("读取图像值失败：%1").arg(qstr(e.what())));
    }
}
void MainWindow::refreshImages()
{
    showImage(imageView_, image_);
    showImage(depthView_, depth_);
    showImage(resultView_, overlay_);
    updatePixelReadout(false);
    updatePixelReadout(true);
}
void MainWindow::loadSyntheticImages()
{
    if (watcher_.isRunning())
        return;
    depth_ = cv::Mat::zeros(240, 320, CV_32F);
    selectedImagePixel_.reset();
    selectedDepthPixel_.reset();
    image_ = cv::Mat::zeros(depth_.size(), CV_8UC3);
    for (int y = 45; y < 195; ++y)
        for (int x = 65; x < 255; ++x)
        {
            double z = 500 + 12 * std::sin(x * 0.04) + 8 * std::cos(y * 0.07) + 0.001 * (x - 100) * (y - 100);
            depth_.at<float>(y, x) = float(z);
            image_.at<cv::Vec3b>(y, x) = {uchar(x % 255), uchar(y % 255), 180};
        }
    numbers_.at("fx")->setValue(300);
    numbers_.at("fy")->setValue(300);
    numbers_.at("cx")->setValue(160);
    numbers_.at("cy")->setValue(120);
    numbers_.at("scale")->setValue(1);
    numbers_.at("offset")->setValue(0);
    result_ = {};
    overlay_.release();
    table_->setRowCount(0);
#ifdef SM_ENABLE_PCL_VIEWER
    cloudView_->setData({});
    ui_->cloudInfoLabel->setText(QStringLiteral("已生成合成图，可预览深度点云或运行匹配"));
#endif
    refreshImages();
    log(QStringLiteral("已生成非对称曲面合成数据。可先以当前深度创建模板，再运行匹配。"));
}
void MainWindow::loadModel(bool fromDepth)
{
    try
    {
        auto p = parameters();
        QString path;
        if (!fromDepth)
        {
            path = QFileDialog::getOpenFileName(this, QStringLiteral("打开 3D 模型"), {},
                                                "3D models (*.sfm *.om3 *.ply *.obj *.stl *.dxf);;All files (*)");
            if (path.isEmpty())
                return;
        }
        if (fromDepth && depth_.empty())
            throw std::invalid_argument("请先打开深度图");
        auto depth = depth_.clone();
        auto c = camera();
        auto b = bounds();
        double scale = numbers_.at("fileScale")->value();
        startTask(
            [fromDepth, path, depth, c, b, p, scale]()
            {
                TaskResult r;
                if (fromDepth)
                    r.model = sm::SurfaceMatch::initModel3D(sm::Tool::depthImgToObjectModel3D(depth, c, b), p);
                else if (path.endsWith(".sfm", Qt::CaseInsensitive))
                    r.model = sm::SurfaceMatch::initModel(path.toUtf8().toStdString(), p);
                else
                {
                    HTuple status;
                    HObjectModel3D object(HString(path.toUtf8().constData()), HTuple("m"), HTuple(), HTuple(), &status);
                    if (scale != 1)
                    {
                        HTuple h;
                        HomMat3dIdentity(&h);
                        HomMat3dScale(h, scale, scale, scale, 0, 0, 0, &h);
                        object = object.AffineTransObjectModel3d(HHomMat3D(h));
                    }
                    r.model = sm::SurfaceMatch::initModel3D(object, p);
                }
                r.message = QStringLiteral("模型就绪：%1 点")
                                .arg(qlonglong(r.model->object.GetObjectModel3dParams("num_points").L()));
                return r;
            });
    }
    catch (const HException &e)
    {
        log(QString::fromUtf8(e.ErrorMessage().Text()));
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
}
void MainWindow::runSmokeWorkflow()
{
    auto depth = depth_.clone();
    auto c = camera();
    auto b = bounds();
    auto p = parameters();
    p.numMatches = 1;
    resultCamera_ = c;
    startTask(
        [depth, c, b, p]()
        {
            TaskResult r;
            auto scene = sm::Tool::depthImgToObjectModel3D(depth, c, b);
            r.model = sm::SurfaceMatch::initModel3D(scene, p);
            r.match = sm::SurfaceMatch::runModel(*r.model, {}, depth, c, b, p);
#ifdef SM_ENABLE_PCL_VIEWER
            r.cloud = CloudView::prepareData(scene, &r.model->object, r.match->poses);
#endif
            r.message = QStringLiteral("界面自动测试完成：%1 个候选").arg(r.match->scores.Length());
            return r;
        });
}
bool MainWindow::isBusy() const
{
    return watcher_.isRunning();
}
bool MainWindow::hasMatches() const
{
    return result_.scores.Length() > 0 && table_->rowCount() > 0;
}
void MainWindow::runModel()
{
    try
    {
        if (!model_)
            throw std::invalid_argument("请先加载或创建模型");
        if (depth_.empty())
            throw std::invalid_argument("请先加载深度图");
        auto c = camera();
        auto b = bounds();
        auto p = parameters();
        auto d = detector();
        bool use2D = use2D_->isChecked();
        int attitude = attitude_->value();
        auto image = image_.clone(), depth = depth_.clone();
        auto model = model_;
        resultCamera_ = c;
        result_ = {};
        overlay_.release();
        table_->setRowCount(0);
        graspText_->setText(QStringLiteral("正在匹配…"));
        refreshImages();
        startTask(
            [model, image, depth, c, b, p, d, use2D, attitude]()
            {
                TaskResult r;
                r.match = sm::SurfaceMatch::runModel(*model, image, depth, c, b, p, use2D ? &d : nullptr, attitude);
#ifdef SM_ENABLE_PCL_VIEWER
                // 全场景仅用于三维背景显示，显示抽样不改变已完成的匹配结果。
                auto scene = sm::Tool::depthImgToObjectModel3D(depth, c, b);
                r.cloud = CloudView::prepareData(scene, &model->object, r.match->poses);
#endif
                r.message = QStringLiteral("runModel 完成：%1 个候选，耗时 %2 ms")
                                .arg(r.match->scores.Length())
                                .arg(r.match->elapsedMs, 0, 'f', 1);
                return r;
            });
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
}
void MainWindow::filterDepth()
{
    try
    {
        if (depth_.empty())
            throw std::invalid_argument("请先打开深度图");
        auto depth = depth_.clone();
        auto c = camera();
        auto path = depthRuntime_->text().toUtf8().toStdString();
        startTask(
            [depth, c, path]()
            {
                TaskResult r;
                r.depth = sm::NativeBackend::DepthRadiusFilter(depth, c, path, 5, 4, 1);
                r.message = QStringLiteral("原生深度滤波完成，已替换当前深度图（内存中）");
                return r;
            });
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
}
void MainWindow::startTask(std::function<TaskResult()> job)
{
    if (watcher_.isRunning())
    {
        log(QStringLiteral("请等待当前任务完成"));
        return;
    }
    controls_->setEnabled(false);
    log(QStringLiteral("正在后台执行算法…"));
    watcher_.setFuture(QtConcurrent::run(
        [job = std::move(job)]()
        {
            try
            {
                return job();
            }
            catch (const HException &e)
            {
                TaskResult r;
                r.error =
                    QStringLiteral("HALCON %1：%2").arg(e.ErrorCode()).arg(QString::fromUtf8(e.ErrorMessage().Text()));
                return r;
            }
            catch (const std::exception &e)
            {
                TaskResult r;
                r.error = qstr(e.what());
                return r;
            }
            catch (int code)
            {
                TaskResult r;
                r.error = QStringLiteral("原生 SDK 异常：%1").arg(code);
                return r;
            }
        }));
}
void MainWindow::finishTask()
{
    controls_->setEnabled(true);
    auto result = watcher_.result();
    if (!result.error.isEmpty())
    {
        log(result.error);
        graspText_->setText(QStringLiteral("任务失败，请查看日志"));
        return;
    }
    if (result.model)
    {
        model_ = result.model;
        modelLabel_->setText(result.message);
        result_ = {};
        showResult();
#ifdef SM_ENABLE_PCL_VIEWER
        cloudView_->setData({});
#endif
    }
    if (!result.depth.empty())
    {
        depth_ = result.depth;
        result_ = {};
        showResult();
#ifdef SM_ENABLE_PCL_VIEWER
        cloudView_->setData({});
#endif
    }
    if (result.match)
    {
        result_ = std::move(*result.match);
        showResult();
    }
#ifdef SM_ENABLE_PCL_VIEWER
    if (result.cloud)
    {
        cloudView_->setData(result.cloud);
        cloudModelPreview_ = result.cloudIsModelPreview;
        selectCloudCandidate();
        ui_->cloudInfoLabel->setText(result.cloudDescription.isEmpty()
                                         ? QStringLiteral("显示场景 %1 点 · 模板 %2 点 · 匹配 %3 个（仅显示采样）")
                                               .arg(qulonglong(result.cloud->scenePoints.size()))
                                               .arg(qulonglong(result.cloud->modelPoints.size()))
                                               .arg(qulonglong(result.cloud->modelPoses.size()))
                                         : result.cloudDescription);
        ui_->viewTabs->setCurrentWidget(ui_->cloudPage);
    }
#endif
    log(result.message);
    refreshImages();
}
void MainWindow::previewPointCloud()
{
#ifdef SM_ENABLE_PCL_VIEWER
    try
    {
        if (depth_.empty())
            throw std::invalid_argument("请先打开深度图");
        auto depth = depth_.clone();
        auto c = camera();
        auto b = bounds();
        startTask(
            [depth, c, b]()
            {
                TaskResult r;
                auto scene = sm::Tool::depthImgToObjectModel3D(depth, c, b);
                r.cloud = CloudView::prepareData(scene);
                r.match = sm::MatchResult{};
                r.message = QStringLiteral("已按当前内参与范围生成深度点云");
                return r;
            });
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
#else
    log(QStringLiteral("当前构建未启用 PCL 点云视图"));
#endif
}
void MainWindow::savePointCloud()
{
    try
    {
        if (depth_.empty())
            throw std::invalid_argument("请先打开深度图");
        auto c = camera();
        auto b = bounds();
        QFileDialog dialog(this, QStringLiteral("保存当前深度的 3D 点云"));
        dialog.setAcceptMode(QFileDialog::AcceptSave);
        dialog.setNameFilter("PLY (*.ply)");
        dialog.setDefaultSuffix("ply");
        dialog.setFileMode(QFileDialog::AnyFile);
        if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty())
            return;
        auto path = dialog.selectedFiles().first();
        if (!path.endsWith(".ply", Qt::CaseInsensitive))
            throw std::invalid_argument("点云文件须使用 .ply 扩展名");
        auto depth = depth_.clone();
        startTask(
            [depth, c, b, path]()
            {
                TaskResult r;
                auto scene = sm::Tool::depthImgToObjectModel3D(depth, c, b);
                auto count = scene.GetObjectModel3dParams("num_points").L();
                if (count == 0)
                    throw std::invalid_argument("当前内参和范围内没有有效点，未保存点云");
                scene.WriteObjectModel3d(HString("ply"), HString(path.toStdWString().c_str()), HTuple(), HTuple());
                r.message = QStringLiteral("已保存完整深度点云：%1 点，文件：%2").arg(qlonglong(count)).arg(path);
                return r;
            });
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
}
void MainWindow::showModel()
{
#ifdef SM_ENABLE_PCL_VIEWER
    if (!model_)
    {
        log(QStringLiteral("请先加载 3D 模型"));
        return;
    }
    auto model = model_;
    startTask(
        [model]()
        {
            TaskResult r;
            HTuple identity;
            CreatePose(0, 0, 0, 0, 0, 0, "Rp+T", "abg", "point", &identity);
            r.cloud = CloudView::prepareData(HObjectModel3D(), &model->object, identity);
            r.cloudIsModelPreview = true;
            r.cloudDescription = QStringLiteral("模型预览：%1 点（模板坐标系，仅显示采样）")
                                     .arg(qulonglong(r.cloud->modelPoints.size()));
            r.message = QStringLiteral("已在三维视图显示已加载模型，坐标保持为模板坐标系");
            return r;
        });
#else
    log(QStringLiteral("当前构建未启用 PCL 点云视图"));
#endif
}
void MainWindow::selectCloudCandidate()
{
#ifdef SM_ENABLE_PCL_VIEWER
    cloudView_->setSelectedCandidate(cloudModelPreview_ ? -1 : table_->currentRow());
#endif
}
void MainWindow::saveCloudScreenshot()
{
#ifdef SM_ENABLE_PCL_VIEWER
    auto path = QFileDialog::getSaveFileName(this, QStringLiteral("保存 3D 点云视图"), {}, "PNG (*.png)");
    if (path.isEmpty())
        return;
    log(cloudView_->saveScreenshot(path) ? QStringLiteral("3D 视图截图已保存")
                                         : QStringLiteral("截图失败：请先打开 3D 页并检查输出路径"));
#endif
}
void MainWindow::showResult()
{
    table_->setRowCount(int(result_.scores.Length()));
    overlay_ = image_.empty() ? cv::Mat(depth_.size(), CV_8UC3, cv::Scalar(25, 38, 52)) : image_.clone();
    for (const auto &d : result_.detections)
        sm::Tool::DrawRotatedRect(overlay_, d.box, d.radians, {0, 190, 255});
    try
    {
        for (Hlong i = 0; i < result_.scores.Length(); ++i)
        {
            for (int j = 0; j < 8; ++j)
            {
                double value = j == 0   ? result_.scores[i].D()
                               : j == 7 ? result_.radians2D.at(size_t(i))
                                        : result_.poses[i * 7 + j - 1].D();
                table_->setItem(int(i), j, new QTableWidgetItem(QString::number(value, 'f', 4)));
            }
            auto moved =
                model_->object.RigidTransObjectModel3d(HPose(result_.poses.TupleSelectRange(i * 7, i * 7 + 6)));
            auto x = moved.GetObjectModel3dParams("point_coord_x"), y = moved.GetObjectModel3dParams("point_coord_y"),
                 z = moved.GetObjectModel3dParams("point_coord_z");
            Hlong step = std::max<Hlong>(1, z.Length() / 12000);
            for (Hlong j = 0; j < z.Length(); j += step)
            {
                double zz = z[j].D();
                if (!std::isfinite(zz) || zz <= 0)
                    continue;
                double u = x[j].D() * resultCamera_.fx / zz + resultCamera_.cx,
                       v = y[j].D() * resultCamera_.fy / zz + resultCamera_.cy;
                if (std::isfinite(u) && std::isfinite(v) && u >= 0 && v >= 0 && u < overlay_.cols && v < overlay_.rows)
                    cv::circle(overlay_, {int(u), int(v)}, 1, cv::Scalar(80, 255, 70), -1);
            }
        }
    }
    catch (const HException &e)
    {
        log(QString::fromUtf8(e.ErrorMessage().Text()));
    }
    if (result_.scores.Length())
    {
        table_->selectRow(0);
        graspText_->setText(QStringLiteral("匹配成功，请在抓手偏移页核对标定参数后转换。"));
    }
    else
        graspText_->setText(QStringLiteral("当前没有匹配候选"));
    refreshImages();
}
HTuple MainWindow::parsePose(const QLineEdit *edit)
{
    auto v = values(edit->text(), 6);
    HTuple p;
    CreatePose(v[0], v[1], v[2], v[3], v[4], v[5], "Rp+T", "abg", "point", &p);
    return p;
}
HTuple MainWindow::graspOffset(const HTuple &matchPose) const
{
    auto handEye = parsePose(handEye_);
    HTuple robot;
    if (eyeInHand_->isChecked())
        robot = parsePose(robot_);
    // 固定相机不解析机器人文本，等同于接口传 nullptr；旧示教字段仅为保留已有配置，不参与新接口。
    return sm::SurfaceMatch::transformPose(matchPose, eyeInHand_->isChecked(), handEye,
                                           eyeInHand_->isChecked() ? &robot : nullptr);
}
void MainWindow::transformSelected()
{
    try
    {
        int row = table_->currentRow();
        if (row < 0 || row >= result_.scores.Length())
            throw std::invalid_argument("请先选择匹配候选");
        auto offset = graspOffset(result_.poses.TupleSelectRange(row * 7, row * 7 + 6));
        graspText_->setText(
            QStringLiteral("抓手相对位移 Pose（基座系）：\n%1\n\n新 TCP = 相对变换 × 参考 TCP").arg(poseText(offset)));
        log(QStringLiteral("候选 %1 已转换为抓手相对位移：%2").arg(row).arg(poseText(offset)));
    }
    catch (const HException &e)
    {
        log(QString::fromUtf8(e.ErrorMessage().Text()));
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
}
QJsonObject MainWindow::settingsToJson() const
{
    QJsonObject root;
    root["version"] = 1;
    for (auto pair : numbers_)
        root[pair.first] = pair.second->value();
    const std::map<QString, QLineEdit *> texts{
        {"model2D", model2D_},     {"runtime", runtime_},     {"depthRuntime", depthRuntime_},
        {"handEye", handEye_},     {"robot", robot_},         {"taughtModel", taughtModel_},
        {"taughtTcp", taughtTcp_}, {"reference", reference_}, {"center", center_}};
    const std::map<QString, QCheckBox *> checks{{"use2D", use2D_},
                                                {"gpu", gpu_},
                                                {"yolo26", yolo26_},
                                                {"eyeInHand", eyeInHand_},
                                                {"symmetry", symmetry_},
                                                {"sceneVisible", ui_->sceneVisibleCheck},
                                                {"modelsVisible", ui_->matchesVisibleCheck}};
    const std::map<QString, QComboBox *> combos{
        {"accuracy", accuracy_}, {"driver", driver_}, {"kind", kind_}, {"offsetMode", offsetMode_}};
    for (auto pair : texts)
        root[pair.first] = pair.second->text();
    for (auto pair : checks)
        root[pair.first] = pair.second->isChecked();
    for (auto pair : combos)
        root[pair.first] = pair.second->currentIndex();
    root["attitude"] = attitude_->value();
    root["pointSize"] = ui_->pointSizeSpin->value();
    return root;
}
void MainWindow::applySettings(const QJsonObject &root)
{
    if (root["version"].toInt() != 1)
        throw std::invalid_argument("配置 JSON 版本无效");
    const std::map<QString, QLineEdit *> texts{
        {"model2D", model2D_},     {"runtime", runtime_},     {"depthRuntime", depthRuntime_},
        {"handEye", handEye_},     {"robot", robot_},         {"taughtModel", taughtModel_},
        {"taughtTcp", taughtTcp_}, {"reference", reference_}, {"center", center_}};
    const std::map<QString, QCheckBox *> checks{{"use2D", use2D_},
                                                {"gpu", gpu_},
                                                {"yolo26", yolo26_},
                                                {"eyeInHand", eyeInHand_},
                                                {"symmetry", symmetry_},
                                                {"sceneVisible", ui_->sceneVisibleCheck},
                                                {"modelsVisible", ui_->matchesVisibleCheck}};
    const std::map<QString, QComboBox *> combos{
        {"accuracy", accuracy_}, {"driver", driver_}, {"kind", kind_}, {"offsetMode", offsetMode_}};
    for (auto pair : numbers_)
        if (root.contains(pair.first))
        {
            auto v = root[pair.first];
            if (!v.isDouble() || !std::isfinite(v.toDouble()) || v.toDouble() < pair.second->minimum() ||
                v.toDouble() > pair.second->maximum())
                throw std::invalid_argument("配置数值超出范围：" + pair.first.toStdString());
        }
    for (auto pair : texts)
        if (root.contains(pair.first) && !root[pair.first].isString())
            throw std::invalid_argument("配置文本类型无效");
    for (auto pair : checks)
        if (root.contains(pair.first) && !root[pair.first].isBool())
            throw std::invalid_argument("配置开关类型无效");
    for (auto pair : combos)
        if (root.contains(pair.first))
        {
            auto v = root[pair.first];
            if (!v.isDouble() || !std::isfinite(v.toDouble()) || std::floor(v.toDouble()) != v.toDouble() ||
                v.toDouble() < 0 || v.toDouble() >= pair.second->count())
                throw std::invalid_argument("配置选择项无效");
        }
    const std::map<QString, QSpinBox *> integers{{"attitude", attitude_}, {"pointSize", ui_->pointSizeSpin}};
    for (auto pair : integers)
        if (root.contains(pair.first))
        {
            auto v = root[pair.first];
            if (!v.isDouble() || !std::isfinite(v.toDouble()) || std::floor(v.toDouble()) != v.toDouble() ||
                v.toDouble() < pair.second->minimum() || v.toDouble() > pair.second->maximum())
                throw std::invalid_argument("配置整数超出范围");
        }
    QScopedValueRollback<bool> restoring(restoringSettings_, true);
    for (auto pair : numbers_)
        if (root.contains(pair.first))
            pair.second->setValue(root[pair.first].toDouble());
    for (auto pair : texts)
        if (root.contains(pair.first))
            pair.second->setText(root[pair.first].toString());
    for (auto pair : checks)
        if (root.contains(pair.first))
            pair.second->setChecked(root[pair.first].toBool());
    for (auto pair : combos)
        if (root.contains(pair.first))
            pair.second->setCurrentIndex(root[pair.first].toInt());
    for (auto pair : integers)
        if (root.contains(pair.first))
            pair.second->setValue(root[pair.first].toInt());
}
void MainWindow::restoreAutomaticSettings()
{
    QFile file(settingsPath_);
    if (!file.exists())
        return;
    if (!file.open(QIODevice::ReadOnly))
    {
        log(QStringLiteral("自动恢复参数失败：%1").arg(file.errorString()));
        return;
    }
    QJsonParseError error;
    auto doc = QJsonDocument::fromJson(file.readAll(), &error);
    if (error.error != QJsonParseError::NoError || !doc.isObject())
    {
        log(QStringLiteral("参数文件损坏，已保留原文件：%1").arg(settingsPath_));
        return;
    }
    try
    {
        applySettings(doc.object());
        log(QStringLiteral("已恢复上次修改的参数"));
    }
    catch (const std::exception &e)
    {
        log(QStringLiteral("自动恢复参数失败：%1").arg(qstr(e.what())));
    }
}
void MainWindow::scheduleSettingsSave()
{
    if (!settingsEnabled_ || restoringSettings_)
        return;
    settingsDirty_ = true;
    settingsSaveTimer_->start();
}
void MainWindow::saveAutomaticSettings()
{
    if (!settingsEnabled_ || !settingsDirty_)
        return;
    settingsSaveTimer_->stop();
    QSaveFile file(settingsPath_);
    auto bytes = QJsonDocument(settingsToJson()).toJson();
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
    {
        log(QStringLiteral("自动保存参数失败：%1；路径：%2").arg(file.errorString(), settingsPath_));
        return;
    }
    settingsDirty_ = false;
}
void MainWindow::settingsFile(bool save)
{
    auto path = save ? QFileDialog::getSaveFileName(this, QStringLiteral("导出配置"), {}, "JSON (*.json)")
                     : QFileDialog::getOpenFileName(this, QStringLiteral("导入配置"), {}, "JSON (*.json)");
    if (path.isEmpty())
        return;
    try
    {
        if (save)
        {
            QSaveFile file(path);
            auto bytes = QJsonDocument(settingsToJson()).toJson();
            if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
                throw std::runtime_error("无法写入配置文件");
        }
        else
        {
            QFile file(path);
            if (!file.open(QIODevice::ReadOnly))
                throw std::runtime_error("无法读取配置文件");
            QJsonParseError error;
            auto doc = QJsonDocument::fromJson(file.readAll(), &error);
            if (error.error != QJsonParseError::NoError || !doc.isObject())
                throw std::runtime_error("配置 JSON 格式无效");
            applySettings(doc.object());
            scheduleSettingsSave();
        }
        log(save ? QStringLiteral("配置已导出") : QStringLiteral("配置已导入并加入自动记录"));
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
}
void MainWindow::exportResults()
{
    try
    {
        if (!result_.scores.Length())
            throw std::invalid_argument("没有可导出的结果");
        auto path = QFileDialog::getSaveFileName(this, QStringLiteral("导出结果"), {}, "JSON (*.json)");
        if (path.isEmpty())
            return;
        QJsonArray list;
        for (Hlong i = 0; i < result_.scores.Length(); ++i)
        {
            auto pose = result_.poses.TupleSelectRange(i * 7, i * 7 + 6);
            auto offset = graspOffset(pose);
            QJsonObject item;
            item["score"] = result_.scores[i].D();
            item["modelPoseInCamera"] = tupleJson(pose);
            item["outputPose"] = tupleJson(offset);
            list.append(item);
        }
        QJsonObject root;
        root["elapsedMs"] = result_.elapsedMs;
        root["outputPoseConvention"] = "base_relative_left_multiply";
        root["candidates"] = list;
        QSaveFile file(path);
        if (!file.open(QIODevice::WriteOnly) || file.write(QJsonDocument(root).toJson()) < 0 || !file.commit())
            throw std::runtime_error("无法保存结果 JSON");
        log(QStringLiteral("匹配与抓手结果已导出"));
    }
    catch (const HException &e)
    {
        log(QString::fromUtf8(e.ErrorMessage().Text()));
    }
    catch (const std::exception &e)
    {
        log(qstr(e.what()));
    }
}
void MainWindow::log(const QString &message)
{
    log_->appendPlainText(QDateTime::currentDateTime().toString("HH:mm:ss ") + message);
}
