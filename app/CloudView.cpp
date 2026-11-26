#include "CloudView.h"
#include <QApplication>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QResizeEvent>
#include <QPainter>
#include <QImage>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <vtkWin32OpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkWindowToImageFilter.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
/** @brief 从 HALCON 点云提取显示点，避免把无效坐标送入 GPU。
 * @details 零点模型直接返回空数组，不查询该模型不存在的 XYZ 属性，支持只预览模板的场景。
 * @param object 只读点云句柄，可以为零点模型。
 * @param limit 正整数显示上限。
 * @return 有限 float XYZ 点，最多 limit 项；不修改原点云。 */
std::vector<cv::Vec3f> extractPoints(const HalconCpp::HObjectModel3D &object, std::size_t limit)
{
    if (object.GetObjectModel3dParams("num_points").L() == 0)
        return {};
    auto x = object.GetObjectModel3dParams("point_coord_x"), y = object.GetObjectModel3dParams("point_coord_y"),
         z = object.GetObjectModel3dParams("point_coord_z");
    std::size_t count = std::size_t(x.Length());
    if (y.Length() != x.Length() || z.Length() != x.Length())
        throw std::runtime_error("点云 XYZ 数量不一致");
    std::size_t step = std::max<std::size_t>(1, count / limit + (count % limit != 0));
    std::vector<cv::Vec3f> points;
    points.reserve(std::min(count, limit));
    for (std::size_t i = 0; i < count; i += step)
    {
        double px = x[Hlong(i)].D(), py = y[Hlong(i)].D(), pz = z[Hlong(i)].D();
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
            std::abs(px) > std::numeric_limits<float>::max() || std::abs(py) > std::numeric_limits<float>::max() ||
            std::abs(pz) > std::numeric_limits<float>::max())
            continue;
        points.emplace_back(float(px), float(py), float(pz));
    }
    return points;
}
/** @brief 将 CPU 快照复制到 PCL 显示点云，数据所有权与 HALCON 解耦。
 * @param points 有限 XYZ 数组，允许为空。
 * @return 独立 PCL 点云共享指针，仅在 UI 线程渲染。 */
pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(const std::vector<cv::Vec3f> &points)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(points.size());
    for (auto p : points)
        cloud->push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    cloud->is_dense = true;
    return cloud;
}
}

/** @brief UI 线程独占的 PCL/VTK 渲染状态。
 * @details data 持有不可变 CPU 快照，scene/model 持有用于 actor 的几何；viewer 引用 window/renderer，
 * interactor 仅处理从 Qt 转发的事件，不接管 HWND 或启动独立消息循环。
 * native 表示 Qt 使用 Windows 平台插件，offscreen 时保持纯 Qt 占位绘制。 */
struct CloudView::Impl
{
    std::shared_ptr<const CloudSceneData> data;                   ///< 最近收到的不可变快照。
    vtkSmartPointer<vtkWin32OpenGLRenderWindow> window;           ///< 借用 QWidget HWND 的 OpenGL 窗口。
    vtkSmartPointer<vtkRenderer> renderer;                        ///< 当前场景渲染器。
    vtkSmartPointer<vtkGenericRenderWindowInteractor> interactor; ///< Qt 事件适配的 VTK 交互器。
    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer;    ///< UI 线程独占的 PCL 可视化器。
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene, model;             ///< PCL 几何数据，多个模板 actor 共用 model。
    bool native = QApplication::platformName() == "windows";      ///< 仅 Windows 原生平台建立 OpenGL 上下文。
    bool sceneVisible = true;                                     ///< 是否显示背景场景。
    bool modelsVisible = true;                                    ///< 是否显示匹配模型。
    int selected = -1;                                            ///< 高亮候选，-1 表示未选中。
    int pointSize = 2;                                            ///< 场景点像素大小，范围 [1,8]。
    int wheelRemainder = 0;                                       ///< 尚未构成一个 120 单位滚轮刻度的余量。
};

CloudView::CloudView(QWidget *parent) : QWidget(parent), impl_(std::make_unique<Impl>())
{
    setMinimumSize(360, 240);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    if (impl_->native)
    {
        setAttribute(Qt::WA_NativeWindow);
        setAttribute(Qt::WA_PaintOnScreen);
        setAttribute(Qt::WA_NoSystemBackground);
        setAutoFillBackground(false);
    }
}
CloudView::~CloudView()
{
    if (impl_->interactor)
    {
        impl_->interactor->Disable();
        impl_->interactor->SetRenderWindow(nullptr);
    }
    impl_->viewer.reset();
    if (impl_->window)
        impl_->window->Finalize();
    impl_.reset();
}
std::shared_ptr<const CloudSceneData> CloudView::prepareData(const HalconCpp::HObjectModel3D &scene,
                                                             const HalconCpp::HObjectModel3D *model,
                                                             const HalconCpp::HTuple &poses, std::size_t maxScenePoints,
                                                             std::size_t maxModelPoints)
{
    if (!maxScenePoints || !maxModelPoints || poses.Length() % 7 || (!model && poses.Length()))
        throw std::invalid_argument("点云显示数量或候选位姿配置无效");
    auto data = std::make_shared<CloudSceneData>();
    data->scenePoints = extractPoints(scene, maxScenePoints);
    if (model)
        data->modelPoints = extractPoints(*model, maxModelPoints);
    for (Hlong i = 0; i < poses.Length() / 7; ++i)
    {
        HalconCpp::HTuple h;
        HalconCpp::PoseToHomMat3d(poses.TupleSelectRange(i * 7, i * 7 + 6), &h);
        std::array<double, 16> matrix{};
        for (int j = 0; j < 12; ++j)
        {
            matrix[j] = h[j].D();
            if (!std::isfinite(matrix[j]))
                throw std::invalid_argument("匹配矩阵含非有限值");
        }
        matrix[15] = 1;
        data->modelPoses.push_back(matrix);
    }
    return data;
}
void CloudView::setData(std::shared_ptr<const CloudSceneData> data)
{
    impl_->data = std::move(data);
    impl_->selected = -1;
    if (impl_->viewer)
    {
        rebuildActors();
        resetCamera();
    }
    update();
}
void CloudView::initializeRenderer()
{
    if (!impl_->native || impl_->viewer)
        return;
    impl_->window = vtkSmartPointer<vtkWin32OpenGLRenderWindow>::New();
    impl_->window->SetWindowId(reinterpret_cast<void *>(winId()));
    impl_->window->SetMultiSamples(0);
    impl_->window->SetSize(std::max(1, int(width() * devicePixelRatioF())),
                           std::max(1, int(height() * devicePixelRatioF())));
    impl_->renderer = vtkSmartPointer<vtkRenderer>::New();
    impl_->viewer =
        std::make_unique<pcl::visualization::PCLVisualizer>(impl_->renderer, impl_->window, "SurfaceMatch 3D", false);
    impl_->viewer->setBackgroundColor(0.055, 0.085, 0.13);
    impl_->viewer->setShowFPS(false);
    impl_->interactor = vtkSmartPointer<vtkGenericRenderWindowInteractor>::New();
    impl_->viewer->setupInteractor(impl_->interactor, impl_->window);
    auto style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    impl_->interactor->SetInteractorStyle(style);
    impl_->interactor->Initialize();
    impl_->interactor->Enable();
    rebuildActors();
    resetCamera();
}
void CloudView::rebuildActors()
{
    auto &s = *impl_;
    if (!s.viewer)
        return;
    s.viewer->removeAllPointClouds();
    s.scene.reset();
    s.model.reset();
    if (!s.data)
        return;
    s.scene = pclCloud(s.data->scenePoints);
    s.model = pclCloud(s.data->modelPoints);
    if (!s.scene->empty())
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(s.scene, 140, 155, 175);
        s.viewer->addPointCloud(s.scene, color, "scene");
    }
    if (!s.model->empty())
        for (std::size_t i = 0; i < s.data->modelPoses.size(); ++i)
        {
            auto id = "match_" + std::to_string(i);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(s.model, 70, 240, 130);
            s.viewer->addPointCloud(s.model, color, id);
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    transform.matrix()(r, c) = float(s.data->modelPoses[i][r * 4 + c]);
            s.viewer->updatePointCloudPose(id, transform);
        }
    updateAppearance();
}
void CloudView::updateAppearance()
{
    auto &s = *impl_;
    if (!s.viewer)
        return;
    if (s.scene && !s.scene->empty())
    {
        s.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, s.pointSize, "scene");
        s.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                   s.sceneVisible ? 1.0 : 0.0, "scene");
    }
    if (s.data && s.model && !s.model->empty())
        for (std::size_t i = 0; i < s.data->modelPoses.size(); ++i)
        {
            auto id = "match_" + std::to_string(i);
            bool selected = int(i) == s.selected;
            s.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, selected ? 1.0 : 0.27,
                                                       selected ? 0.65 : 0.94, selected ? 0.1 : 0.5, id);
            s.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                       s.pointSize + (selected ? 3 : 1), id);
            s.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       s.modelsVisible ? 1.0 : 0.0, id);
        }
    update();
}
void CloudView::setSelectedCandidate(int index)
{
    impl_->selected = impl_->data && index >= 0 && std::size_t(index) < impl_->data->modelPoses.size() ? index : -1;
    updateAppearance();
}
void CloudView::setSceneVisible(bool visible)
{
    impl_->sceneVisible = visible;
    updateAppearance();
}
void CloudView::setModelsVisible(bool visible)
{
    impl_->modelsVisible = visible;
    updateAppearance();
}
void CloudView::setPointSize(int pixels)
{
    impl_->pointSize = std::clamp(pixels, 1, 8);
    updateAppearance();
}
void CloudView::resetCamera()
{
    auto &s = *impl_;
    if (!s.viewer)
        return;
    double b[6];
    s.renderer->ComputeVisiblePropBounds(b);
    if (b[0] > b[1])
        return;
    double x = (b[0] + b[1]) / 2, y = (b[2] + b[3]) / 2, z = (b[4] + b[5]) / 2,
           d = std::max(1.0, std::hypot(std::hypot(b[1] - b[0], b[3] - b[2]), b[5] - b[4]));
    s.viewer->setCameraPosition(x + d, y - d, z - 1.2 * d, x, y, z, 0, -1, 0);
    s.renderer->ResetCamera();
    s.renderer->ResetCameraClippingRange();
    update();
}
bool CloudView::saveScreenshot(const QString &path)
{
    if (!impl_->window || path.isEmpty())
        return false;
    impl_->window->Render();
    auto capture = vtkSmartPointer<vtkWindowToImageFilter>::New();
    capture->SetInput(impl_->window);
    capture->SetInputBufferTypeToRGB();
    capture->ReadFrontBufferOff();
    capture->Update();
    auto *data = capture->GetOutput();
    int size[3];
    data->GetDimensions(size);
    if (size[0] <= 0 || size[1] <= 0)
        return false;
    QImage image(static_cast<uchar *>(data->GetScalarPointer()), size[0], size[1], size[0] * 3, QImage::Format_RGB888);
    return image.mirrored(false, true).save(path);
}
std::array<double, 9> CloudView::cameraState() const
{
    std::array<double, 9> out{};
    if (impl_->renderer)
    {
        auto *c = impl_->renderer->GetActiveCamera();
        c->GetPosition(out.data());
        c->GetFocalPoint(out.data() + 3);
        c->GetViewUp(out.data() + 6);
    }
    return out;
}
QPaintEngine *CloudView::paintEngine() const
{
    return impl_->native ? nullptr : QWidget::paintEngine();
}
void CloudView::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    initializeRenderer();
}
void CloudView::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    if (impl_->window)
    {
        int w = std::max(1, int(width() * devicePixelRatioF())), h = std::max(1, int(height() * devicePixelRatioF()));
        impl_->window->SetSize(w, h);
        impl_->interactor->UpdateSize(w, h);
        update();
    }
}
void CloudView::paintEvent(QPaintEvent *event)
{
    (void)event;
    if (impl_->window)
        impl_->window->Render();
    else if (!impl_->native)
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(14, 22, 34));
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter,
                         QStringLiteral("3D 点云需要 Windows 原生 OpenGL 窗口\n当前为无窗口测试平台"));
    }
}
void CloudView::setEventPosition(const QPointF &point, Qt::KeyboardModifiers modifiers)
{
    if (impl_->interactor)
        impl_->interactor->SetEventInformationFlipY(
            int(point.x() * devicePixelRatioF()), int(point.y() * devicePixelRatioF()),
            modifiers.testFlag(Qt::ControlModifier), modifiers.testFlag(Qt::ShiftModifier));
}
void CloudView::mousePressEvent(QMouseEvent *event)
{
    if (!impl_->interactor)
    {
        QWidget::mousePressEvent(event);
        return;
    }
    setFocus();
    setEventPosition(event->localPos(), event->modifiers());
    if (event->button() == Qt::LeftButton)
        impl_->interactor->LeftButtonPressEvent();
    else if (event->button() == Qt::MiddleButton)
        impl_->interactor->MiddleButtonPressEvent();
    else if (event->button() == Qt::RightButton)
        impl_->interactor->RightButtonPressEvent();
    else
    {
        event->ignore();
        return;
    }
    grabMouse();
    event->accept();
}
void CloudView::mouseReleaseEvent(QMouseEvent *event)
{
    if (!impl_->interactor)
    {
        QWidget::mouseReleaseEvent(event);
        return;
    }
    setEventPosition(event->localPos(), event->modifiers());
    if (event->button() == Qt::LeftButton)
        impl_->interactor->LeftButtonReleaseEvent();
    else if (event->button() == Qt::MiddleButton)
        impl_->interactor->MiddleButtonReleaseEvent();
    else if (event->button() == Qt::RightButton)
        impl_->interactor->RightButtonReleaseEvent();
    if (event->buttons() == Qt::NoButton)
        releaseMouse();
    event->accept();
    update();
}
void CloudView::mouseMoveEvent(QMouseEvent *event)
{
    if (!impl_->interactor)
    {
        QWidget::mouseMoveEvent(event);
        return;
    }
    setEventPosition(event->localPos(), event->modifiers());
    impl_->interactor->MouseMoveEvent();
    event->accept();
}
void CloudView::wheelEvent(QWheelEvent *event)
{
    if (!impl_->interactor)
    {
        QWidget::wheelEvent(event);
        return;
    }
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    setEventPosition(event->position(), event->modifiers());
#else
    setEventPosition(event->posF(), event->modifiers());
#endif
    impl_->wheelRemainder += event->angleDelta().y();
    while (impl_->wheelRemainder >= 120)
    {
        impl_->interactor->MouseWheelForwardEvent();
        impl_->wheelRemainder -= 120;
    }
    while (impl_->wheelRemainder <= -120)
    {
        impl_->interactor->MouseWheelBackwardEvent();
        impl_->wheelRemainder += 120;
    }
    event->accept();
    update();
}
