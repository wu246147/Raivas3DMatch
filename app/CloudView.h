#pragma once
#include "surface_match/Types.h"
#include <QWidget>
#include <array>
#include <memory>

class QMouseEvent;
class QWheelEvent;
class QShowEvent;
class QResizeEvent;
class QPaintEvent;
class QPaintEngine;

/** @brief 点云显示所需的不可变 CPU 快照，连接后台 HALCON 算法与 UI 线程中的 PCL 渲染。
 * @details 仅保存有限 XYZ 点和模型到相机矩阵，不包含 HALCON、PCL 或 VTK 句柄。
 * 场景与模板长度单位一致；modelPoses 每项按行主序保存 4x4 刚体变换。
 * 后台构造后通过 shared_ptr<const> 交给 UI 只读使用，不修改算法原始点云。 */
struct CloudSceneData
{
    std::vector<cv::Vec3f> scenePoints;             ///< 相机系场景点，最多保留调用方指定数量。
    std::vector<cv::Vec3f> modelPoints;             ///< 模板坐标系点，多候选共用同一份几何数据。
    std::vector<std::array<double, 16>> modelPoses; ///< 每个匹配的模型到相机变换，顺序与结果表一致。
};

/** @brief 嵌入 Qt 的 PCL/VTK 点云视图，支持旋转、缩放、平移和候选高亮。
 * @details 本机 VTK 未包含 GUISupportQt，使用现有 Qt 原生子窗口承载 VTK OpenGL；
 * Qt 鼠标事件通过 vtkGenericRenderWindowInteractor 转给 TrackballCamera。
 * 渲染器、交互器和 OpenGL 资源都只在创建它们的 UI 线程访问；后台仅调用 prepareData。
 * Qt offscreen 平台不创建 OpenGL 窗口，显示说明文字，供普通界面 smoke 测试使用。
 * 析构在 Qt 原生窗口销毁前释放 VTK 图形资源，PImpl 隔离 PCL/VTK 头文件。 */
class CloudView final : public QWidget
{
  public:
    /** @brief 创建空视图，首次显示时建立渲染器，不启动独立事件循环。
     * @param parent Qt 父控件，可为空；Qt 负责子窗口生命周期。 */
    explicit CloudView(QWidget *parent = nullptr);
    /** @brief 在 UI 线程中释放渲染器、交互器与当前 OpenGL 上下文。
     * @note 必须先于 Qt 原生窗口销毁，不能从后台线程调用。 */
    ~CloudView() override;
    /** @brief 在后台提取有限点并按固定步长限制显示数量，保留模板几何与每个候选变换。
     * @param scene 有效场景句柄，可为零点模型，只读借用。
     * @param model 可选模板点云指针，nullptr 表示只显示场景；仅要求调用期间有效。
     * @param poses 连续七元 HALCON pose，数量必须为七的倍数；无模板时须为空。
     * @param maxScenePoints 场景显示上限，正整数，默认 250000，不影响匹配精度。
     * @param maxModelPoints 模板显示上限，正整数，默认 80000，不影响匹配精度。
     * @return 独立持有数据的只读快照，可跨线程传递。
     * @throws std::invalid_argument 数量上限、pose 数量或矩阵数值无效；HALCON 异常原样传播。 */
    static std::shared_ptr<const CloudSceneData> prepareData(const HalconCpp::HObjectModel3D &scene,
                                                             const HalconCpp::HObjectModel3D *model = nullptr,
                                                             const HalconCpp::HTuple &poses = {},
                                                             std::size_t maxScenePoints = 250000,
                                                             std::size_t maxModelPoints = 80000);
    /** @brief 替换场景和匹配模型，首次加载后自动调整到能看全点云的视角。
     * @param data 不可变快照，nullptr 清空视图；本对象持有共享引用。
     * @note 仅 UI 线程调用；不改变输入坐标和匹配结果。 */
    void setData(std::shared_ptr<const CloudSceneData> data);
    /** @brief 高亮表格选中的候选，其他候选保持绿色。
     * @param index 从零开始的候选编号，-1 表示不高亮；越界视为 -1。
     * @note 仅 UI 线程修改显示颜色，保持当前相机视角。 */
    void setSelectedCandidate(int index);
    /** @brief 控制背景场景显示。
     * @param visible true 显示灰色场景，false 隐藏；不影响匹配模型。 */
    void setSceneVisible(bool visible);
    /** @brief 控制全部匹配模型显示。
     * @param visible true 显示，false 隐藏；不影响背景场景。 */
    void setModelsVisible(bool visible);
    /** @brief 调整显示点的大小，选中候选始终比背景更醒目。
     * @param pixels 点大小，像素，[1,8]，超出范围会被限制。 */
    void setPointSize(int pixels);
    /** @brief 恢复可看全场景和匹配模型的初始相机视角。
     * @note 空场景时不改变相机；仅 UI 线程操作。 */
    void resetCamera();
    /** @brief 保存当前真实 OpenGL 渲染结果，支持 Qt 可编码的图片格式和中文路径。
     * @param path 非空输出文件路径，通常以 .png 结尾；已有文件由调用方决定是否覆盖。
     * @return true 保存成功；没有原生渲染上下文、空路径或编码失败时 false。
     * @note 仅 UI 线程调用，有文件写入副作用；不通过 QWidget 截图获取原生 OpenGL 内容。 */
    bool saveScreenshot(const QString &path);
    /** @brief 读取当前相机状态，便于核对视角与验证鼠标交互。
     * @return 位置 XYZ、焦点 XYZ、上方向 XYZ，共九项；未初始化时全零。
     * @note 只允许 UI 线程读取。 */
    std::array<double, 9> cameraState() const;

  protected:
    /** @brief 原生 VTK 绘图期间禁用 Qt 光栅绘图引擎，避免擦除 OpenGL 画面。
     * @return 原生模式 nullptr；offscreen 模式使用 QWidget 引擎。 */
    QPaintEngine *paintEngine() const override;
    /** @brief 首次显示时初始化原生渲染资源，并应用之前收到的数据。
     * @param event Qt 显示事件，不转移所有权。 */
    void showEvent(QShowEvent *event) override;
    /** @brief 按设备像素比同步 VTK 视口尺寸，保持旋转和拾取坐标一致。
     * @param event Qt 尺寸事件，不转移所有权。 */
    void resizeEvent(QResizeEvent *event) override;
    /** @brief 重绘点云；无原生上下文时绘制不可交互原因说明。
     * @param event Qt 重绘事件，不转移所有权。 */
    void paintEvent(QPaintEvent *event) override;
    /** @brief 转发鼠标按下，开始左键旋转、中键平移或右键缩放，并捕获拖动。
     * @param event Qt 鼠标事件，位置采用控件局部逻辑像素。 */
    void mousePressEvent(QMouseEvent *event) override;
    /** @brief 转发拖动结束并释放鼠标捕获。
     * @param event Qt 鼠标释放事件，调用期间有效。 */
    void mouseReleaseEvent(QMouseEvent *event) override;
    /** @brief 转发当前坐标和修饰键，让 VTK 计算相机运动。
     * @param event Qt 鼠标移动事件，调用期间有效。 */
    void mouseMoveEvent(QMouseEvent *event) override;
    /** @brief 将滚轮增量转换为 VTK 前后缩放事件，保留高分辨率滚轮余量。
     * @param event Qt 滚轮事件，angleDelta 以 1/8 度表示。 */
    void wheelEvent(QWheelEvent *event) override;

  private:
    /** @brief 隐藏的 PCL/VTK 所有权与可变显示状态。
     * @details 定义在实现文件，由 unique_ptr 唯一持有，只允许 UI 线程访问。 */
    struct Impl;
    std::unique_ptr<Impl> impl_; ///< 渲染与交互资源，在控件析构体内提前释放。
    /** @brief 初始化现有 Qt HWND 中的 VTK 渲染窗口，不创建另一个顶层窗口。 */
    void initializeRenderer();
    /** @brief 根据快照重建场景和候选 actor，应用颜色、大小及可见性。 */
    void rebuildActors();
    /** @brief 只更新 actor 外观，不改变几何或相机视角。 */
    void updateAppearance();
    /** @brief 将 Qt 逻辑像素坐标与修饰键转换为 VTK 底部原点的设备像素。
     * @param point 控件内局部逻辑坐标。
     * @param modifiers 当前 Ctrl/Shift 修饰键。 */
    void setEventPosition(const QPointF &point, Qt::KeyboardModifiers modifiers);
};
