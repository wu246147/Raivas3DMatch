#pragma once
#include "surface_match/SurfaceMatch.h"
#include <QMainWindow>
#include <QFutureWatcher>
#include <functional>
#include <map>
#include <optional>
class QDoubleSpinBox;
class QSpinBox;
class QComboBox;
class QCheckBox;
class QLineEdit;
class QLabel;
class QPlainTextEdit;
class QTableWidget;
class QFormLayout;
class QPushButton;
class QCloseEvent;
class QTimer;
class QJsonObject;
class QEvent;
class CloudView;
struct CloudSceneData;
namespace Ui
{
class MainWindow;
}

/** @brief 后台任务的值结果，阻止后台线程直接修改界面状态。
 * @details 错误文本为空表示正常；model/match/depth/cloud 按任务种类选择填充。
 * cloudDescription 与 cloudIsModelPreview 区分模板预览和匹配显示，避免把单位变换的预览计为匹配。
 * OpenCV 数据由返回值独立持有，模型以只读 shared_ptr 共享，UI 线程接收后更新。 */
struct TaskResult
{
    QString error;                               ///< 捕获并保留的实际错误信息。
    QString message;                             ///< 正常任务描述。
    std::shared_ptr<const sm::ModelData> model;  ///< 新建或加载的只读模型，可为空。
    std::optional<sm::MatchResult> match;        ///< 本次匹配结果，可无值。
    cv::Mat depth;                               ///< 新深度图，仅滤波任务填充。
    std::shared_ptr<const CloudSceneData> cloud; ///< 后台准备的点云显示快照，可为空，UI 接收后渲染。
    QString cloudDescription;                    ///< 可选三维显示说明，为空时使用匹配场景统计。
    bool cloudIsModelPreview = false;            ///< true 表示单独预览模板，不与结果表的候选高亮联动。
};
/** @brief HALCON/OpenCV 静态库的 Qt 测试窗口。
 * @details GUI 线程持有图像、模型及最后结果；建模、滤波与匹配只读捕获快照到后台。
 * 同时只允许一个任务，关闭时等待任务完成，后台不捕获 this，避免悬空 UI。
 * 参数在程序目录中自动记录并于启动恢复，同时支持手动导入/导出 JSON；机器人操作仅计算，不发送命令。 */
class MainWindow final : public QMainWindow
{
  public:
    /** @brief 从 MainWindow.ui 建立 Designer 布局并绑定算法与图像取值事件，初始化 SDK 路径与可选 3D 容器。
     * @details 图像和模板通过外部文件加载，合成数据仍由独立的 smoke 测试入口使用。
     * 抓手页使用四参数相对变换，固定相机禁用机器人输入；隐藏旧示教字段但保留其配置值以兼容旧 JSON。
     * @param parent Qt 父窗口，可为空，由 Qt 管理子控件。
     * @param settingsDirectory 配置目录，空字符串使用程序所在目录；非空主要供隔离测试使用。 */
    explicit MainWindow(QWidget *parent = nullptr, const QString &settingsDirectory = {});
    /** @brief 等待后台任务结束，再释放窗口持有的模型与图像资源。
     * @note 析构须在 UI 线程调用，可能阻塞到 HALCON 同步算子结束。 */
    ~MainWindow() override;
    /** @brief 生成用于验证界面、图像显示和参数联动的合成图像，不读用户文件。
     * @details 设置配套相机内参，生成不对称表面的深度图及灰度图，清除旧的匹配和三维显示，不启动耗时匹配。 */
    void loadSyntheticImages();
    /** @brief 自动执行合成数据建模和匹配，验证后台任务到界面结果的完整路径。
     * @details 仅 --smoke 验证入口使用，后台返回模型和匹配结果，由常规 finishTask 接收。 */
    void runSmokeWorkflow();
    /** @brief 查询后台算法是否仍在运行，供 smoke 事件循环等待。
     * @return true 表示任务尚未结束；只允许 UI 线程读取。 */
    bool isBusy() const;
    /** @brief 查询界面是否已显示至少一个匹配候选。
     * @return true 表示存在匹配分数及对应表格行；只允许 UI 线程读取。 */
    bool hasMatches() const;

  protected:
    /** @brief 后台执行中延迟关闭，允许关闭时先落盘尚未保存的参数。
     * @param event Qt 关闭事件，后台忙时忽略并在日志说明。 */
    void closeEvent(QCloseEvent *event) override;
    /** @brief 处理图像控件的缩放与左键取值，不拦截其他控件事件。
     * @param watched 安装事件过滤器的 2D、深度或结果图 QLabel，不转移所有权。
     * @param event Qt 派发的事件，调用期间有效；只消费 2D/深度图的左键点击。
     * @return 已处理的取值点击返回 true，其余沿用父类结果。
     * @note 仅 UI 线程调用；尺寸变化时重绘显示副本，取值始终读取原始 Mat。 */
    bool eventFilter(QObject *watched, QEvent *event) override;

  private:
    /** @brief 从数值控件读取并校验相机内参快照。
     * @return 有效相机配置，错误抛标准异常。 */
    sm::CameraIntrinsics camera() const;
    /** @brief 读取并校验 XYZ 裁剪范围。
     * @return 当前裁剪配置。 */
    sm::Bounds3D bounds() const;
    /** @brief 读取匹配参数，先应用档位，再以显式控件值覆盖查找/训练配置。
     * @return 已校验的参数快照。 */
    sm::MatchParameters parameters() const;
    /** @brief 从模型文件和任务/驱动选择构建原生 SDK 配置。
     * @return 独立配置值，完整合法性在原生调用前检查。 */
    sm::DetectorOptions detector() const;
    /** @brief 选择并加载 2D 或原始深度图，成功时清除对应像素选择、旧匹配与三维显示，失败保留原图。
     * @param depth true 加载保留位宽的深度图，false 加载 BGR 图。 */
    void openImage(bool depth);
    /** @brief 更新三个图像显示区及当前选中像素值，深度仅做显示归一化，不改算法输入。
     * @details 结果图叠加检测框和匹配模型投影；图像尺寸变化后自动清除越界的像素选择。 */
    void refreshImages();
    /** @brief 在 QLabel 内容区域内等比例显示图像，保留边框和边距，QImage 复制后脱离 Mat 生命周期。
     * @param label Qt 管理的显示控件，非空。
     * @param image BGR 或单通道图，允许空图显示提示。 */
    static void showImage(QLabel *label, const cv::Mat &image);
    /** @brief 将控件点击映射为原图像素，考虑等比例缩放、居中留白、边距和 pixmap 设备像素比。
     * @param label 当前显示图像的 QLabel，非空，要求未启用 scaledContents。
     * @param image 对应的原始图像，允许为空。
     * @param position 控件局部逻辑像素坐标，可为小数。
     * @return 原图中从零开始的列/行；空图、无 pixmap 或点击留白时返回空值。
     * @note 只读 UI 状态与 Mat 尺寸，不复制图像、不采样显示插值颜色。 */
    static std::optional<cv::Point> imagePixelAt(const QLabel *label, const cv::Mat &image, const QPointF &position);
    /** @brief 读取原始图像一个通道的数值，统一为 double 用于显示。
     * @param image 非空 Mat，支持 OpenCV 常见整数及 float32/float64 类型，允许非连续 ROI。
     * @param pixel 图内有效的列/行坐标。
     * @param channel 通道编号，范围为 [0,image.channels())。
     * @return 原始像素值，不应用颜色转换、深度换算或归一化。
     * @throws std::invalid_argument 图像、通道、像素或类型不受支持。 */
    static double imageChannelValue(const cv::Mat &image, const cv::Point &pixel, int channel);
    /** @brief 记录一次有效像素选择并更新图下读数，留白点击清除选择并显示提示。
     * @param depth true 读取原始深度图，false 读取 2D 图。
     * @param position 图像控件局部逻辑像素坐标。
     * @note 只修改瞬时 UI 状态，不写入参数配置或图像数据。 */
    void inspectImagePixel(bool depth, const QPointF &position);
    /** @brief 从原始图像刷新当前像素读数，显示原始通道值及深度换算结果。
     * @param depth true 更新深度读数，false 更新 2D 灰度/BGR/BGRA 读数。
     * @details 深度使用当前比例和偏移，零、负值、NaN、Inf 或非正换算深度会标记无效。
     * 无选择或选择越界时显示操作提示；读取异常转为可见说明，不修改输入图像。 */
    void updatePixelReadout(bool depth);
    /** @brief 后台读取/训练模型，或使用当前深度图生成模板。
     * @param fromDepth true 使用当前深度图，false 弹出模型文件选择。 */
    void loadModel(bool fromDepth);
    /** @brief 抓取图像与配置快照后在后台调用 runModel，期间禁用操作区域。
     * @details 启用 PCL 时在后台额外准备有限显示点与模型矩阵，完成后交给 UI 渲染，不改变匹配点云。 */
    void runModel();
    /** @brief 按原 SDK 固定 5/4/1 规则过滤深度图，成功后替换当前深度图。
     * @note SDK 当前实现忽略导出函数中的可调门限，界面据实显示此限制。 */
    void filterDepth();
    /** @brief 调度一个后台任务，统一捕获异常并在 UI 线程接收结果。
     * @param job 不捕获窗口指针的同步任务，输入应已按值捕获，返回自有数据。
     * @note 同时仅一个任务；调用线程必须为 UI 线程。 */
    void startTask(std::function<TaskResult()> job);
    /** @brief 任务结束后更新模型、图像、结果和日志，将不可变显示快照交给 UI 线程中的 PCL 渲染器。
     * @details 不在后台访问任何渲染对象；新模型或深度会清除旧的三维显示，随后按新快照刷新。
     * 模板预览使用独立说明和显示状态，不修改已有候选，也不把预览单位变换计为匹配结果。 */
    void finishTask();
    /** @brief 以当前深度、内参与裁剪范围在后台准备点云，在 3D 页显示并清除过期匹配。
     * @note 不修改深度图数据；未启用 PCL 时只显示中文提示。 */
    void previewPointCloud();
    /** @brief 选择 PLY 输出文件后，在后台按当前深度、内参与 XYZ 范围生成并保存完整点云。
     * @details 保存相机坐标系的全部有效点，不使用显示抽样，不修改原始深度或已有匹配。
     * 未加载深度、参数无效或点云为空时记录错误；取消文件对话框不写文件。
     * @note 输出长度单位与深度换算一致；文件选择由 UI 线程执行，转换和写盘由后台任务持有快照执行。 */
    void savePointCloud();
    /** @brief 在后台准备已加载模型的显示快照，并在三维页以模板坐标系单独预览。
     * @details 使用单位变换显示模板，不叠加场景或匹配姿态，不修改模型和候选结果。
     * @note 未加载模型或未启用 PCL 时给出中文提示；PCL 渲染只在 UI 线程进行。 */
    void showModel();
    /** @brief 按表格当前行更新三维候选高亮，保留用户旋转后的相机视角。
     * @note 模板预览期间不高亮候选；仅修改渲染外观，未启用 PCL 时无副作用。 */
    void selectCloudCandidate();
    /** @brief 选择文件路径并保存原生三维视图截图，失败时保留实际错误提示。
     * @note 仅 UI 线程执行，取消对话框不写文件。 */
    void saveCloudScreenshot();
    /** @brief 将最后结果填入候选表并刷新图像投影，空结果清空旧表。 */
    void showResult();
    /** @brief 对表格选中候选调用四参数 transformPose，显示基座系抓手相对位移和左乘应用方式。
     * @details 未选候选或标定无效时保留实际错误到日志；不计算绝对 TCP，不发送机器人命令。
     * @note 只在 UI 线程读取控件、更新读数。 */
    void transformSelected();
    /** @brief 导入或导出相机、匹配与抓手测试配置 JSON，显式文件操作。
     * @param save true 写文件，false 读文件；取消对话框不改变状态。 */
    void settingsFile(bool save);
    /** @brief 将全部算法、标定和三维外观控件收集为同一版本的配置快照。
     * @return 含 version=1 的 JSON 对象，不包含图像像素和 HALCON 句柄。 */
    QJsonObject settingsToJson() const;
    /** @brief 先验证完整 JSON 字段，再一次性应用控件值，恢复期间不触发自动落盘。
     * @param root 版本为 1 的配置对象，缺失字段保持当前值，以兼容旧配置。
     * @throws std::invalid_argument 版本、类型、枚举或数值范围不合法；失败不修改控件。 */
    void applySettings(const QJsonObject &root);
    /** @brief 启动时从程序目录恢复参数；文件不存在时保持 Designer 默认值。
     * @note 读取失败记录原因并保留原文件，不因此自动覆盖损坏配置。 */
    void restoreAutomaticSettings();
    /** @brief 标记参数变更并重新启动 500 毫秒单次计时，合并连续编辑。
     * @note 恢复配置或普通 --smoke 测试期间忽略变更；仅 UI 线程使用。 */
    void scheduleSettingsSave();
    /** @brief 使用 QSaveFile 原子写入变更快照，成功后清除脏标记。
     * @note 写入程序目录；失败保留脏标记并记录实际文件错误，不切换到其他目录。 */
    void saveAutomaticSettings();
    /** @brief 导出匹配位姿、分数和基座系抓手相对位移到 JSON。
     * @details outputPose 保存四参数接口结果，明确记录基座系左乘约定，不再输出无法由四参数得到的绝对 TCP。
     * 每个候选按当前手眼与拍照位姿计算，配置错误停止导出；UI 线程选择文件，取消不写盘。 */
    void exportResults();
    /** @brief 根据六元数值文本创建 HALCON abg pose。
     * @param edit 包含 x,y,z,rx,ry,rz 的控件，允许逗号、空格或分号分隔。
     * @return 七元 pose，角度单位度。
     * @throws std::invalid_argument 元素数量错误或无法解析。 */
    static HalconCpp::HTuple parsePose(const QLineEdit *edit);
    /** @brief 按当前手眼标定和拍照位姿将选中相机系目标偏移转换为基座系抓手相对位移。
     * @param matchPose 单个七元相机系偏移，调用期间只读；模板坐标须满足四参数接口的参考要求。
     * @return 独立七元 abg 相对 pose，平移单位不变，用于左乘参考 TCP。
     * @throws std::invalid_argument 标定文本或眼在手上所需的机器人文本无效。
     * @note UI 线程调用；眼在手外不读取机器人文本，旧版示教/控制器配置不参与计算。 */
    HalconCpp::HTuple graspOffset(const HalconCpp::HTuple &matchPose) const;
    /** @brief 追加带时间的中文日志。
     * @param message 可显示的消息，不含敏感数据。 */
    void log(const QString &message);
    std::unique_ptr<Ui::MainWindow> ui_;          ///< uic 生成的 Designer 控件映射；控件由 Qt 父子关系释放。
    CloudView *cloudView_ = nullptr;              ///< 可选 PCL 原生窗口，Qt 父对象负责释放，仅 UI 线程访问。
    bool cloudModelPreview_ = false;              ///< 当前三维页是否单独预览模板，禁止将结果表选择当成预览匹配。
    QTimer *settingsSaveTimer_ = nullptr;         ///< 合并连续参数编辑的单次计时器，由 Qt 父对象释放。
    QString settingsPath_;                        ///< 程序目录中的 surface_match_settings.json，测试可注入隔离目录。
    bool settingsEnabled_ = true;                 ///< 是否启用自动持久化，普通 smoke 测试关闭以免污染用户参数。
    bool restoringSettings_ = false;              ///< 批量恢复期间屏蔽变更保存。
    bool settingsDirty_ = false;                  ///< 尚未成功写盘的控件变更。
    QWidget *controls_ = nullptr;                 ///< 忙时禁用的整个参数区。
    std::map<QString, QDoubleSpinBox *> numbers_; ///< 数值控件，Qt 父子对象负责释放。
    QComboBox *accuracy_ = nullptr, *driver_ = nullptr, *kind_ = nullptr, *offsetMode_ = nullptr; ///< 枚举选择。
    QCheckBox *use2D_ = nullptr, *gpu_ = nullptr, *yolo26_ = nullptr, *eyeInHand_ = nullptr,
              *symmetry_ = nullptr; ///< 流程开关。
    QSpinBox *attitude_ = nullptr;  ///< 类别过滤，-1 全部。
    QLineEdit *model2D_ = nullptr, *runtime_ = nullptr, *depthRuntime_ = nullptr, *handEye_ = nullptr,
              *robot_ = nullptr, *taughtModel_ = nullptr, *taughtTcp_ = nullptr, *reference_ = nullptr,
              *center_ = nullptr; ///< 路径及位姿输入。
    QLabel *imageView_ = nullptr, *depthView_ = nullptr, *resultView_ = nullptr, *modelLabel_ = nullptr,
           *graspText_ = nullptr;                 ///< 显示控件。
    QPlainTextEdit *log_ = nullptr;               ///< 只读日志。
    QTableWidget *table_ = nullptr;               ///< 候选位姿列表。
    QFutureWatcher<TaskResult> watcher_;          ///< 后台任务生命周期，窗口销毁前须完成。
    cv::Mat image_, depth_, overlay_;             ///< UI 线程拥有的图像。
    std::optional<cv::Point> selectedImagePixel_; ///< 2D 图当前取值像素，空表示未选择，不随参数持久化。
    std::optional<cv::Point> selectedDepthPixel_; ///< 深度图当前取值像素，保存原图列/行，不保存缩放控件坐标。
    std::shared_ptr<const sm::ModelData> model_;  ///< 当前只读模型。
    sm::MatchResult result_;                      ///< 最后一次匹配。
    sm::CameraIntrinsics resultCamera_;           ///< 匹配时内参快照，防止参数编辑改变旧结果投影。
};
