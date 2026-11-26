# SurfaceMatch C++ 库与 Qt 测试程序

本工程迁移 `htGrab3D/3dObjectGrabbing/Tool.cs`、`SurfaceMatch.cs` 的算法功能，提供 C++ 静态库 `Raivas3DMatch.lib`、C# 可调用的原生动态库 `Raivas3DMatchC.dll`（导入库为 `Raivas3DMatchC.lib`）、托管封装 `SurfaceMatch.Interop.dll` 和 Qt Widgets 测试程序。内部算法仍采用静态函数，模型和参数显式传入，不依赖 WPF、相机 SDK 或机器人通信。

## 构建与 OpenCV 查找

本机已配置 MSVC x64、Qt 5.15.2、HALCON 23.05、OpenCV 4.8。原生 RaiVas DLL 基于 OpenCV 4.8，启用原生后端时使用 Release 配置并保持相同 OpenCV/CRT。

在本目录执行：

```powershell
cmake --preset local-release
cmake --build --preset local-release
ctest --preset local-release
.\build\local\Release\surface_match_demo.exe
```

Qt Creator 也可以直接打开 `CMakeLists.txt`。工程会自动查找本机 `D:/opencv4.8.0/opencv/build/x64/vc16/lib`；显式传入的 `OpenCV_DIR` 优先。若已有构建缓存保存了错误路径，在 Qt Creator 的 CMake 配置中设置：

```text
OpenCV_DIR:PATH=D:/opencv4.8.0/opencv/build/x64/vc16/lib
```

该目录必须包含 `OpenCVConfig.cmake`，不是 OpenCV 的 `include` 或 DLL 目录。切换到 MSVC 64 位 Kit 后重新配置。可单独运行以下命令验证：

```powershell
cmake -S . -B build/manual -G "Visual Studio 17 2022" -A x64 `
  -DOpenCV_DIR=D:/opencv4.8.0/opencv/build/x64/vc16/lib `
  -DCMAKE_PREFIX_PATH=D:/Qt/5.15.2/msvc2019_64
```

当前两个可选原生后端的默认开关为 ON；`local-release` preset 也已启用并填写本机 SDK 路径。Qt Creator 新建 Debug/Release 构建目录时，`cmake/LocalSdkPaths.cmake` 会为启用的后端补齐空 SDK 路径：优先搜索同名环境变量，其次按本机源码相对布局查找原始头文件和 Release DLL。已有非空路径和 OFF 开关保持其显式配置。其他机器请调整 preset 或通过 `-D` 传入路径。

| CMake 参数 | 含义 |
|---|---|
| `HALCON_ROOT` | HALCON 安装目录，默认读取环境变量 HALCONROOT |
| `OpenCV_DIR` | OpenCVConfig.cmake 所在目录 |
| `SM_BUILD_GUI` / `SM_BUILD_TESTS` | 是否构建 Qt 界面 / 验证程序 |
| `SM_BUILD_C_API` | 默认 ON，构建 C# P/Invoke 所需的原生 DLL |
| `SM_ENABLE_PCL_VIEWER` | Windows 默认 ON，启用 PCL/VTK 三维点云页 |
| `SM_ENABLE_RAIVAS` | 是否编译原始 RaiVas 头文件适配层 |
| `RAIVAS_ROOT` | `E:/code/project/OCR/ocr_alg_yolo_obb_opencv` |
| `RAIVAS_RUNTIME_DIR` | 六种封装 DLL 及 ONNX/OpenVINO 依赖的目录；界面可以修改 |
| `SM_ENABLE_DEPTH_FILTER` | 是否启用原始 DepthRadiusFilter.h 接口 |
| `DEPTH_FILTER_ROOT` | `E:/code/study/qt/pcl_rescale_ply` |
| `DEPTH_FILTER_RUNTIME_DIR` | RaivasAlgDepthFilter.dll 所在目录；界面可以修改 |
| `PCL_DIR` | 本机 `D:/Program Files/PCL 1.12.1/cmake`，供深度滤波头文件和三维视图使用；匹配核心不依赖 PCL |

可选 DLL 使用原始头文件中的 `decltype(&函数名)` 获取完整 C++ 签名与调用约定，再按绝对路径加载导出入口。没有重写、复制或猜测原 SDK 的函数声明；不链接 C# 的 P/Invoke 接口。DLL 和依赖目录按需载入，句柄保留到进程结束，避免原 SDK 的全局模型及延迟加载插件失效。原项目和 SDK 文件均未修改。

Qt Creator 已有构建目录中的 `CMakeCache.txt` 会保留旧值，修改 `option(... ON)` 不会覆盖已有的 OFF。启用后端时，在当前 Kit 的 CMake 配置中显式设置 `SM_ENABLE_RAIVAS=ON`、`SM_ENABLE_DEPTH_FILTER=ON` 及上表中的四个 SDK/运行目录，再重新运行 CMake 并构建。界面的“启用原生 2D 后端辅助匹配”控制本次是否使用 2D，仍需选择对应 ONNX 模型并加载 2D 图；深度滤波由按钮单独触发。已有自动配置中的空 DLL 目录也应补齐，否则恢复配置会覆盖编译时的默认目录。

## 界面使用

1. 在“输入”页填写 `fx/fy/cx/cy`、深度比例和偏移，打开 2D 图和深度图。深度必须是单通道 uint16/int16/float32/float64，按原位宽读取。显示归一化不改变算法数据。
   在“2D 原图”或“深度图”页左键单击图像，可在图下查看从零开始的像素坐标和原始值。2D 显示灰度或 BGR 通道，深度显示原始值及当前比例、偏移换算后的 Z，单位与点云一致；零、负值、NaN、Inf 等无效深度会明确标记。缩放和居中留白不会改变取值坐标，读数可以选中复制。
2. 保证图像已去畸变、2D 与深度已配准。设置相机坐标系 XYZ 范围。
3. 在“匹配”页选择精度、匹配数量和分数，加载点云/CAD/SFM。模型和场景必须使用相同长度单位。
4. 可勾选原生 2D 后端，选择 ONNX 模型、任务 OBB/Seg/Det、ONNX/OpenVINO 驱动及 YOLO11/26。`-1` 接受所有类别。OBB 保留原方向一致性门限；Seg 使用原图大小掩膜限制点云；Det 不进行没有依据的方向筛选。
5. 点击固定在侧栏底部的“运行 runModel”，查看分数、六维位姿、耗时及模型点云投影。新增“3D 点云结果”页会显示完整场景和匹配模板；计算和显示数据提取在后台运行，同一时间只接受一个任务。
6. 在“抓手偏移”页填标定和示教位姿，选择候选后计算最终抓手偏移。可导出配置 JSON，以及候选模型位姿、绝对 TCP、最终输出位姿 JSON。

“显示当前深度的 3D 点云”下方的“保存 3D 点云”会按当前内参、深度换算和 XYZ 范围重新生成相机坐标系点云，保存完整有效点到 PLY 文件，不使用显示抽样。“加载 3D 模型 / SFM”下方的“显示 3D 模型”会单独预览已加载模板，保持模板坐标系，并保留已有匹配结果。

全部相机、裁剪、匹配、推理路径、标定位姿和三维外观参数自动记录到 **EXE 所在目录的 `surface_match_settings.json`**。连续编辑合并为 500 毫秒后的原子写入，关闭时保存最后尚未落盘的修改，启动时恢复。目录不可写时在日志中显示真实路径和错误，不转存到其他目录。损坏文件保持原样，未修改参数时不会自动覆盖；普通 `--smoke` 验证不读写用户参数。

## Qt Designer 与 PCL 三维视图

直接在 Qt Creator / Designer 中打开 `app/MainWindow.ui` 即可调整所有静态控件、标签页、布局、默认范围和文字。`MainWindow.cpp` 只保留控件引用、事件绑定、参数读写与算法调度；`ui_MainWindow.h` 由 CMake AUTOUIC 生成，**不要手工编辑生成头文件**。

主要对象名：`fxSpin/fySpin/cxSpin/cySpin` 为内参，`runButton` 为运行入口，`resultTable` 为候选结果，`viewTabs` 为显示标签页，`cloudHostLayout` 为 PCL 渲染布局容器。修改对象名时同步修改 C++ 引用和隔离测试。

使用的 PCL 是 `D:/Program Files/PCL 1.12.1`，VTK 为其自带的 `3rdParty/VTK`。该安装没有 GUISupportQt，因此通过 Qt 原生 HWND 嵌入 VTK OpenGL，并由 `vtkGenericRenderWindowInteractor` 接收 Qt 鼠标事件，无须重编译 VTK。实现依据 [VTK 原生事件交互接口](https://vtk.org/doc/nightly/release/9.1/html/classvtkGenericRenderWindowInteractor.html)。

- 灰色为场景，绿色为匹配模板，选中的候选为橙色；表格选择与高亮联动。
- 左键拖动旋转，中键拖动平移，滚轮或右键拖动缩放；“重置视角”恢复全景。
- 支持隐藏场景/模板、改变点大小、保存真实三维渲染截图。
- 场景默认最多显示 250000 点，模板最多显示 80000 点，显示抽样不影响 HALCON 匹配数据。
- PCL/VTK 只在 UI 线程操作；后台只生成不可变 XYZ/矩阵快照。

## C# 调用与动态库

C# `DllImport` 不能直接调用静态 `.lib`。本工程把已有静态算法链接进 `Raivas3DMatchC.dll`，再导出 `extern "C"`、固定 Cdecl 的稳定 C ABI；静态函数不影响 DLL 封装。C# 推荐使用 `bindings/csharp/SurfaceMatchSession.cs`，或引用构建生成的 `SurfaceMatch.Interop.dll`。设计依据 [Microsoft 原生互操作建议](https://learn.microsoft.com/dotnet/standard/native-interop/abi-support)。

已有 Grab3D 工程目标为 .NET Framework 4.8，因此封装和真实跨进程 ABI 测试按 **.NET Framework 4.8 / x64** 构建。该 C# 源码也可纳入较新的 .NET 项目。托管实例通过 SafeHandle 释放原生模型和图像；所有路径 UTF-8，异常在 DLL 内转换为状态码和错误文本。

```csharp
using SurfaceMatch.Interop;

using (var session = new SurfaceMatchSession())
{
    var config = SurfaceMatchSession.DefaultConfig;
    config.Fx = 1200; config.Fy = 1200;
    config.Cx = 640; config.Cy = 480;
    config.DepthScale = 1;
    session.SetConfig(config);
    session.LoadImages("Depth.tiff", "2D.bmp");
    session.LoadModel("model.ply");
    var matches = session.RunModel();
    // matches[i].Pose 为模型到相机的 abg 位姿；模板保留参考相机坐标时可作为目标偏移。
    // 示例标定值须替换为实际“相机到基座”标定，平移单位须统一。
    var handEye = new Pose { X = 100, Y = 200, Z = 300, Rz = 90 };
    if (matches.Length > 0)
    {
        Pose relative = SurfaceMatchSession.TransformPose(matches[0].Pose, false, handEye, null);
        // relative 是基座系左乘相对变换，不是绝对 TCP，也不能与机器人 XYZ/角度逐项相加。
    }
}
```

也可用 `SetImage(byte[],...)` 传入像素，或用 `SetImage(IntPtr,...)` 传入 OpenCvSharp 的 `Mat.Data` 和步长。DLL 在调用期间复制像素，返回后调用方可释放原内存，避免不同 OpenCV 版本的 `cv::Mat` 跨模块传递。

当前 C ABI 覆盖配置、图像文件/原始缓冲区、模板读取/深度建模、2D 可选后端的 runModel、候选复制与最终抓手转换。完整 Tool 工具集仍可从 C++ 公共头文件调用。C ABI 定义见 `include/surface_match/CApi.h`，C# 结构尺寸与调用约定有真实测试验证。

保留会话但暂时卸载三维模型时，C 接口调用 `SmDisposeModel(context)`，C# 调用 `session.DisposeModel()`。它释放模型点云和表面匹配模型、清空原生匹配缓存，保留图像、相机参数和匹配配置；不删除磁盘文件，也不改变此前复制出的候选数组。无模型和重复卸载均成功，空 C 句柄返回 `SM_INVALID_ARGUMENT`，已整体 Dispose 的 C# 会话会抛出 `ObjectDisposedException`。卸载后匹配报告 `SM_NOT_READY`，须重新 `LoadModel` 或 `CreateModelFromDepth`；最终仍应 `Dispose()` 释放整个会话。同一会话的卸载与匹配共用锁，可能等待当前计算完成，C 调用方不得同时销毁会话。C++ 静态接口返回的模型仍由 `shared_ptr` 管理，释放最后一个引用即可。

部署 C# 时需要 `SurfaceMatch.Interop.dll`、`Raivas3DMatchC.dll`、`halconcpp.dll`、`halcon.dll`、`opencv_world480.dll` 和适配的 VC++ 运行时，HALCON 还需有效许可证。仅算法调用不需要 Qt、PCL 或 VTK；只有使用可选 RaiVas 后端时才需要其 DLL 及依赖。若用 Ninja 构建，C# 项目可在 Visual Studio 中单独编译；VS 生成器的默认测试构建会自动生成托管封装与调用示例。

合成图片与从深度图建模保留用于底层接口和 `--smoke` 自动验证，界面使用外部图片及模型文件进行测试。

原 `RaivasAlgDepthFilter` 导出函数内部实际固定调用 `filter(...,5,4,1)`，忽略其他门限参数。本界面明确沿用该实现，不提供看似可调但实际无效的参数。

## 抓手 transformPose

主接口为 `sm::SurfaceMatch::transformPose(matchPose, eyeInHand, handEyePose, robotPose)`，直接返回七元抓手相对偏移，输出为 HALCON `Rp+T/abg/point`，角度为度。

约定 `T_A_B` 表示从 B 坐标到 A 坐标的齐次变换，`D_C` 为相机系的目标偏移。所有平移使用统一长度单位。

```text
眼在手外：T_B_C = handEyePose
眼在手上：T_B_C = robotPose × handEyePose
返回偏移：D_B   = T_B_C × D_C × inverse(T_B_C)
应用偏移：T_B_TCP_new = D_B × T_B_TCP_reference
```

| 参数 | 含义 |
|---|---|
| matchPose | 单个七元相机系目标偏移 `D_C` |
| eyeInHand | true 眼在手上，false 固定相机 |
| handEyePose | 眼在手上为相机到标定末端；眼在手外为相机到基座 |
| robotPose | 拍照时标定末端到基座；眼在手上必填，眼在手外允许 nullptr 并忽略 |

`runModel` 原始输出是模型到当前相机的位姿。模板保留参考相机坐标，且参考关系未改变时，它可直接作为 `D_C`。如果模板移到了局部原点、相机参考发生变化，须先用真实参考把匹配绝对位姿恢复为当前相机系的偏移；四参数接口没有示教模板输入，无法推导缺失的参考关系。

完整矩阵换基会保留旋转产生的平移补偿。例如相机位于基座 `(100,0,0)` 且轴向一致，目标绕相机 Z 轴转 90°，返回基座偏移平移为 `(100,-100,0)`、Rz 为 90°；左乘参考 TCP `(110,0,0)` 后得到 `(100,10,0)`。返回的 XYZ 并不是某个具体 TCP 的位置差，也不能直接对欧拉角做加法。

C# 对应 `Pose TransformPose(Pose matchPose, bool eyeInHand, Pose handEyePose, Pose? robotPose = null)`；固定相机可省略最后一项，眼在手上漏传会明确报错。C DLL 新增 `SmTransformRelativePose`，原有 `SmTransformPose` 和结构布局保持兼容。

Qt 抓手页和结果导出均调用四参数接口；固定相机禁用机器人输入，旧示教/控制器控件隐藏，但继续保留已有 JSON 中的旧字段值。导出 `outputPose` 为相对偏移，并附带 `outputPoseConvention=base_relative_left_multiply`，不再生成绝对 `tcpInBase`。

旧版 `transformPose(matchPose, GraspTransform)` 仍作为兼容重载保留，支持示教模板、示教 TCP、对称补偿及 Absolute/Fanuc/Nachi 输出。两个接口都不操作机器人，也不进行 IK、碰撞校验或机器人专用角度转换。

## 库调用示例

```cpp
#include <surface_match/SurfaceMatch.h>

// 以下调用放在已有函数中；异常由应用边界捕获并记录。
sm::CameraIntrinsics camera;
camera.fx = 1200;
camera.fy = 1200;
camera.cx = 640;
camera.cy = 480;
camera.depthScale = 1; // 深度已是毫米时使用 1，模型与机器人平移也须为毫米。

sm::MatchParameters parameters;
sm::SurfaceMatch::setPara(parameters, sm::MatchingAccuracy::Normal);
auto depth = sm::Tool::readImage("Depth.tiff", true);
auto model = sm::SurfaceMatch::initModel("model.ply", parameters);
auto matches = sm::SurfaceMatch::runModel(*model, {}, depth, camera, {}, parameters);

// 示例数值须替换为实际“相机到基座”标定；模板保留参考相机坐标。
HalconCpp::HTuple handEye;
HalconCpp::CreatePose(100, 200, 300, 0, 0, 90, "Rp+T", "abg", "point", &handEye);
if (matches.scores.Length() > 0) {
    auto selected = matches.poses.TupleSelectRange(0, 6);
    auto relative = sm::SurfaceMatch::transformPose(selected, false, handEye, nullptr);
    // relative 为七元抓手相对位移，用于左乘参考 TCP。
}
```

已有 CMake 应用可通过 `add_subdirectory` 引入，再链接 `Raivas3DMatch::Raivas3DMatch`；原 `SurfaceMatch::surface_match` 别名保留兼容。CMake 的实际库目标为 `Raivas3DMatch` 和 `Raivas3DMatchC`。`cmake --install build/local --config Release --prefix <目标目录>` 可以安装静态库与公共头文件；使用静态库时仍需链接 HALCON/OpenCV，并部署它们的运行时。

## 原函数迁移对应

| 原功能 | C++ 入口 |
|---|---|
| LimitImage / DrawRotatedRect / MatToHObject / MatToHObjectDepth | `sm::Tool` 同名静态函数 |
| getPoint3DValue / get3DPoints / depthImgToObjectModel3D / _S | `sm::Tool` 同名函数，内参与范围收拢到值类型 |
| filter / filterZ / pointTransform / pointCloudSplice | `sm::Tool` 同名函数，返回 RAII 点云 |
| gripper2base / transformPose / transformPose2Target / getHTarget2Gripper | `sm::Tool` 保留原绝对变换用途 |
| runModel 到抓手偏移 | `sm::SurfaceMatch::transformPose` 四参数接口进行相对变换换基；保留示教链路兼容重载 |
| poseRotate / poseRotateYToZero / pose_translate_local | `sm::Tool` 同名函数 |
| Line3DFitter、四元数平均、平面和多点定位 | `sm::Tool`，保留多点入口名称，公共实现为 `mulPointToPose` |
| getHighestModel / getNoTouchModel | `sm::Tool` 返回候选索引列表；取首项前检查是否为空 |
| initModel / initModel3D / run3DModel / runModel | `sm::SurfaceMatch`，显式只读 `ModelData` |
| initModel2D / initShapeModel / findShapeModel | `sm::SurfaceMatch` HALCON 形状模型入口 |
| locationImage / coarseLocation / fineLocation | 显式传入二维检测或中心定位结果，避免隐藏的全局模型和路径状态 |
| runMulObjModel | 单个模板多帧统一坐标匹配；逐帧模式固定每帧一个目标，避免多目标姿态混平均 |
| init/run/get OBB/Seg/Det 与 Vino、DepthRadiusFilter | `sm::NativeBackend` 直接适配原生头文件 |

原未实现的两点粗定位已用两点精定位规则补全；原分列不均导致丢点、共线点归一化产生 NaN、零深度被偏移变成点云、临时句柄不释放等问题已处理。模型法线按像素映射定向，已有 CAD 法线保留，避免模型/场景朝向不一致导致翻面匹配。相关 HALCON 规则见 [HALCON 23.05 FindSurfaceModel](https://www.mvtec.com/doc/halcon/2305/en/find_surface_model.html)。

## 验证

`capi_lifecycle` 从独立 C++ 程序调用真实 C DLL，验证模型卸载后缓存立即清空、空模型重复卸载、错误状态、保留图像参数并重新建模匹配；`csharp_interop` 同时验证 `DisposeModel()` 的托管封装和会话销毁后的调用限制。

`algorithms` 验证真实 HALCON 建模与匹配、已知非单位旋转平移、深度反投影、无效值和掩膜、非连续 Mat 生命周期、PCA/平面/四元数、Fanuc/Nachi 与对称中心补偿、空场景和 SFM 往返。`gui_smoke` 自动完成合成数据建模、后台匹配、表格和图像渲染，保存 `surface_match_smoke.png`。

另外，`csharp_interop` 从 .NET Framework 4.8 x64 实际调用 DLL，验证结构布局、UTF-8、像素所有权、HALCON 匹配、抓手转换和 SafeHandle；`pcl_viewer` 使用隐藏的真实 Windows OpenGL 窗口验证旋转、平移、滚轮缩放、候选高亮和截图；`gui_native_smoke` 验证完整 Designer 主窗口到嵌入式点云窗口的真实渲染链路；`settings_persistence` 在隔离目录验证参数自动记录和恢复，并通过真实按钮验证模板预览、中文路径下完整 PLY 导出与已有匹配结果保留，不修改用户的参数或图像。

原生 SDK 验证可手动运行（使用用户现有的真实模型与图片）：

```powershell
.\build\local\Release\surface_match_tests.exe --native `
  E:/code/project/OCR/ocr_alg_yolo_obb_opencv/build/codex-openvino-check/Release `
  E:/code/study/qt/opencv_onnxruntime/data `
  E:/code/study/qt/pcl_rescale_ply/build/Desktop_Qt_5_15_2_MSVC2019_64bit-Release
```

该命令验证 YOLO11 OBB/Det 的 ONNX 与 OpenVINO CPU 调用，以及原生深度滤波。Seg、YOLO26 和 GPU 接口已按原头文件接入，需要相应模型和设备进行额外实测。
