using System;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;

namespace SurfaceMatch.Interop
{
    /// <summary>与 C ABI 1 对应的 Pose 值结构，按八字节对齐；长度单位须统一，位姿为 abg 度，不持有原生资源。</summary>
    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct Pose
    {
        /// <summary>X 平移。</summary>
        public double X;
        /// <summary>Y 平移。</summary>
        public double Y;
        /// <summary>Z 平移。</summary>
        public double Z;
        /// <summary>X 旋转，度。</summary>
        public double Rx;
        /// <summary>Y 旋转，度。</summary>
        public double Ry;
        /// <summary>Z 旋转，度。</summary>
        public double Rz;
        /// <summary>HALCON pose 类型，本 ABI 统一为 0。</summary>
        public double Type;
    }
    /// <summary>与 C ABI 1 对应的 Config 值结构，按八字节对齐；长度单位须统一，位姿为 abg 度，不持有原生资源。</summary>
    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct Config
    {
        /// <summary>X 焦距，像素，有限正数。</summary>
        public double Fx;
        /// <summary>Y 焦距，像素，有限正数。</summary>
        public double Fy;
        /// <summary>主点列，像素，有限值。</summary>
        public double Cx;
        /// <summary>主点行，像素，有限值。</summary>
        public double Cy;
        /// <summary>深度比例，有限正数。</summary>
        public double DepthScale;
        /// <summary>深度偏移，有限值。</summary>
        public double DepthOffset;
        /// <summary>相机系 X 下界。</summary>
        public double MinX;
        /// <summary>相机系 X 上界。</summary>
        public double MaxX;
        /// <summary>相机系 Y 下界。</summary>
        public double MinY;
        /// <summary>相机系 Y 上界。</summary>
        public double MaxY;
        /// <summary>相机系 Z 下界。</summary>
        public double MinZ;
        /// <summary>相机系 Z 上界。</summary>
        public double MaxZ;
        /// <summary>匹配阈值，[0,1]。</summary>
        public double MinScore;
        /// <summary>重叠相对距离，非负。</summary>
        public double MaxOverlapDistance;
        /// <summary>模型相对采样，(0,1]。</summary>
        public double ModelSampling;
        /// <summary>场景相对采样，(0,1]。</summary>
        public double SceneSampling;
        /// <summary>关键点比例，(0,1]。</summary>
        public double KeyPointFraction;
        /// <summary>降采样绝对间距，0 关闭。</summary>
        public double DownsampleDistance;
        /// <summary>OBB 角度容差，度，[0,180]。</summary>
        public double AngleTolerance;
        /// <summary>最大匹配数量，正整数。</summary>
        public int NumMatches;
        /// <summary>模型 MLS 邻居数，0 或至少 3。</summary>
        public int ModelSmoothK;
        /// <summary>场景 MLS 邻居数，0 或至少 3。</summary>
        public int SceneSmoothK;
        /// <summary>框扩张像素，非负整数。</summary>
        public int RoiExpansion;
    }
    /// <summary>与 C ABI 1 对应的 Match 值结构，按八字节对齐；长度单位须统一，位姿为 abg 度，不持有原生资源。</summary>
    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct Match
    {
        /// <summary>模型到当前相机的七元 abg pose。</summary>
        public Pose Pose;
        /// <summary>匹配分数。</summary>
        public double Score;
        /// <summary>2D 方向弧度，纯 3D 模式为零。</summary>
        public double Radians2D;
    }
    /// <summary>与 C ABI 1 对应的 GraspConfig 值结构，按八字节对齐；长度单位须统一，位姿为 abg 度，不持有原生资源。</summary>
    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct GraspConfig
    {
        /// <summary>当前相机到基座。</summary>
        public Pose CameraInBase;
        /// <summary>示教模板到基座。</summary>
        public Pose TaughtModelInBase;
        /// <summary>示教抓手 TCP 到基座。</summary>
        public Pose TaughtTcpInBase;
        /// <summary>控制器偏移基准；绝对模式忽略。</summary>
        public Pose OffsetReferenceInBase;
        /// <summary>模板中心 X，仅对称补偿时使用。</summary>
        public double CenterX;
        /// <summary>模板中心 Y，仅对称补偿时使用。</summary>
        public double CenterY;
        /// <summary>模板中心 Z，仅对称补偿时使用。</summary>
        public double CenterZ;
        /// <summary>0 绝对 TCP，1 Fanuc 矩阵偏移，2 Nachi 位置差。</summary>
        public int Mode;
        /// <summary>0 保留旋转，1 去除局部 Z 对称角并补偿中心。</summary>
        public int RotationalSymmetry;
    }
    /// <summary>与 C ABI 1 对应的 GraspResult 值结构，按八字节对齐；长度单位须统一，位姿为 abg 度，不持有原生资源。</summary>
    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct GraspResult
    {
        /// <summary>当前模板到基座。</summary>
        public Pose ModelInBase;
        /// <summary>基座系模板运动。</summary>
        public Pose MotionInBase;
        /// <summary>绝对抓手 TCP。</summary>
        public Pose TcpInBase;
        /// <summary>最终发送位姿或偏移。</summary>
        public Pose OutputPose;
    }

    /// <summary>原始像素格式，值与 C ABI 固定对应，不作为标志组合。</summary>
    public enum PixelFormat
    {
        /// <summary>单通道八位灰度，仅用于 2D 图。</summary>
        Gray8 = 1,
        /// <summary>三通道八位 BGR，仅用于 2D 图。</summary>
        Bgr8 = 2,
        /// <summary>单通道无符号十六位深度。</summary>
        DepthU16 = 3,
        /// <summary>单通道三十二位浮点深度。</summary>
        DepthF32 = 4,
        /// <summary>单通道六十四位浮点深度。</summary>
        DepthF64 = 5
    }

    /// <summary>使用 SafeHandle 管理原生算法会话，提供 .NET Framework 4.8 和较新 .NET 可调用的同步封装。</summary>
    /// <remarks>同一实例的调用串行；耗时计算可整体放入 Task.Run。必须 Dispose；P/Invoke 期间 SafeHandle 保持句柄有效。
    /// 图像输入由 DLL 复制，结果由托管数组持有，不将 OpenCV 或 HALCON 对象跨 ABI 传递。</remarks>
    public sealed class SurfaceMatchSession : IDisposable
    {
        private readonly object gate = new object(); // 保护一次运行与结果复制的完整流程。
        private readonly NativeHandle handle; // 原生资源由 SafeHandle 唯一释放。

        /// <summary>检查原生 ABI 并创建会话；不加载模型或图像。</summary>
        /// <exception cref="NotSupportedException">原生 ABI 不是 1。</exception>
        /// <exception cref="InvalidOperationException">原生会话创建失败。</exception>
        public SurfaceMatchSession()
        {
            if (Native.SmGetAbiVersion() != 1) throw new NotSupportedException("原生 DLL 的 C ABI 版本不受支持");
            Check(Native.SmCreate(out handle));
        }

        /// <summary>获取原生默认配置，并核对托管结构尺寸，返回值可独立修改。</summary>
        public static Config DefaultConfig
        {
            get { Config config; Check(Native.SmGetDefaultConfig(out config, (uint)Marshal.SizeOf(typeof(Config)))); return config; }
        }

        /// <summary>校验并替换相机、范围与匹配配置，清除上一次结果。</summary>
        /// <param name="config">由 DefaultConfig 初始化后修改的有效配置；训练参数改变后须重新建模。</param>
        /// <exception cref="ObjectDisposedException">会话已释放。</exception>
        /// <exception cref="InvalidOperationException">原生配置验证失败，消息包含实际原因。</exception>
        public void SetConfig(Config config)
        {
            lock (gate) { EnsureOpen(); Check(Native.SmSetConfig(handle, ref config, (uint)Marshal.SizeOf(typeof(Config)))); }
        }

        /// <summary>读取图像文件，深度保留位宽，成功后清除旧匹配。</summary>
        /// <param name="depthPath">非空深度图路径，使用 UTF-8 封送，支持中文。</param>
        /// <param name="imagePath">可选配准 2D 路径；null 表示纯 3D。</param>
        /// <exception cref="InvalidOperationException">文件、图像格式或原生操作失败。</exception>
        public void LoadImages(string depthPath, string imagePath = null)
        {
            lock (gate) { EnsureOpen(); Check(Native.SmLoadImages(handle, imagePath, depthPath)); }
        }

        /// <summary>固定托管像素数组后交给原生 DLL 复制，返回时立即解除固定。</summary>
        /// <param name="pixels">非空像素字节数组，可含行尾填充，调用期间不得并发修改。</param>
        /// <param name="width">像素宽度，正整数。</param>
        /// <param name="height">像素高度，正整数。</param>
        /// <param name="stride">每行字节数，至少为宽度乘单像素字节数。</param>
        /// <param name="format">与数据位宽一致的像素格式。</param>
        /// <param name="depth">true 设置深度图，false 设置 2D 图；格式必须与种类一致。</param>
        /// <exception cref="ArgumentException">数组为空。</exception>
        /// <exception cref="InvalidOperationException">原生缓冲区大小或图像格式验证失败。</exception>
        public void SetImage(byte[] pixels, int width, int height, ulong stride, PixelFormat format, bool depth)
        {
            if (pixels == null || pixels.Length == 0) throw new ArgumentException("像素数组不能为空", nameof(pixels));
            GCHandle pin = GCHandle.Alloc(pixels, GCHandleType.Pinned);
            try { SetImage(pin.AddrOfPinnedObject(), (ulong)pixels.LongLength, width, height, stride, format, depth); }
            finally { pin.Free(); }
        }

        /// <summary>从调用方的非托管像素地址复制图像，适用于 OpenCvSharp Mat.Data 等输入。</summary>
        /// <param name="pixels">有效像素首地址，调用期间不得释放或修改，不转移所有权。</param>
        /// <param name="bufferBytes">可访问的缓冲区总字节数，必须覆盖所有行。</param>
        /// <param name="width">像素宽度，正整数。</param>
        /// <param name="height">像素高度，正整数。</param>
        /// <param name="stride">每行字节数，允许尾部填充。</param>
        /// <param name="format">像素位宽和通道格式。</param>
        /// <param name="depth">true 为深度输入，false 为 2D 输入。</param>
        /// <exception cref="InvalidOperationException">原生输入验证或复制失败。</exception>
        public void SetImage(IntPtr pixels, ulong bufferBytes, int width, int height, ulong stride, PixelFormat format, bool depth)
        {
            lock (gate) { EnsureOpen(); Check(Native.SmSetImage(handle, depth ? 1 : 0, pixels, bufferBytes, width, height, stride, (int)format)); }
        }

        /// <summary>读取点云/CAD 并训练，或直接加载 SFM，失败保留旧模型。</summary>
        /// <param name="path">非空模型文件路径，使用 UTF-8 封送。</param>
        /// <param name="fileScale">HALCON 输入文件单位，默认 m；须与深度最终长度单位一致。</param>
        /// <exception cref="InvalidOperationException">模型读取或建模失败。</exception>
        public void LoadModel(string path, string fileScale = "m")
        {
            lock (gate) { EnsureOpen(); Check(Native.SmLoadModel(handle, path, fileScale)); }
        }

        /// <summary>卸载模型点云和表面匹配模型，并清空原生匹配缓存，保留会话、输入图像及全部算法配置。</summary>
        /// <exception cref="ObjectDisposedException">会话已经整体 Dispose，不能再卸载其中的模型。</exception>
        /// <exception cref="InvalidOperationException">原生卸载失败，消息包含实际错误信息。</exception>
        /// <remarks>未加载模型和重复调用均成功，不删除磁盘文件，也不改变已返回的托管候选数组。
        /// 与本实例其他操作共用锁，可能等待正在执行的匹配；卸载后必须 LoadModel 或 CreateModelFromDepth 才能再次匹配。
        /// 本方法不替代会话的 Dispose，使用结束仍须释放整个会话。</remarks>
        public void DisposeModel()
        {
            lock (gate) { EnsureOpen(); Check(Native.SmDisposeModel(handle)); }
        }

        /// <summary>以当前深度图、内参和空间范围创建表面模板，清除旧匹配；用于快速验证主流程。</summary>
        /// <exception cref="InvalidOperationException">没有深度或 HALCON 建模失败。</exception>
        public void CreateModelFromDepth()
        {
            lock (gate) { EnsureOpen(); Check(Native.SmCreateModelFromDepth(handle)); }
        }

        /// <summary>同步运行匹配并在同一锁内复制全部候选，无目标时返回空数组；失败不返回旧结果。</summary>
        /// <param name="detectorPath">可选 2D ONNX 模型路径；null 表示纯 3D。</param>
        /// <param name="runtimeDirectory">启用 2D 时的原生 DLL 与依赖目录。</param>
        /// <param name="kind">0 OBB、1 Seg、2 Det。</param>
        /// <param name="driver">0 ONNX、1 OpenVINO。</param>
        /// <param name="threshold">2D 置信度，范围 [0,1]。</param>
        /// <param name="useGpu">是否交给原 SDK 使用 GPU。</param>
        /// <param name="yolo26">true YOLO26，false YOLO11。</param>
        /// <param name="attitudeId">-1 全类别，否则匹配指定类别。</param>
        /// <returns>独立托管候选数组，每项包含模型到相机 pose、分数和 2D 方向。</returns>
        /// <exception cref="InvalidOperationException">模型、深度或原生匹配失败。</exception>
        public Match[] RunModel(string detectorPath = null, string runtimeDirectory = null, int kind = 0, int driver = 0,
                                double threshold = 0.5, bool useGpu = false, bool yolo26 = false, int attitudeId = -1)
        {
            lock (gate)
            {
                EnsureOpen();
                Check(Native.SmRunModel(handle, detectorPath, runtimeDirectory, kind, driver, threshold, useGpu ? 1 : 0, yolo26 ? 1 : 0, attitudeId));
                ulong count; int status = Native.SmCopyMatches(handle, null, 0, out count);
                if (status != 4) Check(status);
                if (count > int.MaxValue) throw new OverflowException("候选数量超过托管数组上限");
                var results = new Match[(int)count];
                if (count != 0) Check(Native.SmCopyMatches(handle, results, count, out count));
                return results;
            }
        }

        /// <summary>将相机系匹配偏移转换为基座系抓手相对位移，内部使用 C*matchPose*inverse(C)，包含旋转杠杆臂。</summary>
        /// <param name="matchPose">单个相机系目标偏移，Type=0、abg 度；runModel 模板须保留参考相机坐标，局部模板须先恢复参考偏移。</param>
        /// <param name="eyeInHand">true 为眼在手上，false 为固定相机。</param>
        /// <param name="handEyePose">眼在手上为相机到标定末端，眼在手外为相机到基座；Type=0、abg 度。</param>
        /// <param name="robotPose">拍照时标定末端到基座的位姿，眼在手上必须提供；眼在手外允许 null 并忽略此值。</param>
        /// <returns>七元基座系相对 Pose，平移与输入同单位，角度为度；以矩阵左乘参考 TCP 得到新的 TCP，不是分量加法。</returns>
        /// <exception cref="ArgumentException">眼在手上却没有提供 robotPose。</exception>
        /// <exception cref="InvalidOperationException">位姿或原生运算无效，包含原生错误说明。</exception>
        /// <remarks>不依赖会话，不操作机器人，无需释放返回值；所有平移单位必须一致，可并发调用。</remarks>
        public static Pose TransformPose(Pose matchPose, bool eyeInHand, Pose handEyePose, Pose? robotPose = null)
        {
            Pose result;
            if (eyeInHand)
            {
                if (!robotPose.HasValue) throw new ArgumentException("眼在手上必须提供拍照时的机器人位姿", nameof(robotPose));
                var robot = robotPose.Value;
                Check(Native.SmTransformRelativePose(ref matchPose, 1, ref handEyePose, ref robot, out result));
            }
            else
                Check(Native.SmTransformRelativePose(ref matchPose, 0, ref handEyePose, IntPtr.Zero, out result));
            return result;
        }

        /// <summary>兼容旧版示教链路，把单个匹配目标 pose 转成基座模型运动、绝对 TCP 与最终抓手偏移，不操作机器人。</summary>
        /// <param name="matchPose">runModel 的一个模型到相机七元 abg pose。</param>
        /// <param name="config">真实标定和示教参考；Mode 为 0 绝对、1 Fanuc、2 Nachi。</param>
        /// <returns>四阶段抓手转换结果，平移与输入同单位。</returns>
        /// <exception cref="InvalidOperationException">位姿、标定或模式无效。</exception>
        public static GraspResult TransformPose(Pose matchPose, GraspConfig config)
        {
            GraspResult result; Check(Native.SmTransformPose(ref matchPose, ref config, out result)); return result;
        }

        /// <summary>串行释放原生会话；可重复调用，释放后的实例不再接受算法请求。</summary>
        public void Dispose() { lock (gate) { handle.Dispose(); } }

        /// <summary>检查会话生命周期，防止把已释放句柄送入原生代码。</summary>
        /// <exception cref="ObjectDisposedException">SafeHandle 已关闭或无效。</exception>
        private void EnsureOpen() { if (handle.IsClosed || handle.IsInvalid) throw new ObjectDisposedException(nameof(SurfaceMatchSession)); }

        /// <summary>将非零原生状态转成携带 UTF-8 错误的托管异常，立即复制本线程错误文本。</summary>
        /// <param name="status">C ABI 状态，0 成功，非零失败。</param>
        /// <exception cref="InvalidOperationException">原生状态非零。</exception>
        private static void Check(int status)
        {
            if (status == 0) return;
            IntPtr text = Native.SmGetLastErrorMessage(); int length = 0;
            while (Marshal.ReadByte(text, length) != 0) ++length;
            byte[] bytes = new byte[length]; Marshal.Copy(text, bytes, 0, length);
            throw new InvalidOperationException("SurfaceMatch (" + status + "): " + Encoding.UTF8.GetString(bytes));
        }

        /// <summary>原生句柄的唯一托管所有者，P/Invoke 调用期间由运行时保持有效，GC 兜底释放。</summary>
        internal sealed class NativeHandle : SafeHandleZeroOrMinusOneIsInvalid
        {
            /// <summary>供 P/Invoke 创建空 SafeHandle 外壳，成功创建后由运行时填写句柄。</summary>
            public NativeHandle() : base(true) { }
            /// <summary>释放原生会话，只调用不会向托管边界抛出 C++ 异常的 C API。</summary>
            /// <returns>原生销毁成功时 true，否则 false。</returns>
            protected override bool ReleaseHandle() { return Native.SmDestroy(handle) == 0; }
        }

        /// <summary>与 CApi.h 一一对应的内部声明，固定 Cdecl 和精确导出名称，避免 C++ 名字修饰。</summary>
        private static class Native
        {
            private const string Library = "Raivas3DMatchC.dll"; // 与 CMake 输出的原生 DLL 名称一致。
            /// <summary>查询此 DLL 的 C ABI 版本，不分配资源。</summary>
            /// <returns>C ABI 版本号。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern uint SmGetAbiVersion();
            /// <summary>获取本线程最后错误的 UTF-8 文本，不清除错误状态。</summary>
            /// <returns>本线程借用的 UTF-8 错误指针，须立即复制。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern IntPtr SmGetLastErrorMessage();
            /// <summary>初始化默认配置并验证调用方结构尺寸。</summary>
            /// <param name="config">非空输出结构，调用方分配，不转移所有权。</param>
            /// <param name="configSize">调用方 sizeof(SmConfig)，必须与 ABI 1 一致。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmGetDefaultConfig(out Config config, uint configSize);
            /// <summary>创建独立算法会话，失败时输出置空。</summary>
            /// <param name="context">非空句柄输出地址；成功后调用方必须恰好调用一次 SmDestroy。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmCreate(out NativeHandle context);
            /// <summary>释放会话中的模型、图像和缓存；nullptr 是无操作。</summary>
            /// <param name="context">仍有效的唯一会话句柄，不得与其他调用并发，不得重复释放。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmDestroy(IntPtr context);
            /// <summary>校验并替换会话配置，成功后清除旧匹配，保留图像和已训练模型。</summary>
            /// <param name="context">有效会话句柄。</param>
            /// <param name="config">调用期间有效的只读配置结构。</param>
            /// <param name="configSize">sizeof(SmConfig)，用于拒绝错误版本或封送布局。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmSetConfig(NativeHandle context, ref Config config, uint configSize);
            /// <summary>读取配准图像文件，深度保留原位宽；成功后清除旧匹配。</summary>
            /// <param name="context">有效会话句柄。</param>
            /// <param name="imagePath">可选 UTF-8 2D 路径，nullptr 或空串表示纯 3D 输入。</param>
            /// <param name="depthPath">非空 UTF-8 深度文件路径。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmLoadImages(NativeHandle context, [MarshalAs(UnmanagedType.LPUTF8Str)] string imagePath, [MarshalAs(UnmanagedType.LPUTF8Str)] string depthPath);
            /// <summary>从原始像素缓冲区复制一张图像，不保留调用方内存引用。</summary>
            /// <param name="context">有效会话句柄。</param>
            /// <param name="imageKind">0 表示 2D 图，1 表示深度图。</param>
            /// <param name="pixels">调用期间有效的输入地址，非空；返回后可释放或解除固定。</param>
            /// <param name="bufferBytes">输入缓冲区可访问的总字节数，必须包含全部行。</param>
            /// <param name="width">像素宽度，正整数。</param>
            /// <param name="height">像素高度，正整数。</param>
            /// <param name="stride">每行字节数，正数，至少为 width 乘单像素大小，允许尾部填充。</param>
            /// <param name="pixelFormat">1=Gray8，2=BGR8，3=DepthU16，4=DepthF32，5=DepthF64；须与 imageKind 一致。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmSetImage(NativeHandle context, int imageKind, IntPtr pixels, ulong bufferBytes, int width, int height, ulong stride, int pixelFormat);
            /// <summary>从文件读取并训练模板，.sfm 直接加载；成功后替换旧模型并清除旧匹配。</summary>
            /// <param name="context">有效会话句柄。</param>
            /// <param name="path">非空 UTF-8 模型文件路径。</param>
            /// <param name="fileScale">HALCON 文件单位，如 m/mm，nullptr 默认 m；模型须与场景最终单位一致。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmLoadModel(NativeHandle context, [MarshalAs(UnmanagedType.LPUTF8Str)] string path, [MarshalAs(UnmanagedType.LPUTF8Str)] string fileScale);
            /// <summary>在会话锁内卸载三维模型并清空结果缓存，保留图像及配置，不销毁会话。</summary>
            /// <param name="context">有效的原生会话，SafeHandle 在同步调用期间保持其存活；无模型时允许重复调用。</param>
            /// <returns>0 成功；非零由上层 Check 转为包含原生错误信息的异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmDisposeModel(NativeHandle context);
            /// <summary>以当前深度、内参和范围生成模板并训练，用于快速验证流程。</summary>
            /// <param name="context">已设置有效深度和配置的会话。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmCreateModelFromDepth(NativeHandle context);
            /// <summary>同步执行 runModel 并缓存本次结果，失败时清除旧结果，避免误用上一次候选。</summary>
            /// <param name="context">已具备模型和深度的会话；耗时调用建议由 C# 后台任务执行。</param>
            /// <param name="detectorPath">可选 UTF-8 2D 模型路径，nullptr/空串表示纯 3D。</param>
            /// <param name="runtimeDirectory">2D DLL 目录，启用 2D 时非空，否则忽略。</param>
            /// <param name="kind">2D 任务，0=OBB、1=Seg、2=Det。</param>
            /// <param name="driver">0=ONNX、1=OpenVINO。</param>
            /// <param name="threshold">2D 置信度，[0,1]，纯 3D 忽略。</param>
            /// <param name="useGpu">0 CPU，1 GPU，按原 SDK 支持情况执行。</param>
            /// <param name="yolo26">0 YOLO11，1 YOLO26。</param>
            /// <param name="attitudeId">-1 接受全部类别，非负值按类别筛选。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmRunModel(NativeHandle context, [MarshalAs(UnmanagedType.LPUTF8Str)] string detectorPath, [MarshalAs(UnmanagedType.LPUTF8Str)] string runtimeDirectory, int kind, int driver, double threshold, int useGpu, int yolo26, int attitudeId);
            /// <summary>将缓存候选复制到调用方数组，容量不足时不写入任何候选。</summary>
            /// <param name="context">有效会话；同一业务流程的 run/copy 若由多个线程使用，调用方须整体同步。</param>
            /// <param name="matches">输出数组，capacity=0 时允许 nullptr 进行数量查询。</param>
            /// <param name="capacity">数组的 SmMatch 元素容量，不是字节数。</param>
            /// <param name="written">非空输出地址，接收需要的元素数量；成功时等于实际写入数。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmCopyMatches(NativeHandle context, [Out] Match[] matches, ulong capacity, out ulong written);
            /// <summary>将单个 runModel 模型位姿转换为最终抓手位姿/偏移，不依赖会话。</summary>
            /// <param name="matchPose">非空单个模型到相机的 abg pose。</param>
            /// <param name="config">非空且有效的示教与标定配置。</param>
            /// <param name="result">非空输出结构，由调用方分配。</param>
            /// <returns>C ABI 状态码，0 成功；非零由上层转换为异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmTransformPose(ref Pose matchPose, ref GraspConfig config, out GraspResult result);

            /// <summary>调用相对偏移导出入口，使用空机器人指针处理固定相机；全部输入仅在同步调用期间借用。</summary>
            /// <param name="matchPose">相机系目标偏移，Type=0、abg 度。</param>
            /// <param name="eyeInHand">固定相机分支传 0。</param>
            /// <param name="handEyePose">相机到基座位姿，与输入偏移同长度单位。</param>
            /// <param name="robotPose">固定相机传 IntPtr.Zero，原生不读取此指针。</param>
            /// <param name="relativePose">原生成功时返回的基座系相对偏移，调用方无需释放。</param>
            /// <returns>0 成功；非零由 Check 转为携带本线程错误信息的异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmTransformRelativePose(ref Pose matchPose, int eyeInHand, ref Pose handEyePose,
                                                               IntPtr robotPose, out Pose relativePose);

            /// <summary>调用同一相对偏移导出入口，借用有效机器人位姿处理眼在手上，不保留指针。</summary>
            /// <param name="matchPose">相机系目标偏移，Type=0、abg 度。</param>
            /// <param name="eyeInHand">眼在手上分支传 1。</param>
            /// <param name="handEyePose">相机到标定末端位姿，与其他输入同长度单位。</param>
            /// <param name="robotPose">拍照时标定末端到基座的位姿，调用期间有效。</param>
            /// <param name="relativePose">原生成功时返回的基座系相对偏移，调用方无需释放。</param>
            /// <returns>0 成功；非零由 Check 转为携带本线程错误信息的异常。</returns>
            [DllImport(Library, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
            internal static extern int SmTransformRelativePose(ref Pose matchPose, int eyeInHand, ref Pose handEyePose,
                                                               ref Pose robotPose, out Pose relativePose);

        }
    }
}
