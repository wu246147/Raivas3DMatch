using System;
using System.Runtime.InteropServices;
using SurfaceMatch.Interop;

/// <summary>从真实 .NET Framework 4.8 进程验证 C ABI 布局、缓冲区复制、HALCON 匹配及抓手变换。</summary>
internal static class InteropSmoke
{
    /// <summary>失败即抛出具有中文场景说明的异常，Release 构建也执行。</summary>
    /// <param name="condition">必须成立的条件。</param>
    /// <param name="message">失败场景。</param>
    /// <exception cref="InvalidOperationException">条件为 false。</exception>
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }

    /// <summary>通过真实 DLL 验证四参数接口的空机器人指针、眼在手上封送、杠杆臂和错误传播。</summary>
    /// <exception cref="InvalidOperationException">实际结果、参数分支或原生错误传播不符合预期。</exception>
    /// <remarks>不创建会话或操纵机器人，预期结果使用独立的平面旋转几何关系计算。</remarks>
    private static void TestRelativeGrasp()
    {
        var delta = SurfaceMatchSession.TransformPose(new Pose { X = 10, Y = 20, Z = 30 }, false,
                                                     new Pose { X = 100, Y = 200, Rz = 90 }, null);
        Require(Math.Abs(delta.X + 20) < 1e-8 && Math.Abs(delta.Y - 10) < 1e-8 && Math.Abs(delta.Z - 30) < 1e-8,
                "C# 固定相机的 null 机器人指针及旋转换算错误");
        delta = SurfaceMatchSession.TransformPose(new Pose { Rz = 90 }, false, new Pose { X = 100 });
        Require(Math.Abs(delta.X - 100) < 1e-8 && Math.Abs(delta.Y + 100) < 1e-8,
                "C# 相机旋转的杠杆臂补偿错误");
        delta = SurfaceMatchSession.TransformPose(new Pose { Rz = 90 }, true, new Pose { X = 10 },
                                                 new Pose { X = 100, Y = 20, Rz = 90 });
        Require(Math.Abs(delta.X - 130) < 1e-8 && Math.Abs(delta.Y + 70) < 1e-8,
                "C# 眼在手上的机器人 pose 封送或组合顺序错误");
        delta = SurfaceMatchSession.TransformPose(new Pose(), false, new Pose { X = 100, Rz = 90 },
                                                 new Pose { X = double.NaN });
        Require(Math.Abs(delta.X) < 1e-8 && Math.Abs(delta.Y) < 1e-8, "固定相机应忽略传入的机器人位姿");
        bool rejected = false;
        try { SurfaceMatchSession.TransformPose(new Pose(), true, new Pose(), null); }
        catch (ArgumentException) { rejected = true; }
        Require(rejected, "C# 眼在手上缺失机器人位姿必须明确报错");
        rejected = false;
        try { SurfaceMatchSession.TransformPose(new Pose(), false, new Pose { X = double.NaN }); }
        catch (InvalidOperationException error) { rejected = error.Message.Contains("有限"); }
        Require(rejected, "C# 必须保留原生位姿错误说明");
    }

    /// <summary>创建合成深度验证原生建模、模型独立卸载和重用、两种抓手接口、错误传播与会话释放限制。</summary>
    /// <returns>全部通过返回 0，异常返回 1。</returns>
    private static int Main()
    {
        try
        {
            Require(Marshal.SizeOf(typeof(Pose)) == 56 && Marshal.SizeOf(typeof(Config)) == 168 &&
                    Marshal.SizeOf(typeof(Match)) == 72 && Marshal.SizeOf(typeof(GraspConfig)) == 256 &&
                    Marshal.SizeOf(typeof(GraspResult)) == 224, "C# 结构封送尺寸不正确");
            var session = new SurfaceMatchSession();
            using (session)
            {
                session.DisposeModel(); // 空会话可卸载模型，仍保留会话可用。
                var config = SurfaceMatchSession.DefaultConfig;
                config.Fx = config.Fy = 220; config.Cx = 80; config.Cy = 60; config.NumMatches = 1;
                session.SetConfig(config);
                bool rejected = false;
                try { session.LoadImages("不存在的中文深度图.tiff"); }
                catch (InvalidOperationException ex) { rejected = ex.Message.Contains("不存在的中文深度图"); }
                Require(rejected, "原生 UTF-8 文件错误必须完整传播到 C#");
                float[] depth = new float[160 * 120];
                for (int y = 15; y < 105; ++y)
                    for (int x = 20; x < 140; ++x)
                        depth[y * 160 + x] = (float)(400 + 45 * Math.Exp(-((x - 65.0) * (x - 65) / 180 + (y - 45.0) * (y - 45) / 120))
                            + 28 * Math.Exp(-((x - 112.0) * (x - 112) / 80 + (y - 82.0) * (y - 82) / 200)) + (x < 48 && y > 65 ? 32 : 0) + 0.15 * x + 0.04 * y);
                byte[] pixels = new byte[depth.Length * 4]; Buffer.BlockCopy(depth, 0, pixels, 0, pixels.Length);
                session.SetImage(pixels, 160, 120, 160 * 4, PixelFormat.DepthF32, true);
                Array.Clear(pixels, 0, pixels.Length); // 原生数据必须已独立复制，托管输入改变后仍应能匹配。
                session.CreateModelFromDepth();
                var matches = session.RunModel();
                Require(matches.Length == 1 && matches[0].Score > 0.6, "C# 到原生的真实 HALCON 匹配失败");
                Require(Math.Abs(matches[0].Pose.X) < 1 && Math.Abs(matches[0].Pose.Y) < 1, "C# 读取到的位姿应接近单位变换");
                session.DisposeModel();
                session.DisposeModel();
                Require(matches.Length == 1 && matches[0].Score > 0.6, "卸载不能破坏已复制的托管候选数组");
                rejected = false;
                try { session.RunModel(); }
                catch (InvalidOperationException error) { rejected = error.Message.Contains("SurfaceMatch (3)"); }
                Require(rejected, "卸载模型后运行必须报告缺少模型");
                session.CreateModelFromDepth(); // 不重新设置参数和图像，验证卸载后保留的数据可直接复用。
                matches = session.RunModel();
                Require(matches.Length == 1 && matches[0].Score > 0.6, "模型卸载后应能使用原深度重新建模匹配");
                rejected = false;
                try { session.SetImage(new byte[4], 160, 120, 160 * 4, PixelFormat.DepthF32, true); }
                catch (InvalidOperationException) { rejected = true; }
                Require(rejected, "过小像素缓冲区必须拒绝，不能越界读取");
            }
            bool disposed = false; try { session.CreateModelFromDepth(); } catch (ObjectDisposedException) { disposed = true; }
            Require(disposed, "释放后不得继续使用会话");
            disposed = false; try { session.DisposeModel(); } catch (ObjectDisposedException) { disposed = true; }
            Require(disposed, "会话销毁后 DisposeModel 必须拒绝调用");
            var grasp = new GraspConfig { CameraInBase = new Pose { X = 100 }, TaughtModelInBase = new Pose { X = 100 },
                TaughtTcpInBase = new Pose { X = 110 }, OffsetReferenceInBase = new Pose { X = 110 }, Mode = 2 };
            var result = SurfaceMatchSession.TransformPose(new Pose { Rz = 90 }, grasp);
            Require(Math.Abs(result.OutputPose.X + 10) < 1e-8 && Math.Abs(result.OutputPose.Y - 10) < 1e-8, "C# 抓手偏移转换错误");
            TestRelativeGrasp();
            Console.WriteLine("PASS .NET Framework 4.8 x64：结构布局、UTF-8、内存所有权、模型卸载重用、HALCON runModel、抓手转换与 SafeHandle");
            return 0;
        }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }
}
