using System;
using System.Linq;
using System.Windows;
using System.Windows.Forms.Integration;
using WpfTextBox = System.Windows.Controls.TextBox;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using WpfRobot.collision;
using static WpfRobot.simulation.RobotVisualization;

namespace WpfRobot.simulation
{
    /// <summary>
    /// WPF版本实时机器人仿真刷新入口。
    /// 
    /// 这个类作为机器人可视化的公共 SDK 调用入口：
    /// 1. 对外提供统一初始化接口；
    /// 2. 对外提供关节角更新接口；
    /// 3. 内部调用 RobotVisualization；
    /// 4. 其他窗口不需要直接操作 VTK。
    /// </summary>
    public class SimulationRealTime
    {
        private RobotVisualization robotVisualization;

        /// <summary>
        /// WPF中用于承载WinForms Panel的宿主控件。
        /// VTK最终会渲染到 WindowsFormsHost 内部的 WinForms Panel 上。
        /// </summary>
        private WindowsFormsHost hostControl;

        private bool isInitialized = false;

        /// <summary>
        /// 当前关节角，单位 degree。
        /// 默认 6 个关节。
        /// </summary>
        public double[] CurrentJointDeg { get; private set; } = new double[6]
        {
            90, -90, 90, -90, -90, 90
        };

        /// <summary>
        /// 判断是否已经初始化。
        /// </summary>
        public bool IsInitialized
        {
            get { return isInitialized; }
        }

        /// <summary>
        /// 获取底层 RobotVisualization。
        /// 如果需要高级操作，例如添加点、显示 mesh、切换轨迹，可以通过它访问。
        /// </summary>
        public RobotVisualization Visualization
        {
            get { return robotVisualization; }
        }

        /// <summary>
        /// 初始化机器人可视化。
        /// 
        /// hostControl 应该传入 WPF XAML 中的 WindowsFormsHost。
        /// 例如：
        /// simulationRealTime.Initialize(VtkRobotHost, "./ur7e_ply");
        /// </summary>
        public void Initialize(WindowsFormsHost hostControl, string modelFolder = "./ur7e_ply")
        {
            if (hostControl == null)
                throw new ArgumentNullException(nameof(hostControl));

            this.hostControl = hostControl;

            robotVisualization = new RobotVisualization(hostControl, modelFolder);
            robotVisualization.Initialize();

            isInitialized = true;

            // 初始化后显示默认姿态
            UpdateJointAngles(CurrentJointDeg);
        }

        /// <summary>
        /// 更新 6 个关节角。
        /// 单位 degree。
        /// </summary>
        public void UpdateJointAngles(
            double q1Deg,
            double q2Deg,
            double q3Deg,
            double q4Deg,
            double q5Deg,
            double q6Deg)
        {
            double[] joints = new double[]
            {
                q1Deg, q2Deg, q3Deg, q4Deg, q5Deg, q6Deg
            };

            UpdateJointAngles(joints);
        }

        /// <summary>
        /// 更新关节角。
        /// jointsDeg 必须是 6 个。
        /// 单位 degree。
        /// </summary>
        public void UpdateJointAngles(double[] jointsDeg)
        {
            EnsureInitialized();

            if (jointsDeg == null)
                throw new ArgumentNullException(nameof(jointsDeg));

            if (jointsDeg.Length != 6)
                throw new ArgumentException("jointsDeg 必须是 6 个关节角，单位 degree。");

            CurrentJointDeg = jointsDeg.ToArray();

            RunOnUIThread(() =>
            {
                robotVisualization.SetJointAngles(CurrentJointDeg, true);
            });
        }

        /// <summary>
        /// 从 WPF TextBox 读取 6 个关节角并更新显示。
        /// TextBox 中的数值单位为 degree。
        /// </summary>
        /// <summary>
        /// 从 WPF TextBox 读取 6 个关节角并更新显示。
        /// </summary>
        public bool UpdateJointAnglesFromTextBox(
            TextBox j1Text,
            TextBox j2Text,
            TextBox j3Text,
            TextBox j4Text,
            TextBox j5Text,
            TextBox j6Text)
        {
            if (!TryReadDouble(j1Text, out double q1)) return false;
            if (!TryReadDouble(j2Text, out double q2)) return false;
            if (!TryReadDouble(j3Text, out double q3)) return false;
            if (!TryReadDouble(j4Text, out double q4)) return false;
            if (!TryReadDouble(j5Text, out double q5)) return false;
            if (!TryReadDouble(j6Text, out double q6)) return false;

            UpdateJointAngles(q1, q2, q3, q4, q5, q6);

            return true;
        }

        /// <summary>
        /// 指定某一个关节增加 stepDeg。
        /// jointIndex 从 1 开始：
        /// 1 表示 q1，6 表示 q6。
        /// </summary>
        public void AddJointAngle(int jointIndex, double stepDeg)
        {
            EnsureInitialized();

            if (jointIndex < 1 || jointIndex > 6)
                throw new ArgumentException("jointIndex 必须在 1 到 6 之间。");

            double[] joints = CurrentJointDeg.ToArray();
            joints[jointIndex - 1] += stepDeg;

            UpdateJointAngles(joints);
        }
        /// <summary>
        /// 设置地面网格大小，单位 mm。
        /// </summary>
        /// <param name="gridSizeMm"></param>
        public void SetGridSize(double gridSizeMm)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetGridSize(gridSizeMm);
            });
        }
        /// <summary>
        /// 设置某一个关节角。
        /// jointIndex 从 1 开始：
        /// 1 表示 q1，6 表示 q6。
        /// </summary>
        public void SetJointAngle(int jointIndex, double angleDeg)
        {
            EnsureInitialized();

            if (jointIndex < 1 || jointIndex > 6)
                throw new ArgumentException("jointIndex 必须在 1 到 6 之间。");

            double[] joints = CurrentJointDeg.ToArray();
            joints[jointIndex - 1] = angleDeg;

            UpdateJointAngles(joints);
        }

        /// <summary>
        /// 强制刷新渲染窗口。
        /// </summary>
        public void Render()
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.Render();
            });
        }
        public void SetAxisVisible(bool visible)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetAxisVisible(visible);
            });
        }

        public void SetGridVisible(bool visible)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetGridVisible(visible);
            });
        }

        public void SetWorkspaceVisible(bool visible)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetWorkspaceVisible(visible);
            });
        }

        public void SetTargetVisible(bool visible)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetTargetVisible(visible);
            });
        }

        public void SetJointAxisVisible(bool visible)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetJointAxisVisible(visible);
            });
        }

        public void ApplyDisplayOptions(
            bool showAxis,
            bool showGrid,
            bool showWorkspace,
            bool showTarget,
            bool showJointAxis)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetDisplayOptions(
                    showAxis,
                    showGrid,
                    showWorkspace,
                    showTarget,
                    showJointAxis
                );
            });
        }

        public void SetTargetPoint(double x, double y, double z)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetTargetPoint(x, y, z);
            });
        }
        /// <summary>
        /// 切换机器人显示 / 隐藏。
        /// </summary>
        public void ToggleRobotVisibility()
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.ToggleRobotVisibility();
            });
        }

        /// <summary>
        /// 切换轨迹显示 / 隐藏。
        /// </summary>
        public void ToggleTrajectoryVisibility()
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.ToggleTrajectoryVisibility();
            });
        }

        /// <summary>
        /// 添加一个轨迹点。
        /// </summary>
        public void AddTrajectoryPoint(double x, double y, double z)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.AddTrajectoryPoint(x, y, z, true);
            });
        }

        /// <summary>
        /// 添加一个轨迹点，并显示球。
        /// </summary>
        public void AddTrajectoryPointWithSphere(
            double x,
            double y,
            double z,
            double radius = 4.0,
            double[] color = null)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.AddTrajectoryPointWithSphere(x, y, z, radius, color, true);
            });
        }

        /// <summary>
        /// 清空轨迹。
        /// </summary>
        public void ClearTrajectory()
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.ClearTrajectory(true);
            });
        }

        /// <summary>
        /// 清空球形标记。
        /// </summary>
        public void ClearSpheres()
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.ClearSpheres(true);
            });
        }

        /// <summary>
        /// 释放底层可视化资源。
        /// 窗口关闭时可以调用。
        /// </summary>
        public void Dispose()
        {
            if (robotVisualization != null)
            {
                robotVisualization.Dispose();
                robotVisualization = null;
            }

            hostControl = null;
            isInitialized = false;
        }

        /// <summary>
        /// 判断是否已经初始化。
        /// </summary>
        public void EnsureInitialized()
        {
            if (!isInitialized || robotVisualization == null)
                throw new InvalidOperationException("SimulationRealTime 尚未初始化，请先调用 Initialize(...)。");
        }

        /// <summary>
        /// 安全读取 WPF TextBox 中的 double。
        /// </summary>
        private bool TryReadDouble(TextBox textBox, out double value)
        {
            value = 0.0;

            if (textBox == null)
            {
                System.Windows.MessageBox.Show("TextBox 为空。");
                return false;
            }

            if (!double.TryParse(textBox.Text, out value))
            {
                System.Windows.MessageBox.Show("请输入有效数字：" + textBox.Name);
                return false;
            }

            return true;
        }

        /// <summary>
        /// 确保在 WPF UI 线程执行渲染更新。
        /// </summary>
        private void RunOnUIThread(Action action)
        {
            if (hostControl == null)
            {
                action();
                return;
            }

            if (!hostControl.Dispatcher.CheckAccess())
            {
                hostControl.Dispatcher.Invoke(action);
            }
            else
            {
                action();
            }
        }
        /// <summary>
        /// 设置相机视角模式。
        /// </summary>
        /// <param name="mode"></param>
        public void SetCameraView(RobotVisualization.CameraViewMode mode)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetCameraView(mode);
            });
        }
        /// <summary>
        /// 旋转还是平移相机。
        /// </summary>
        /// <param name="mode"></param>
        public void SetMouseInteractionMode(RobotVisualization.MouseInteractionMode mode)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetMouseInteractionMode(mode);
            });
        }
        public void PanCameraByPixels(int dx, int dy)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.PanCameraByPixels(dx, dy);
            });
        }
        /// <summary>
        /// 缩放相机，zoomFactor > 1 表示放大，0 < zoomFactor < 1 表示缩小。
        /// </summary>
        /// <param name="zoomFactor"></param>
        public void ZoomCamera(double zoomFactor)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.ZoomCamera(zoomFactor);
            });
        }

        //public void UpdateJointFramesByTransforms(IList<Matrix<double>> transforms)
        //{
        //    EnsureInitialized();

        //    RunOnUIThread(() =>
        //    {
        //        robotVisualization.UpdateJointFramesByTransforms(transforms);
        //    });
        //}

        public void SetCollisionDetectionEnabled(bool enabled)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetCollisionDetectionEnabled(enabled);
            });
        }

        public CollisionReport CheckCollisionNow()
        {
            EnsureInitialized();

            CollisionReport report = CollisionReport.Empty();

            RunOnUIThread(() =>
            {
                report = robotVisualization.UpdateCollisionState(false);
            });

            return report;
        }

        public void SetCollisionBoundsVisible(bool visible)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetCollisionBoundsVisible(visible);
            });
        }

        /// <summary>
        /// 碰撞检测角度制输入，输出碰撞报告
        /// </summary>
        /// <param name="jointsDeg"></param>
        /// <returns></returns>
        public CollisionReport CheckCollisionForJointAngles(double[] jointsDeg)
        {
            EnsureInitialized();

            CollisionReport report = CollisionReport.Empty();

            RunOnUIThread(() =>
            {
                report = robotVisualization.CheckCollisionForJointAngles(
                    jointsDeg,
                    true
                );
            });

            return report;
        }

        public TrajectoryCollisionResult CheckTrajectoryCollision( IList<double[]> jointPath,  int interpolationCountPerSegment = 5)
        {
            EnsureInitialized();

            TrajectoryCollisionResult result = new TrajectoryCollisionResult();

            RunOnUIThread(() =>
            {
                result = robotVisualization.CheckTrajectoryCollision(
                    jointPath,
                    interpolationCountPerSegment
                );
            });

            return result;
        }
        public void SetRcmPoint(double x, double y, double z)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetRcmPoint(x, y, z);
            });
        }

        public void SetRcmPointVisible(bool visible)
        {
            EnsureInitialized();

            RunOnUIThread(() =>
            {
                robotVisualization.SetRcmPointVisible(visible);
            });
        }
    }
}