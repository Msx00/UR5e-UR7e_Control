using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net.Sockets;
using WpfRobot.simulation;
using WpfRobot.inquiry;
using System.Windows.Threading;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace WpfRobot
{
    public static class simulation_mode
    {
        public enum SimulationDriveMode
        {
            TargetCommand,
            ActualRobot
        }

        public static SimulationDriveMode _simulationDriveMode = SimulationDriveMode.TargetCommand;

        public static bool ShowAxis = true;
        public static bool ShowGrid = true;
        public static bool ShowWorkspace = true;
        public static bool ShowTarget = true;
        public static bool ShowJointAxis = false;

        public static double GridSizeMm = 100.0;


        /// <summary>
        /// 碰撞检测
        /// </summary>
        public static bool CollisionDetectionEnabled = false;

        public static void RefreshSimulationByCurrentMode()
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            if (_simulationDriveMode == SimulationDriveMode.TargetCommand)
            {
                global_variable.simulationRealTime.UpdateJointAngles(
                    global_variable.globalJointDeg
                );
            }
            else
            {
                global_variable.simulationRealTime.UpdateJointAngles(
                    global_variable._actualJointDegForTable
                );
            }

            ApplyGridSize();
            ApplyDisplayOptions();

            global_variable.simulationRealTime.Render();
        }

        public static void SetDisplayOptions(
            bool showAxis,
            bool showGrid,
            bool showWorkspace,
            bool showTarget,
            bool showJointAxis)
        {
            ShowAxis = showAxis;
            ShowGrid = showGrid;
            ShowWorkspace = showWorkspace;
            ShowTarget = showTarget;
            ShowJointAxis = showJointAxis;

            ApplyDisplayOptions();
        }

        public static void ApplyDisplayOptions()
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.ApplyDisplayOptions(
                ShowAxis,
                ShowGrid,
                ShowWorkspace,
                ShowTarget,
                ShowJointAxis
            );
        }

        public static void SetGridSize(double gridSizeMm)
        {
            if (gridSizeMm <= 0)
                return;

            GridSizeMm = gridSizeMm;

            ApplyGridSize();
        }

        public static void ApplyGridSize()
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.SetGridSize(GridSizeMm);
        }

        public static void SetTargetPoint(double x, double y, double z)
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.SetTargetPoint(x, y, z);
        }
        public static void SetCameraView(string viewModeText)
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            WpfRobot.simulation.RobotVisualization.CameraViewMode mode =
                WpfRobot.simulation.RobotVisualization.CameraViewMode.Free;

            switch (viewModeText)
            {
                case "正视":
                    mode = WpfRobot.simulation.RobotVisualization.CameraViewMode.Front;
                    break;

                case "俯视":
                    mode = WpfRobot.simulation.RobotVisualization.CameraViewMode.Top;
                    break;

                case "侧视":
                    mode = WpfRobot.simulation.RobotVisualization.CameraViewMode.Side;
                    break;

                case "自由":
                default:
                    mode = WpfRobot.simulation.RobotVisualization.CameraViewMode.Free;
                    break;
            }

            global_variable.simulationRealTime.SetCameraView(mode);
        }
        public static void SetRotateMode()
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.SetMouseInteractionMode(
                WpfRobot.simulation.RobotVisualization.MouseInteractionMode.Rotate
            );
        }

        public static void SetPanMode()
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.SetMouseInteractionMode(
                WpfRobot.simulation.RobotVisualization.MouseInteractionMode.Pan
            );
        }

        public static void ZoomIn()
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.ZoomCamera(1.2);
        }

        public static void ZoomOut()
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.ZoomCamera(1.0 / 1.2);
        }
        public static void PanCameraByPixels(int dx, int dy)
        {
            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.PanCameraByPixels(dx, dy);
        }

        /// <summary>
        /// 碰撞检测
        /// </summary>
        public static void SetCollisionDetectionEnabled(bool enabled)
        {
            CollisionDetectionEnabled = enabled;

            if (global_variable.simulationRealTime == null ||
                !global_variable.simulationRealTime.IsInitialized)
            {
                return;
            }

            global_variable.simulationRealTime.SetCollisionDetectionEnabled(enabled);
        }
    }

    public static class global_variable
    {
        public static SimulationRealTime simulationRealTime = new SimulationRealTime();

        public static double[] _actualJointDegForTable = new double[6]
        {
            90, -90, 90, -90, -90, 90
        }; //根据编码器反馈，电机的真实角度，单位 degree

        public static double[] _targetJointDegForTable = new double[6]
        {
            90, -90, 90, -90, -90, 90
        };

        // 全局关节角，单位 degree
        // [q1, q2, q3, q4, q5, q6]
        public static double[] globalJointDeg = new double[6]
        {
            90, -90, 90, -90, -90, 90
        };

        // 工具 TCP 相对于第七轴输出坐标系的位置，单位 mm
        public static double[] globalToolVector = new double[3]
        {
            0, 0, 503
        };

        // 工具 TCP 相对于第七轴输出坐标系的姿态，单位 degree
        public static double globalToolRxDeg = 0.0;
        public static double globalToolRyDeg = 0.0;
        public static double globalToolRzDeg = 0.0;

        public static void SetGlobalJointDeg(
            double q1, double q2, double q3,
            double q4, double q5, double q6)
        {
            globalJointDeg[0] = q1;
            globalJointDeg[1] = q2;
            globalJointDeg[2] = q3;
            globalJointDeg[3] = q4;
            globalJointDeg[4] = q5;
            globalJointDeg[5] = q6;
        }

        public static void SetGlobalJointDeg(double[] jointsDeg)
        {
            if (jointsDeg == null || jointsDeg.Length < 6)
                return;

            SetGlobalJointDeg(
                jointsDeg[0],
                jointsDeg[1],
                jointsDeg[2],
                jointsDeg[3],
                jointsDeg[4],
                jointsDeg[5]
            );
        }
        public static void SetCustomJointDeg()
        {
            SetGlobalJointDeg(
                90, -90, 90, -90, -90, 90
            );
        }
        public static void SetDefaultJointDeg()
        {
            SetGlobalJointDeg(
                0, 0, 0, 0, 0, 0
            );
            //SetGlobalJointDeg(
            //    -91.71, -98.96, -126.22, -46.39, 91.39, -1.78, 0
            //);
        }

        public static void SetGlobalToolVector(double x, double y, double z)
        {
            globalToolVector[0] = x;
            globalToolVector[1] = y;
            globalToolVector[2] = z;
        }

        public static void SetGlobalTool(
            double x, double y, double z,
            double rxDeg = 0.0,
            double ryDeg = 0.0,
            double rzDeg = 0.0)
        {
            globalToolVector[0] = x;
            globalToolVector[1] = y;
            globalToolVector[2] = z;

            globalToolRxDeg = rxDeg;
            globalToolRyDeg = ryDeg;
            globalToolRzDeg = rzDeg;
        }

        public static void SyncToolToForward()
        {
            // 这里先保留为空。
            // 后面如果你的 Forward 正运动学类直接读取 global_variable.globalToolVector，
            // 那这里不需要做任何事。
        }


        public static Matrix<double> globalT06 = DenseMatrix.CreateIdentity(4);
        public static Matrix<double> globalT07 = DenseMatrix.CreateIdentity(4);
        public static Matrix<double> globalT0Tcp = DenseMatrix.CreateIdentity(4);

        public static bool globalFkValid = false;
    }
}
