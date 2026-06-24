using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows.Forms.Integration;
using Forms = System.Windows.Forms;
using Kitware.VTK;
using System.Runtime.InteropServices;
using MathNet.Numerics.LinearAlgebra;
using WpfRobot.kinematics;
using WpfRobot.collision;
using CollisionDetector = WpfRobot.collision.collision;

namespace WpfRobot.simulation
{
    public class RobotVisualization : IDisposable
    {
        public double[] background_color = new double[3] { 1, 1, 1 };
        private vtkRenderWindow renderWindow;
        private vtkRenderer renderer;
        private vtkRenderWindowInteractor interactor;
        private vtkInteractorStyleTrackballCamera style;

        /// <summary>
        /// 当前鼠标交互模式：旋转或平移。
        /// </summary>
        private MouseInteractionMode mouseInteractionMode = MouseInteractionMode.Rotate;
        private vtkInteractorStyleUser panStyle = null;
        private bool isPanning = false;
        private System.Drawing.Point lastMousePoint;
        private int lastVtkMouseX = 0;
        private int lastVtkMouseY = 0;

        private readonly WindowsFormsHost wpfHost;
        private Forms.Panel hostPanel;

        private readonly List<vtkAssembly> urAssemblies = new List<vtkAssembly>();

        private vtkPlaneSource gridPlaneSource = null;
        private double currentGridSizeMm = 100.0;
        private const double GroundTotalSizeMm = 2000.0;

        private vtkActor gridActor = null;
        private vtkAxesActor axesActor = null;
        private vtkActor workspaceActor = null;
        private vtkActor targetPointActor = null;
        private vtkActor rcmPointActor = null;
        private vtkSphereSource rcmPointSphereSource = null;
        public bool IsRcmPointVisible { get; private set; } = true;

        //趣味六面体，指明方向
        // 右下角方向立方体 / 坐标方向指示器
        private vtkOrientationMarkerWidget orientationMarkerWidget = null;
        private vtkAnnotatedCubeActor orientationCubeActor = null;
        private vtkAxesActor orientationAxesActor = null;
        private vtkPropAssembly orientationMarkerAssembly = null;
        public bool IsOrientationCubeVisible { get; private set; } = true;


        // =====================================================
        // 碰撞检测
        // =====================================================
        private readonly List<vtkActor> urLinkActors = new List<vtkActor>();
        private readonly List<string> urLinkNames = new List<string>();

        private readonly CollisionDetector collisionDetector = new CollisionDetector();

        private readonly List<vtkActor> collisionDebugBoxActors = new List<vtkActor>();
        private bool showCollisionDebugBoxes = false;

        public CollisionReport LastCollisionReport { get; private set; } = CollisionReport.Empty();

        public bool IsCollisionDetectionEnabled
        {
            get { return collisionDetector.Enabled; }
        }
        private double[] currentJointDegForCollision = new double[6]; //当前关节角缓存 //用于路径规划打的碰撞检测，使用静默方式，不更新UI
        public class TrajectoryCollisionResult
        {
            public bool HasCollision { get; set; }

            public int SegmentIndex { get; set; } = -1;

            public double Alpha { get; set; } = 0.0;

            public double[] CollisionJointDeg { get; set; }

            public CollisionReport Report { get; set; } = CollisionReport.Empty();

            public string Summary
            {
                get
                {
                    if (!HasCollision)
                        return "路径未检测到碰撞";

                    return $"路径在第 {SegmentIndex} 段 alpha={Alpha:F2} 处发生碰撞：{Report.Summary}";
                }
            }
        }

        /// <summary>
        /// 显示关节坐标轴
        /// </summary>
        private vtkAssembly jointAxisAssembly = null;
        //private readonly List<vtkAxesActor> jointFrameAxesActors = new List<vtkAxesActor>();
        private readonly List<vtkLineSource[]> jointFrameAxisSources = new List<vtkLineSource[]>();
        private readonly List<vtkActor[]> jointFrameAxisActors = new List<vtkActor[]>();
        private readonly List<vtkLineSource> jointAxisLineSources = new List<vtkLineSource>();
        private readonly List<vtkActor> jointAxisLineActors = new List<vtkActor>();
        private const double JointFrameAxisLength = 90.0;
        private const double JointAxisLineLength = 160.0;

        public enum CameraViewMode
        {
            Free,
            Front,
            Top,
            Side
        }
        public enum MouseInteractionMode
        {
            Rotate,
            Pan
        }
        public bool IsAxisVisible { get; private set; } = true;
        public bool IsGridVisible { get; private set; } = true;
        public bool IsWorkspaceVisible { get; private set; } = true;
        public bool IsTargetVisible { get; private set; } = true;

        /// <summary>
        /// 显示机械臂关节坐标系
        /// </summary>
        public bool IsJointAxisVisible { get; private set; } = false;
        private double[] lastJointDeg = new double[6]
        {
            90, -90, 90, -90, -90, 90
        };

        private vtkActor trajectoryActor = null;
        private readonly List<vtkActor> sphereActors = new List<vtkActor>();

        private vtkPoints trajectoryPointsVTK;
        private vtkPolyLine trajectoryPolyLine;
        private vtkCellArray trajectoryCells;
        private vtkPolyData trajectoryPolyData;
        private vtkPolyDataMapper trajectoryMapper;

        private bool initialized = false;

        public bool IsRobotVisible { get; private set; } = true;
        public bool IsTrajectoryVisible { get; private set; } = true;

        public vtkAssembly PassiveAssembly { get; private set; } = null;

        /// <summary>
        /// PLY 文件所在目录。
        /// 默认使用 ./ur7e_ply
        /// </summary>
        public string ModelFolder { get; set; } = "./ur7e_ply";

        /// <summary>
        /// UR 六轴模型的固定相对位姿。
        /// 格式：
        /// {x, y, z, rotateX, rotateY, rotateZ}
        /// 单位：
        /// x/y/z 为 mm，角度为 degree。
        /// </summary>
        private readonly double[,] jointPosesUR7e = new double[,]
        {
            {0, 0, 0, 0, 0, 90},
            {0, 0, 162.5, 90, 90, 0},
            {0, 0, 137.8, 90, 0, 180},
            {0, 131.2, -425, 90, 0, 0},
            {-392.2, 0, 126.7, 90, 180, 90},
            {0, 0, -99.7, 90, 90, 90},
            {0, 0, -99.6, 0, 0, 0},
            {0, 0, 0, -180, 0, 90},
            {115, 0, 174, 90, -90, 180},
            {68.284, 36.643, 164.973, -0.46, 2.24, 2.93},
        };

        /// <summary>
        /// WPF版本构造函数。
        /// 传入 XAML 中的 WindowsFormsHost。
        /// </summary>
        public RobotVisualization(WindowsFormsHost wpfHost, string modelFolder = "./ur7e_ply")
        {
            this.wpfHost = wpfHost ?? throw new ArgumentNullException(nameof(wpfHost));
            this.ModelFolder = modelFolder;
        }
       

        private void Interactor_LeftButtonPressEvt(vtkObject sender, vtkObjectEventArgs e)
        {
            if (mouseInteractionMode != MouseInteractionMode.Pan)
                return;

            int[] pos = interactor.GetEventPosition();

            lastVtkMouseX = pos[0];
            lastVtkMouseY = pos[1];

            isPanning = true;
        }

        private void Interactor_MouseMoveEvt(vtkObject sender, vtkObjectEventArgs e)
        {
            if (mouseInteractionMode != MouseInteractionMode.Pan)
                return;

            if (!isPanning)
                return;

            int[] pos = interactor.GetEventPosition();

            int currentX = pos[0];
            int currentY = pos[1];

            int dx = currentX - lastVtkMouseX;
            int dy = currentY - lastVtkMouseY;

            lastVtkMouseX = currentX;
            lastVtkMouseY = currentY;

            PanCameraByPixels(dx, dy);
        }

        private void Interactor_LeftButtonReleaseEvt(vtkObject sender, vtkObjectEventArgs e)
        {
            if (mouseInteractionMode != MouseInteractionMode.Pan)
                return;

            isPanning = false;
        }
        /// <summary>
        /// 初始化 VTK 环境、加载机器人模型、添加地面和坐标轴。
        /// </summary>
        public void Initialize()
        {
            if (initialized)
                return;

            CreateHostPanel();

            renderWindow = vtkRenderWindow.New();

            // WPF没有Handle，所以这里使用 WindowsFormsHost 内部的 WinForms Panel.Handle
            renderWindow.SetParentId(hostPanel.Handle);
            renderWindow.SetSize(
                Math.Max(1, hostPanel.ClientSize.Width),
                Math.Max(1, hostPanel.ClientSize.Height)
            );

            renderer = vtkRenderer.New();
            renderWindow.AddRenderer(renderer);

            interactor = vtkRenderWindowInteractor.New();
            interactor.SetRenderWindow(renderWindow);

            style = vtkInteractorStyleTrackballCamera.New();
            interactor.SetInteractorStyle(style);
            interactor.LeftButtonPressEvt += Interactor_LeftButtonPressEvt;
            interactor.MouseMoveEvt += Interactor_MouseMoveEvt;
            interactor.LeftButtonReleaseEvt += Interactor_LeftButtonReleaseEvt;

            LoadUR7eModel();

            //renderer.AddActor(CreateGround());
            //renderer.AddActor(CreateAxes());
            gridActor = CreateGround(currentGridSizeMm); 
            axesActor = CreateAxes();

            workspaceActor = CreateWorkspaceSphere(850.0);
            //红色小球
            targetPointActor = CreateTargetPointActor(0, 0, 0);
            // 新增：RCM点，蓝色小球
            rcmPointActor = CreateRcmPointActor(0, 0, 0);

            jointAxisAssembly = CreateJointAxisAssembly();

            renderer.AddActor(gridActor);
            renderer.AddActor(axesActor);
            renderer.AddActor(workspaceActor);
            renderer.AddActor(targetPointActor);
            renderer.AddActor(rcmPointActor);
            renderer.AddActor(jointAxisAssembly);

            // 默认状态
            SetAxisVisible(true, false);
            SetGridVisible(true, false);
            SetWorkspaceVisible(true, false);
            SetTargetVisible(true, false);
            SetJointAxisVisible(false, false);

            renderer.SetBackground(background_color[0], background_color[1], background_color[2]);

            hostPanel.Resize += HostPanel_Resize;

            interactor.Initialize();

            // 右下角方向立方体
            CreateOrientationCubeMarker();

            vtkCamera camera = renderer.GetActiveCamera();
            camera.SetPosition(1, 1, 1);
            camera.SetFocalPoint(0, 0, 0);
            camera.SetViewUp(0, 0, 1);

            renderer.ResetCamera();
            renderer.ResetCameraClippingRange();

            InitializeTrajectoryLine();

            initialized = true;

            Render();
        }
        private vtkActor CreateRcmPointActor(double x, double y, double z)
        {
            rcmPointSphereSource = vtkSphereSource.New();

            // 球体几何中心放在原点，位置通过 actor.SetPosition 控制
            rcmPointSphereSource.SetCenter(0, 0, 0);
            rcmPointSphereSource.SetRadius(22.0);
            rcmPointSphereSource.SetThetaResolution(32);
            rcmPointSphereSource.SetPhiResolution(16);
            rcmPointSphereSource.Update();

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputConnection(rcmPointSphereSource.GetOutputPort());

            vtkActor actor = vtkActor.New();
            actor.SetMapper(mapper);

            // 蓝色 RCM 点
            actor.GetProperty().SetColor(0.0, 0.25, 1.0);
            actor.GetProperty().SetOpacity(1.0);
            actor.GetProperty().SetDiffuse(0.85);
            actor.GetProperty().SetSpecular(0.6);
            actor.GetProperty().SetSpecularPower(30.0);

            actor.SetPosition(x, y, z);

            return actor;
        }
        public void SetRcmPoint(double x, double y, double z, bool render = true)
        {
            if (rcmPointActor == null)
                return;

            rcmPointActor.SetPosition(x, y, z);

            if (render)
                Render();
        }

        public void SetRcmPointVisible(bool visible, bool render = true)
        {
            if (rcmPointActor == null)
                return;

            rcmPointActor.SetVisibility(visible ? 1 : 0);
            IsRcmPointVisible = visible;

            if (render)
                Render();
        }
        public void SetAxisVisible(bool visible, bool render = true)
        {
            if (axesActor == null)
                return;

            axesActor.SetVisibility(visible ? 1 : 0);
            IsAxisVisible = visible;

            if (render)
                Render();
        }

        public void SetGridVisible(bool visible, bool render = true)
        {
            if (gridActor == null)
                return;

            gridActor.SetVisibility(visible ? 1 : 0);
            IsGridVisible = visible;

            if (render)
                Render();
        }

        public void SetWorkspaceVisible(bool visible, bool render = true)
        {
            if (workspaceActor == null)
                return;

            workspaceActor.SetVisibility(visible ? 1 : 0);
            IsWorkspaceVisible = visible;

            if (render)
                Render();
        }

        public void SetTargetVisible(bool visible, bool render = true)
        {
            if (targetPointActor == null)
                return;

            targetPointActor.SetVisibility(visible ? 1 : 0);
            IsTargetVisible = visible;

            if (render)
                Render();
        }

        public void SetJointAxisVisible(bool visible, bool render = true)
        {
            if (jointAxisAssembly == null)
                return;

            IsJointAxisVisible = visible;
            jointAxisAssembly.SetVisibility(visible ? 1 : 0);

            // 关键：从隐藏切换到显示时，立即用当前关节角刷新一次 T01~T06
            if (visible)
            {
                UpdateJointFramesFromCurrentJoints(false);
            }

            if (render)
                Render();
        }

        /// <summary>
        /// 设置右下角方向立方体是否显示。
        /// </summary>
        public void SetOrientationCubeVisible(bool visible, bool render = true)
        {
            if (orientationMarkerWidget == null)
                return;

            orientationMarkerWidget.SetEnabled(visible ? 1 : 0);
            IsOrientationCubeVisible = visible;

            if (render)
                Render();
        }
        /// <summary>
        /// 创建右下角方向立方体。
        /// 类似机器人仿真软件右下角的 ViewCube / 方向指示器。
        /// </summary>
        private void CreateOrientationCubeMarker()
        {
            if (interactor == null)
                return;

            if (orientationMarkerWidget != null)
                return;

            // 1. 创建带文字的方向立方体
            orientationCubeActor = vtkAnnotatedCubeActor.New();

            // 如果中文显示乱码，可以把这些文字改成 "Right", "Left", "Front", "Back", "Up", "Down"
            //orientationCubeActor.SetXPlusFaceText("右");
            //orientationCubeActor.SetXMinusFaceText("左");
            //orientationCubeActor.SetYPlusFaceText("前");
            //orientationCubeActor.SetYMinusFaceText("后");
            //orientationCubeActor.SetZPlusFaceText("上");
            //orientationCubeActor.SetZMinusFaceText("下");

            //orientationCubeActor.SetXPlusFaceText("+X");
            //orientationCubeActor.SetXMinusFaceText("-X");
            //orientationCubeActor.SetYPlusFaceText("+Y");
            //orientationCubeActor.SetYMinusFaceText("-Y");
            //orientationCubeActor.SetZPlusFaceText("+Z");
            //orientationCubeActor.SetZMinusFaceText("-Z");

            orientationCubeActor.SetFaceTextScale(0.5);

            // 立方体主体颜色
            orientationCubeActor.GetCubeProperty().SetColor(0.92, 0.94, 0.96);
            orientationCubeActor.GetCubeProperty().SetOpacity(0.95);

            // 文字边缘颜色
            orientationCubeActor.GetTextEdgesProperty().SetColor(0.15, 0.15, 0.15);
            orientationCubeActor.GetTextEdgesProperty().SetLineWidth(1.0f);

            // 各个面的浅色区分
            orientationCubeActor.GetXPlusFaceProperty().SetColor(1.00, 0.88, 0.88);
            orientationCubeActor.GetXMinusFaceProperty().SetColor(0.96, 0.96, 0.96);

            orientationCubeActor.GetYPlusFaceProperty().SetColor(0.88, 1.00, 0.88);
            orientationCubeActor.GetYMinusFaceProperty().SetColor(0.96, 0.96, 0.96);

            orientationCubeActor.GetZPlusFaceProperty().SetColor(0.88, 0.92, 1.00);
            orientationCubeActor.GetZMinusFaceProperty().SetColor(0.96, 0.96, 0.96);

            // 2. 添加一个小坐标轴，让 X/Y/Z 更直观
            orientationAxesActor = vtkAxesActor.New();
            orientationAxesActor.SetShaftTypeToCylinder();
            orientationAxesActor.SetTotalLength(1.8, 1.8, 1.8);
            orientationAxesActor.SetCylinderRadius(0.05);
            orientationAxesActor.SetConeRadius(0.2);
            orientationAxesActor.SetSphereRadius(0.08);

            orientationAxesActor.GetXAxisCaptionActor2D().GetCaptionTextProperty().SetColor(1.0, 0.0, 0.0);
            orientationAxesActor.GetYAxisCaptionActor2D().GetCaptionTextProperty().SetColor(0.0, 0.75, 0.0);
            orientationAxesActor.GetZAxisCaptionActor2D().GetCaptionTextProperty().SetColor(0.0, 0.2, 1.0);

            orientationAxesActor.GetXAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(14);
            orientationAxesActor.GetYAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(14);
            orientationAxesActor.GetZAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(14);

            // 3. 把立方体和坐标轴组合起来
            orientationMarkerAssembly = vtkPropAssembly.New();
            orientationMarkerAssembly.AddPart(orientationCubeActor);
            orientationMarkerAssembly.AddPart(orientationAxesActor);

            // 4. 创建右下角方向控件
            orientationMarkerWidget = vtkOrientationMarkerWidget.New();
            orientationMarkerWidget.SetOrientationMarker(orientationMarkerAssembly);
            orientationMarkerWidget.SetInteractor(interactor);

            // 右下角位置，坐标范围是 0~1
            // 参数含义：xmin, ymin, xmax, ymax
            orientationMarkerWidget.SetViewport(0.80, 0.02, 0.98, 0.22);

            // 开启显示
            orientationMarkerWidget.SetEnabled(1);

            // 禁止用户拖动这个方向立方体，避免影响你自己的鼠标旋转/平移逻辑
            orientationMarkerWidget.InteractiveOff();

            IsOrientationCubeVisible = true;
        }

        public void SetDisplayOptions(
            bool showAxis,
            bool showGrid,
            bool showWorkspace,
            bool showTarget,
            bool showJointAxis)
        {
            SetAxisVisible(showAxis, false);
            SetGridVisible(showGrid, false);
            SetWorkspaceVisible(showWorkspace, false);
            SetTargetVisible(showTarget, false);
            SetJointAxisVisible(showJointAxis, false);

            Render();
        }

        public void SetTargetPoint(double x, double y, double z, bool render = true)
        {
            if (targetPointActor == null)
                return;

            targetPointActor.SetPosition(x, y, z);

            if (render)
                Render();
        }
        private void CreateHostPanel()
        {
            if (hostPanel != null)
                return;

            hostPanel = new Forms.Panel();
            hostPanel.Dock = Forms.DockStyle.Fill;
            hostPanel.BackColor = System.Drawing.Color.LightGray;


            wpfHost.Child = hostPanel;
        }
       
        private void HostPanel_MouseUp(object sender, Forms.MouseEventArgs e)
        {
            if (mouseInteractionMode != MouseInteractionMode.Pan)
                return;

            if (e.Button == Forms.MouseButtons.Left)
            {
                isPanning = false;
                hostPanel.Capture = false;
            }
        }
        
        private static double[] Cross(double[] a, double[] b)
        {
            return new double[]
            {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            };
        }

        private static double Norm(double[] v)
        {
            return Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        }

        private static void Normalize(double[] v)
        {
            double n = Norm(v);

            if (n < 1e-12)
                return;

            v[0] /= n;
            v[1] /= n;
            v[2] /= n;
        }
        public void SetMouseInteractionMode(MouseInteractionMode mode, bool render = true)
        {
            mouseInteractionMode = mode;

            if (interactor == null)
                return;

            if (mode == MouseInteractionMode.Rotate)
            {
                if (style == null)
                    style = vtkInteractorStyleTrackballCamera.New();

                interactor.SetInteractorStyle(style);

                isPanning = false;

                if (hostPanel != null)
                    hostPanel.Cursor = Forms.Cursors.Default;
            }
            else if (mode == MouseInteractionMode.Pan)
            {
                if (panStyle == null)
                    panStyle = vtkInteractorStyleUser.New();

                // 进入 Pan 模式时禁用默认 Trackball 左键旋转，
                // 左键事件由 Interactor_LeftButtonPressEvt / MouseMoveEvt 处理
                interactor.SetInteractorStyle(panStyle);

                isPanning = false;

                if (hostPanel != null)
                    hostPanel.Cursor = Forms.Cursors.Hand;
            }

            if (render)
                Render();
        }
        public void PanCameraByPixels(int dx, int dy)
        {
            if (renderer == null || hostPanel == null)
                return;

            vtkCamera camera = renderer.GetActiveCamera();

            double[] pos = camera.GetPosition();
            double[] focal = camera.GetFocalPoint();
            double[] viewUp = camera.GetViewUp();

            double[] viewDir = new double[]
            {
                focal[0] - pos[0],
                focal[1] - pos[1],
                focal[2] - pos[2]
            };

            Normalize(viewDir);
            Normalize(viewUp);

            double[] right = Cross(viewDir, viewUp);
            Normalize(right);

            double pixelToWorld;

            if (camera.GetParallelProjection() != 0)
            {
                pixelToWorld = 2.0 * camera.GetParallelScale() /
                               Math.Max(1, hostPanel.ClientSize.Height);
            }
            else
            {
                double[] fullViewDir = new double[]
                {
                    focal[0] - pos[0],
                    focal[1] - pos[1],
                    focal[2] - pos[2]
                };

                double distance = Norm(fullViewDir);
                double viewAngleRad = camera.GetViewAngle() * Math.PI / 180.0;

                pixelToWorld = 2.0 * distance * Math.Tan(viewAngleRad / 2.0) /
                               Math.Max(1, hostPanel.ClientSize.Height);
            }

            double moveRight = -dx * pixelToWorld; //左右移动
            double moveUp = -dy * pixelToWorld; //上下移动

            double[] delta = new double[]
            {
                right[0] * moveRight + viewUp[0] * moveUp,
                right[1] * moveRight + viewUp[1] * moveUp,
                right[2] * moveRight + viewUp[2] * moveUp
            };

            camera.SetPosition(
                pos[0] + delta[0],
                pos[1] + delta[1],
                pos[2] + delta[2]
            );

            camera.SetFocalPoint(
                focal[0] + delta[0],
                focal[1] + delta[1],
                focal[2] + delta[2]
            );

            renderer.ResetCameraClippingRange();
            Render();
        }

        public void ZoomCamera(double zoomFactor, bool render = true)
        {
            if (renderer == null)
                return;

            if (zoomFactor <= 0)
                return;

            vtkCamera camera = renderer.GetActiveCamera();

            camera.Zoom(zoomFactor);

            renderer.ResetCameraClippingRange();

            if (render)
                Render();
        }
        private void HostPanel_Resize(object sender, EventArgs e)
        {
            if (renderWindow == null || hostPanel == null)
                return;

            int width = Math.Max(1, hostPanel.ClientSize.Width);
            int height = Math.Max(1, hostPanel.ClientSize.Height);

            renderWindow.SetSize(width, height);
            Render();
        }

        /// <summary>
        /// 设置 6 个关节角并更新机器人模型。
        /// 单位：degree。
        /// 这个函数只负责更新机器人 PLY 模型本体，不负责更新关节坐标轴。
        /// </summary>
        public void SetJointAngles(
            double joint1Deg,
            double joint2Deg,
            double joint3Deg,
            double joint4Deg,
            double joint5Deg,
            double joint6Deg,
            bool render = true)
        {
            currentJointDegForCollision = new double[]
            {
                joint1Deg,
                joint2Deg,
                joint3Deg,
                joint4Deg,
                joint5Deg,
                joint6Deg
            };

            if (urAssemblies.Count < 8)
                return;

            urAssemblies[2].SetOrientation(0, -90 + joint1Deg, 0);
            urAssemblies[3].SetOrientation(0, -90 - joint2Deg, 0);
            urAssemblies[4].SetOrientation(0, 0, 90 + joint3Deg);
            urAssemblies[5].SetOrientation(joint4Deg, 0, 0);
            urAssemblies[6].SetOrientation(180 - joint5Deg, 0, 0);
            urAssemblies[7].SetOrientation(0, 0, 90 - joint6Deg);
            if (render)
                Render();
        }

        /// <summary>
        /// 碰撞检测函数，静默检测，路径规划使用，不更新UI，不显示包围盒。
        /// </summary>
        /// <param name="jointsDeg"></param>
        /// <param name="restoreCurrentPose"></param>
        /// <returns></returns>
        public CollisionReport CheckCollisionForJointAngles(
        double[] jointsDeg,
        bool restoreCurrentPose = true)
        {
            if (jointsDeg == null || jointsDeg.Length < 6)
                return CollisionReport.Empty();

            // 保存当前状态
            double[] oldJointDeg = currentJointDegForCollision != null
                ? currentJointDegForCollision.ToArray()
                : null;

            bool oldEnabled = collisionDetector.Enabled;
            bool oldShowDebugBoxes = showCollisionDebugBoxes;

            CollisionReport report = CollisionReport.Empty();

            try
            {
                // 路径规划检测时不显示包围盒、不变色、不渲染
                showCollisionDebugBoxes = false;

                // 先关闭自动碰撞刷新，避免 SetJointAngles 内部触发 UpdateCollisionState()
                collisionDetector.Enabled = false;

                // 静默设置候选姿态
                SetJointAngles(jointsDeg, false);

                // 临时开启 detector，只调用核心检测，不调用 ApplyVisualResult，不调用 Render
                collisionDetector.Enabled = true;

                report = collisionDetector.CheckSelfCollision(
                    urAssemblies,
                    urLinkActors,
                    urLinkNames
                );
            }
            finally
            {
                // 恢复原来的 VTK 姿态
                if (restoreCurrentPose && oldJointDeg != null)
                {
                    collisionDetector.Enabled = false;
                    SetJointAngles(oldJointDeg, false);
                }

                // 恢复原状态
                collisionDetector.Enabled = oldEnabled;
                showCollisionDebugBoxes = oldShowDebugBoxes;

                // 如果原来正在显示碰撞检测状态，则恢复显示
                if (oldEnabled)
                {
                    UpdateCollisionState(false);
                }
            }

            return report;
        }
        public TrajectoryCollisionResult CheckTrajectoryCollision(
    IList<double[]> jointPath,
    int interpolationCountPerSegment = 5)
        {
            TrajectoryCollisionResult result = new TrajectoryCollisionResult();

            if (jointPath == null || jointPath.Count == 0)
                return result;

            double[] oldJointDeg = currentJointDegForCollision != null
                ? currentJointDegForCollision.ToArray()
                : null;

            bool oldEnabled = collisionDetector.Enabled;
            bool oldShowDebugBoxes = showCollisionDebugBoxes;

            try
            {
                showCollisionDebugBoxes = false;
                collisionDetector.Enabled = false;

                // 先检测每个路径点本身
                for (int i = 0; i < jointPath.Count; i++)
                {
                    double[] q = jointPath[i];

                    if (q == null || q.Length < 6)
                        continue;

                    SetJointAngles(q, false);

                    collisionDetector.Enabled = true;

                    CollisionReport report = collisionDetector.CheckSelfCollision(
                        urAssemblies,
                        urLinkActors,
                        urLinkNames
                    );

                    collisionDetector.Enabled = false;

                    if (report.HasCollision)
                    {
                        result.HasCollision = true;
                        result.SegmentIndex = i;
                        result.Alpha = 0.0;
                        result.CollisionJointDeg = q.Take(6).ToArray();
                        result.Report = report;
                        return result;
                    }
                }

                // 再检测相邻路径点之间的插值姿态
                for (int i = 0; i < jointPath.Count - 1; i++)
                {
                    double[] q0 = jointPath[i];
                    double[] q1 = jointPath[i + 1];

                    if (q0 == null || q1 == null)
                        continue;

                    if (q0.Length < 6 || q1.Length < 6)
                        continue;

                    for (int k = 1; k <= interpolationCountPerSegment; k++)
                    {
                        double alpha = (double)k / (interpolationCountPerSegment + 1);

                        double[] q = InterpolateJointDeg(q0, q1, alpha);

                        SetJointAngles(q, false);

                        collisionDetector.Enabled = true;

                        CollisionReport report = collisionDetector.CheckSelfCollision(
                            urAssemblies,
                            urLinkActors,
                            urLinkNames
                        );

                        collisionDetector.Enabled = false;

                        if (report.HasCollision)
                        {
                            result.HasCollision = true;
                            result.SegmentIndex = i;
                            result.Alpha = alpha;
                            result.CollisionJointDeg = q;
                            result.Report = report;
                            return result;
                        }
                    }
                }
            }
            finally
            {
                if (oldJointDeg != null)
                {
                    collisionDetector.Enabled = false;
                    SetJointAngles(oldJointDeg, false);
                }

                collisionDetector.Enabled = oldEnabled;
                showCollisionDebugBoxes = oldShowDebugBoxes;

                if (oldEnabled)
                {
                    UpdateCollisionState(false);
                }
            }

            return result;
        }

        private static double[] InterpolateJointDeg(
            double[] q0,
            double[] q1,
            double alpha)
        {
            double[] q = new double[6];

            for (int i = 0; i < 6; i++)
            {
                q[i] = q0[i] * (1.0 - alpha) + q1[i] * alpha;
            }

            return q;
        }

        /// <summary>
        /// 碰撞检测，UI更新
        /// </summary>
        /// <param name="render"></param>
        public void SetCollisionDetectionEnabled(bool enabled, bool render = true)
        {
            collisionDetector.Enabled = enabled;

            collisionDetector.SafetyMarginMm = 0.0;

            // 忽略相邻两级连杆，避免关节连接处天然接触误报
            collisionDetector.IgnoreNeighborSpan = 2;

            // =====================================================
            // 自定义末端碰撞模型
            // =====================================================
            collisionDetector.UseCustomEndCollisionModel = false;

            // fixed_end.ply 现在要参与碰撞检测
            // 但它的碰撞 OBB 会被缩短
            collisionDetector.IgnoreFixedEndCollision = true;

            collisionDetector.Wrist3LinkName = "wrist3";
            collisionDetector.FixedEndLinkName = "fixed_end";
            collisionDetector.RotatedEndLinkName = "rotated_end";

            // wrist3 延长方向，你当前已经验证这个方向是正确的
            collisionDetector.Wrist3ExtendAxis = 2;
            collisionDetector.Wrist3ExtendPositiveMm = 0.0;
            collisionDetector.Wrist3ExtendNegativeMm = 174.0;

            // rotated_end 不缩短
            collisionDetector.RotatedEndShrinkAxis = 2;
            collisionDetector.RotatedEndCutPositiveMm = 0.0;
            collisionDetector.RotatedEndCutNegativeMm = 0.0;

            // fixed_end 沿同一方向缩短
            // 因为 wrist3 是 Negative 方向延长 174，
            // 所以 fixed_end 默认也从 Negative 方向切掉 174。
            collisionDetector.FixedEndShrinkAxis = 2;
            collisionDetector.FixedEndCutPositiveMm = 0.0;
            collisionDetector.FixedEndCutNegativeMm = 150;

            collisionDetector.MinCollisionBoxLengthMm = 5.0;

            if (!enabled)
            {
                collisionDetector.ClearVisualState(urLinkActors);
                LastCollisionReport = CollisionReport.Empty();
                ClearCollisionDebugBoxActors();
            }
            else
            {
                UpdateCollisionState(false);
            }

            if (render)
                Render();
        }

        public CollisionReport UpdateCollisionState(bool render = true)
        {
            if (!collisionDetector.Enabled)
            {
                LastCollisionReport = CollisionReport.Empty();
                ClearCollisionDebugBoxActors();
                return LastCollisionReport;
            }

            LastCollisionReport = collisionDetector.CheckSelfCollision(
                urAssemblies,
                urLinkActors,
                urLinkNames
            );

            collisionDetector.ApplyVisualResult(
                urLinkActors,
                LastCollisionReport
            );

            RefreshCollisionDebugBoxActors();

            if (render)
                Render();

            return LastCollisionReport;
        }

        public void SetCollisionBoundsVisible(bool visible, bool render = true)
        {
            showCollisionDebugBoxes = visible;

            if (!visible)
            {
                ClearCollisionDebugBoxActors();
            }
            else
            {
                if (!collisionDetector.Enabled)
                {
                    SetCollisionDetectionEnabled(true, false);
                }

                UpdateCollisionState(false);
            }

            if (render)
                Render();
        }

        private void ClearCollisionDebugBoxActors()
        {
            if (renderer == null)
                return;

            foreach (vtkActor actor in collisionDebugBoxActors)
            {
                if (actor != null)
                    renderer.RemoveActor(actor);
            }

            collisionDebugBoxActors.Clear();
        }

        private void RefreshCollisionDebugBoxActors()
        {
            ClearCollisionDebugBoxActors();

            if (!showCollisionDebugBoxes)
                return;

            if (!collisionDetector.Enabled)
                return;

            if (renderer == null)
                return;

            List<vtkActor> boxActors = collisionDetector.CreateDebugObbActors();

            foreach (vtkActor actor in boxActors)
            {
                if (actor == null)
                    continue;

                renderer.AddActor(actor);
                collisionDebugBoxActors.Add(actor);
            }
        }
        public void SetCollisionDetectionEnabledAABB(bool enabled, bool render = true)
        {
            collisionDetector.Enabled = enabled;

            // 可以按需要改成 3、5、10 mm
            collisionDetector.SafetyMarginMm = 0.0;

            // 忽略相邻连杆天然接触
            collisionDetector.IgnoreNeighborSpan = 1;

            if (!enabled)
            {
                collisionDetector.ClearVisualState(urLinkActors);
                LastCollisionReport = CollisionReport.Empty();
            }
            else
            {
                UpdateCollisionState(false);
            }

            if (render)
                Render();
        }


        //public CollisionReport UpdateCollisionState(bool render = true)
        //{
        //    if (!collisionDetector.Enabled)
        //    {
        //        LastCollisionReport = CollisionReport.Empty();
        //        return LastCollisionReport;
        //    }

        //    LastCollisionReport = collisionDetector.CheckSelfCollision(
        //        urAssemblies,
        //        urLinkActors,
        //        urLinkNames
        //    );

        //    collisionDetector.ApplyVisualResult(
        //        urLinkActors,
        //        LastCollisionReport
        //    );

        //    if (render)
        //        Render();

        //    return LastCollisionReport;
        //}


        private void UpdateJointFramesFromCurrentJoints(bool render = true)
        {
            if (!IsJointAxisVisible)
                return;

            if (lastJointDeg == null || lastJointDeg.Length < 6)
                return;

            try
            {
                double[] qRad = lastJointDeg
                    .Select(deg => deg * Math.PI / 180.0)
                    .ToArray();

                List<Matrix<double>> frames =
                    Forward.ForwardKinematicsJointAxisFrames(qRad);

                UpdateJointFramesByTransforms(frames, false);

                if (render)
                    Render();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(
                    "[JointAxis] 更新关节坐标系失败：" + ex.Message
                );
            }
        }
        /// <summary>
        /// 使用数组设置关节角，同时更新机器人模型、T01~T06坐标系和转动轴线。
        /// jointsDeg 必须是 6 个。
        /// 单位：degree。
        /// </summary>
        public void SetJointAngles(double[] jointsDeg, bool render = true)
        {
            if (jointsDeg == null)
                throw new ArgumentNullException(nameof(jointsDeg));

            if (jointsDeg.Length != 6)
                throw new ArgumentException("jointsDeg 必须是 6 个关节角。");

            double[] jointDeg = jointsDeg.ToArray();

            // 缓存当前关节角，供 checkbox 从隐藏切换到显示时使用
            lastJointDeg = jointDeg.ToArray();

            // 1. 始终更新机器人模型本体
            SetJointAngles(
                jointDeg[0],
                jointDeg[1],
                jointDeg[2],
                jointDeg[3],
                jointDeg[4],
                jointDeg[5],
                false
            );

            // 2. 只有勾选“关节轴”时，才计算 T01~T06 并更新坐标系
            if (IsJointAxisVisible)
            {
                UpdateJointFramesFromCurrentJoints(false);
            }

            // 3. 如果开启碰撞检测，则每次关节运动后检测一次
            if (collisionDetector.Enabled)
            {
                UpdateCollisionState(false);
            }

            // 4. 最后只渲染一次
            if (render)
                Render();
        }

        /// <summary>
        /// 加载 UR 六轴 PLY 模型。
        /// </summary>
        private void LoadUR7eModel()
        {
            string[] plyFiles =
            {
                Path.Combine(ModelFolder, "base.ply"),
                Path.Combine(ModelFolder, "shoulder.ply"),
                Path.Combine(ModelFolder, "upperarm.ply"),
                Path.Combine(ModelFolder, "forearm.ply"),
                Path.Combine(ModelFolder, "wrist1.ply"),
                Path.Combine(ModelFolder, "wrist2.ply"),
                Path.Combine(ModelFolder, "wrist3.ply"),
            };

            vtkAssembly previousAssembly = vtkAssembly.New();
            renderer.AddActor(previousAssembly);
            urAssemblies.Add(previousAssembly);

            urLinkActors.Clear();
            urLinkNames.Clear();

            for (int i = 0; i < plyFiles.Length; i++)
            {
                if (!File.Exists(plyFiles[i]))
                {
                    throw new FileNotFoundException("未找到机器人 PLY 模型文件：", plyFiles[i]);
                }

                vtkPLYReader reader = vtkPLYReader.New();
                reader.SetFileName(plyFiles[i]);
                reader.Update();

                vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
                mapper.SetInputConnection(reader.GetOutputPort());

                vtkActor actor = vtkActor.New();
                actor.SetMapper(mapper);

                SetActorColor(actor, i);
                urLinkActors.Add(actor);
                urLinkNames.Add(Path.GetFileNameWithoutExtension(plyFiles[i]));

                vtkAssembly assembly = vtkAssembly.New();
                assembly.AddPart(actor);

                vtkTransform transform = vtkTransform.New();

                transform.Translate(
                    jointPosesUR7e[i, 0],
                    jointPosesUR7e[i, 1],
                    jointPosesUR7e[i, 2]);

                transform.RotateX(jointPosesUR7e[i, 3]);
                transform.RotateY(jointPosesUR7e[i, 4]);
                transform.RotateZ(jointPosesUR7e[i, 5]);

                assembly.SetUserTransform(transform);

                previousAssembly.AddPart(assembly);
                urAssemblies.Add(assembly);
                previousAssembly = assembly;
            }

            vtkTransform baseTransform = vtkTransform.New();
            baseTransform.RotateZ(-90);
            urAssemblies[0].SetUserTransform(baseTransform);
        }

        /// <summary>
        /// 设置末端 passive marker 的外部变换。
        /// </summary>
        public void SetPassiveTransform(vtkTransform transform, bool render = true)
        {
            if (PassiveAssembly == null)
                return;

            if (transform == null)
                return;

            vtkTransform t = vtkTransform.New();
            t.DeepCopy(transform);

            PassiveAssembly.SetUserTransform(t);

            if (collisionDetector.Enabled)
            {
                UpdateCollisionState(false);
            }

            if (render)
                Render();
        }

        /// <summary>
        /// 判断 VTK Transform 是否不是单位矩阵。
        /// </summary>
        public static bool IsTransformSet(vtkTransform transform)
        {
            if (transform == null)
                return false;

            vtkMatrix4x4 m = transform.GetMatrix();

            return !(
                Math.Abs(m.GetElement(0, 0) - 1.0) < 1e-6 &&
                Math.Abs(m.GetElement(1, 1) - 1.0) < 1e-6 &&
                Math.Abs(m.GetElement(2, 2) - 1.0) < 1e-6 &&
                Math.Abs(m.GetElement(0, 1)) < 1e-6 &&
                Math.Abs(m.GetElement(0, 2)) < 1e-6 &&
                Math.Abs(m.GetElement(0, 3)) < 1e-6 &&
                Math.Abs(m.GetElement(1, 0)) < 1e-6 &&
                Math.Abs(m.GetElement(1, 2)) < 1e-6 &&
                Math.Abs(m.GetElement(1, 3)) < 1e-6 &&
                Math.Abs(m.GetElement(2, 0)) < 1e-6 &&
                Math.Abs(m.GetElement(2, 1)) < 1e-6 &&
                Math.Abs(m.GetElement(2, 3)) < 1e-6
            );
        }

        /// <summary>
        /// 刷新渲染窗口。
        /// </summary>
        public void Render()
        {
            renderWindow?.Render();
        }

        /// <summary>
        /// 设置机器人整体是否可见。
        /// </summary>
        public void SetRobotVisible(bool visible)
        {
            if (renderer == null || urAssemblies.Count == 0)
                return;

            if (visible && !IsRobotVisible)
            {
                renderer.AddActor(urAssemblies[0]);
            }
            else if (!visible && IsRobotVisible)
            {
                renderer.RemoveActor(urAssemblies[0]);
            }

            IsRobotVisible = visible;
            Render();
        }

        public void ToggleRobotVisibility()
        {
            SetRobotVisible(!IsRobotVisible);
        }

        /// <summary>
        /// 设置轨迹和轨迹点是否可见。
        /// </summary>
        public void SetTrajectoryVisible(bool visible)
        {
            if (renderer == null)
                return;

            if (trajectoryActor != null)
            {
                if (visible && !IsTrajectoryVisible)
                    renderer.AddActor(trajectoryActor);
                else if (!visible && IsTrajectoryVisible)
                    renderer.RemoveActor(trajectoryActor);
            }

            foreach (vtkActor actor in sphereActors)
            {
                if (visible && !IsTrajectoryVisible)
                    renderer.AddActor(actor);
                else if (!visible && IsTrajectoryVisible)
                    renderer.RemoveActor(actor);
            }

            IsTrajectoryVisible = visible;
            Render();
        }

        public void ToggleTrajectoryVisibility()
        {
            SetTrajectoryVisible(!IsTrajectoryVisible);
        }

        /// <summary>
        /// 添加一个球形标记点。
        /// </summary>
        public vtkActor AddSphereAt(
            double x,
            double y,
            double z,
            double radius = 4.0,
            double[] color = null,
            bool render = true)
        {
            vtkSphereSource sphere = vtkSphereSource.New();
            sphere.SetCenter(x, y, z);
            sphere.SetRadius(radius);

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputConnection(sphere.GetOutputPort());

            vtkActor actor = vtkActor.New();
            actor.SetMapper(mapper);

            if (color != null && color.Length == 3)
                actor.GetProperty().SetColor(color[0], color[1], color[2]);
            else
                actor.GetProperty().SetColor(1, 0, 0);

            renderer.AddActor(actor);
            sphereActors.Add(actor);

            if (render)
                Render();

            return actor;
        }

        /// <summary>
        /// 清空所有球形标记点。
        /// </summary>
        public void ClearSpheres(bool render = true)
        {
            if (renderer == null)
                return;

            foreach (vtkActor actor in sphereActors)
            {
                renderer.RemoveActor(actor);
            }

            sphereActors.Clear();

            if (render)
                Render();
        }

        /// <summary>
        /// 初始化轨迹线。
        /// </summary>
        private void InitializeTrajectoryLine()
        {
            trajectoryPointsVTK = vtkPoints.New();
            trajectoryPolyLine = vtkPolyLine.New();
            trajectoryCells = vtkCellArray.New();
            trajectoryPolyData = vtkPolyData.New();
            trajectoryMapper = vtkPolyDataMapper.New();

            trajectoryActor = vtkActor.New();
            trajectoryActor.SetMapper(trajectoryMapper);
            trajectoryActor.GetProperty().SetColor(0, 1, 0);
            trajectoryActor.GetProperty().SetLineWidth(3);

            trajectoryPolyLine.GetPointIds().SetNumberOfIds(0);
            trajectoryCells.InsertNextCell(trajectoryPolyLine);

            trajectoryPolyData.SetPoints(trajectoryPointsVTK);
            trajectoryPolyData.SetLines(trajectoryCells);

            trajectoryMapper.SetInputData(trajectoryPolyData);

            renderer.AddActor(trajectoryActor);
        }

        /// <summary>
        /// 添加一个轨迹点，并更新轨迹线。
        /// </summary>
        public void AddTrajectoryPoint(double x, double y, double z, bool render = true)
        {
            if (trajectoryPointsVTK == null)
                InitializeTrajectoryLine();

            int idx = (int)trajectoryPointsVTK.GetNumberOfPoints();

            trajectoryPointsVTK.InsertNextPoint(x, y, z);
            trajectoryPolyLine.GetPointIds().InsertNextId(idx);

            trajectoryPointsVTK.Modified();
            trajectoryPolyData.Modified();
            trajectoryMapper.Modified();

            if (render)
                Render();
        }

        /// <summary>
        /// 添加一个轨迹点，同时画球。
        /// </summary>
        public void AddTrajectoryPointWithSphere(
            double x,
            double y,
            double z,
            double sphereRadius = 4.0,
            double[] sphereColor = null,
            bool render = true)
        {
            AddTrajectoryPoint(x, y, z, false);
            AddSphereAt(x, y, z, sphereRadius, sphereColor, false);

            if (render)
                Render();
        }

        /// <summary>
        /// 清空轨迹线。
        /// </summary>
        public void ClearTrajectory(bool render = true)
        {
            if (trajectoryPointsVTK == null)
                return;

            trajectoryPointsVTK.Reset();
            trajectoryPolyLine.GetPointIds().SetNumberOfIds(0);

            trajectoryPointsVTK.Modified();
            trajectoryPolyData.Modified();
            trajectoryMapper.Modified();

            if (render)
                Render();
        }

        /// <summary>
        /// 显示外部 mesh。
        /// 适合显示前列腺、phantom 或其他注册结果。
        /// </summary>
        public vtkActor ShowMesh(
            vtkPolyData polyData,
            double[] color = null,
            double opacity = 1.0,
            vtkTransform transform = null,
            bool render = true)
        {
            if (polyData == null)
                return null;

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputData(polyData);

            vtkActor actor = vtkActor.New();
            actor.SetMapper(mapper);

            if (transform != null)
            {
                actor.SetUserTransform(transform);
            }

            if (color != null && color.Length == 3)
                actor.GetProperty().SetColor(color[0], color[1], color[2]);
            else
                actor.GetProperty().SetColor(0.9, 0.2, 0.3);

            actor.GetProperty().SetOpacity(opacity);

            renderer.AddActor(actor);

            if (render)
                Render();

            return actor;
        }

        /// <summary>
        /// 从 renderer 中移除指定 actor。
        /// </summary>
        public void RemoveActor(vtkActor actor, bool render = true)
        {
            if (actor == null || renderer == null)
                return;

            renderer.RemoveActor(actor);

            if (render)
                Render();
        }

        private vtkActor CreateGround(double gridSizeMm)
        {
            currentGridSizeMm = gridSizeMm;

            double halfSize = GroundTotalSizeMm / 2.0;

            int resolution = Math.Max(
                1,
                (int)Math.Round(GroundTotalSizeMm / currentGridSizeMm)
            );

            gridPlaneSource = vtkPlaneSource.New();

            // 直接用真实 mm 坐标建立地面，不再用 Transform.Scale 放大
            gridPlaneSource.SetOrigin(-halfSize, -halfSize, 0);
            gridPlaneSource.SetPoint1(halfSize, -halfSize, 0);
            gridPlaneSource.SetPoint2(-halfSize, halfSize, 0);

            gridPlaneSource.SetXResolution(resolution);
            gridPlaneSource.SetYResolution(resolution);
            gridPlaneSource.Update();

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputConnection(gridPlaneSource.GetOutputPort());

            vtkActor actor = vtkActor.New();
            actor.SetMapper(mapper);

            actor.GetProperty().SetRepresentationToWireframe();
            actor.GetProperty().SetColor(0.8, 0.8, 0.8);
            actor.GetProperty().SetLineWidth(1.0f);

            return actor;
        }
        public void SetGridSize(double gridSizeMm, bool render = true)
        {
            if (gridSizeMm <= 0)
                return;

            currentGridSizeMm = gridSizeMm;

            if (gridPlaneSource == null)
                return;

            int resolution = Math.Max(
                1,
                (int)Math.Round(GroundTotalSizeMm / currentGridSizeMm)
            );

            gridPlaneSource.SetXResolution(resolution);
            gridPlaneSource.SetYResolution(resolution);
            gridPlaneSource.Modified();
            gridPlaneSource.Update();

            if (render)
                Render();
        }

        private vtkAxesActor CreateAxes()
        {
            double length = 400;

            vtkAxesActor axes = vtkAxesActor.New();
            axes.SetTotalLength(length, length, length);
            axes.SetShaftTypeToLine();

            axes.GetXAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone();
            axes.GetYAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone();
            axes.GetZAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone();

            int fontSize = 16;

            axes.GetXAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(fontSize);
            axes.GetYAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(fontSize);
            axes.GetZAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(fontSize);

            return axes;
        }
        private vtkActor CreateWorkspaceSphere(double radiusMm)
        {
            vtkSphereSource sphere = vtkSphereSource.New();
            sphere.SetCenter(0, 0, 0);
            sphere.SetRadius(radiusMm);
            sphere.SetThetaResolution(72);
            sphere.SetPhiResolution(36);

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputConnection(sphere.GetOutputPort());

            vtkActor actor = vtkActor.New();
            actor.SetMapper(mapper);

            actor.GetProperty().SetColor(0.2, 0.6, 1.0);
            actor.GetProperty().SetOpacity(0.12);
            actor.GetProperty().SetRepresentationToWireframe();

            return actor;
        }

        private vtkActor CreateTargetPointActor(double x, double y, double z)
        {
            vtkSphereSource sphere = vtkSphereSource.New();
            sphere.SetCenter(x, y, z);
            sphere.SetRadius(18.0);
            sphere.SetThetaResolution(32);
            sphere.SetPhiResolution(16);

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputConnection(sphere.GetOutputPort());

            vtkActor actor = vtkActor.New();
            actor.SetMapper(mapper);
            actor.GetProperty().SetColor(1.0, 0.0, 0.0);
            actor.GetProperty().SetOpacity(1.0);

            return actor;
        }

        private vtkActor CreateDynamicLineActor(
            double[] color,
            float lineWidth,
            out vtkLineSource lineSource)
        {
            lineSource = vtkLineSource.New();
            lineSource.SetPoint1(0, 0, 0);
            lineSource.SetPoint2(0, 0, 1);

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputConnection(lineSource.GetOutputPort());

            vtkActor actor = vtkActor.New();
            actor.SetMapper(mapper);

            actor.GetProperty().SetColor(color[0], color[1], color[2]);
            actor.GetProperty().SetLineWidth(lineWidth);

            return actor;
        }
        private vtkAssembly CreateJointAxisAssembly()
        {
            vtkAssembly assembly = vtkAssembly.New();

            jointFrameAxisSources.Clear();
            jointFrameAxisActors.Clear();

            jointAxisLineSources.Clear();
            jointAxisLineActors.Clear();

            for (int i = 0; i < 7; i++)
            {
                vtkLineSource[] frameSources = new vtkLineSource[3];
                vtkActor[] frameActors = new vtkActor[3];

                // X轴：红色
                frameActors[0] = CreateDynamicLineActor(
                    new double[] { 1.0, 0.0, 0.0 },
                    3.0f,
                    out frameSources[0]);

                // Y轴：绿色
                frameActors[1] = CreateDynamicLineActor(
                    new double[] { 0.0, 1, 0.0 },
                    3.0f,
                    out frameSources[1]);

                // Z轴：蓝色
                frameActors[2] = CreateDynamicLineActor(
                    new double[] { 0.0, 0, 1.0 },
                    3.0f,
                    out frameSources[2]);

                for (int k = 0; k < 3; k++)
                {
                    assembly.AddPart(frameActors[k]);
                }

                jointFrameAxisSources.Add(frameSources);
                jointFrameAxisActors.Add(frameActors);

                //// 额外画一条更粗的“关节转动轴线”
                //// 默认用当前坐标系的局部Z轴方向
                //vtkLineSource jointAxisSource = vtkLineSource.New();
                //jointAxisSource.SetPoint1(0, 0, 0);
                //jointAxisSource.SetPoint2(0, 0, JointAxisLineLength);
                //vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
                //mapper.SetInputConnection(jointAxisSource.GetOutputPort());
                //vtkActor jointAxisActor = vtkActor.New();
                //jointAxisActor.SetMapper(mapper);
                //double[] color = GetJointAxisColor(i);
                //jointAxisActor.GetProperty().SetColor(color[0], color[1], color[2]);
                //jointAxisActor.GetProperty().SetLineWidth(5.0f);
                //jointAxisLineSources.Add(jointAxisSource);
                //jointAxisLineActors.Add(jointAxisActor);
                //assembly.AddPart(jointAxisActor);


                // 额外画一条更粗的“关节转动轴线”
                // 默认用当前坐标系的局部Z轴方向
                vtkLineSource jointAxisSource = vtkLineSource.New();
                jointAxisSource.SetPoint1(0, 0, 0);
                jointAxisSource.SetPoint2(0, 0, JointAxisLineLength);
                vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
                mapper.SetInputConnection(jointAxisSource.GetOutputPort());
                mapper.ScalarVisibilityOff();
                vtkActor jointAxisActor = vtkActor.New();
                jointAxisActor.SetMapper(mapper);
                // 固定为蓝色，不再使用 GetJointAxisColor(i)
                jointAxisActor.GetProperty().SetColor(0.0, 0.0, 1.0);
                jointAxisActor.GetProperty().SetLineWidth(5.0f);
                jointAxisLineSources.Add(jointAxisSource);
                jointAxisLineActors.Add(jointAxisActor);
                assembly.AddPart(jointAxisActor);
            }

            return assembly;
        }
        private void UpdateLineSource(
            vtkLineSource source,
            double x1, double y1, double z1,
            double x2, double y2, double z2)
        {
            if (source == null)
                return;

            source.SetPoint1(x1, y1, z1);
            source.SetPoint2(x2, y2, z2);
            source.Modified();
            source.Update();
        }

        //private vtkAssembly CreateJointAxisAssembly()
        //{
        //    vtkAssembly assembly = vtkAssembly.New();

        //    jointFrameAxesActors.Clear();
        //    jointAxisLineSources.Clear();
        //    jointAxisLineActors.Clear();

        //    for (int i = 0; i < 7; i++)
        //    {
        //        vtkAxesActor axes = vtkAxesActor.New();

        //        axes.SetTotalLength(
        //            JointFrameAxisLength,
        //            JointFrameAxisLength,
        //            JointFrameAxisLength
        //        );

        //        axes.SetShaftTypeToLine();

        //        axes.GetXAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone();
        //        axes.GetYAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone();
        //        axes.GetZAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone();

        //        int fontSize = 12;

        //        axes.GetXAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(fontSize);
        //        axes.GetYAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(fontSize);
        //        axes.GetZAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(fontSize);

        //        jointFrameAxesActors.Add(axes);
        //        assembly.AddPart(axes);

        //        vtkLineSource lineSource = vtkLineSource.New();
        //        lineSource.SetPoint1(0, 0, 0);
        //        lineSource.SetPoint2(0, 0, JointAxisLineLength);

        //        vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
        //        mapper.SetInputConnection(lineSource.GetOutputPort());

        //        vtkActor lineActor = vtkActor.New();
        //        lineActor.SetMapper(mapper);

        //        double[] color = GetJointAxisColor(i);
        //        lineActor.GetProperty().SetColor(color[0], color[1], color[2]);
        //        lineActor.GetProperty().SetLineWidth(4.0f);

        //        jointAxisLineSources.Add(lineSource);
        //        jointAxisLineActors.Add(lineActor);

        //        assembly.AddPart(lineActor);
        //    }

        //    return assembly;
        //}

        private double[] GetJointAxisColor(int index)
        {
            double[][] colors =
            {
                new double[] { 1.0, 0.0, 0.0 },
                new double[] { 0.0, 0.8, 0.0 },
                new double[] { 0.0, 0.2, 1.0 },
                new double[] { 1.0, 0.6, 0.0 },
                new double[] { 1.0, 0.0, 1.0 },
                new double[] { 0.0, 0.9, 0.9 },
                new double[] { 0.6, 0.2, 1.0 }
            };

            if (index < 0 || index >= colors.Length)
                return new double[] { 1.0, 1.0, 1.0 };

            return colors[index];
        }

        private vtkTransform MatrixToVtkTransform(Matrix<double> T)
        {
            vtkMatrix4x4 vtkMatrix = vtkMatrix4x4.New();

            for (int r = 0; r < 4; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    vtkMatrix.SetElement(r, c, T[r, c]);
                }
            }

            vtkTransform transform = vtkTransform.New();
            transform.SetMatrix(vtkMatrix);

            return transform;
        }
        //public void UpdateJointFramesByTransforms(IList<Matrix<double>> transforms, bool render = true)
        //{
        //    if (transforms == null)
        //        return;

        //    int count = Math.Min(
        //        transforms.Count,
        //        Math.Min(jointFrameAxesActors.Count, jointAxisLineSources.Count)
        //    );

        //    for (int i = 0; i < count; i++)
        //    {
        //        Matrix<double> T = transforms[i];

        //        if (T == null || T.RowCount < 4 || T.ColumnCount < 4)
        //            continue;

        //        // 1. 坐标系 actor：位置和姿态完全由 T0i 决定
        //        vtkTransform vtkTransform = MatrixToVtkTransform(T);
        //        jointFrameAxesActors[i].SetUserTransform(vtkTransform);

        //        // 2. 转动轴线：默认画当前坐标系的局部 Z 轴
        //        double ox = T[0, 3];
        //        double oy = T[1, 3];
        //        double oz = T[2, 3];

        //        double zx = T[0, 2];
        //        double zy = T[1, 2];
        //        double zz = T[2, 2];

        //        double norm = Math.Sqrt(zx * zx + zy * zy + zz * zz);

        //        if (norm < 1e-9)
        //            continue;

        //        zx /= norm;
        //        zy /= norm;
        //        zz /= norm;

        //        double x2 = ox + JointAxisLineLength * zx;
        //        double y2 = oy + JointAxisLineLength * zy;
        //        double z2 = oz + JointAxisLineLength * zz;

        //        jointAxisLineSources[i].SetPoint1(ox, oy, oz);
        //        jointAxisLineSources[i].SetPoint2(x2, y2, z2);
        //        jointAxisLineSources[i].Modified();
        //        jointAxisLineSources[i].Update();
        //    }

        //    if (render)
        //        Render();
        //}
        public void UpdateJointFramesByTransforms(
            IList<Matrix<double>> transforms,
            bool render = true)
        {
            if (transforms == null)
                return;

            int count = Math.Min(
                transforms.Count,
                Math.Min(jointFrameAxisSources.Count, jointAxisLineSources.Count)
            );

            for (int i = 0; i < count; i++)
            {
                Matrix<double> T = transforms[i];

                if (T == null || T.RowCount < 4 || T.ColumnCount < 4)
                    continue;

                // 当前坐标系原点：T0i 第4列前三行
                double ox = T[0, 3];
                double oy = T[1, 3];
                double oz = T[2, 3];

                // 当前坐标系 X 轴方向：T0i 第一列
                double xx = T[0, 0];
                double xy = T[1, 0];
                double xz = T[2, 0];

                // 当前坐标系 Y 轴方向：T0i 第二列
                double yx = T[0, 1];
                double yy = T[1, 1];
                double yz = T[2, 1];

                // 当前坐标系 Z 轴方向：T0i 第三列
                double zx = T[0, 2];
                double zy = T[1, 2];
                double zz = T[2, 2];

                Normalize3(ref xx, ref xy, ref xz);
                Normalize3(ref yx, ref yy, ref yz);
                Normalize3(ref zx, ref zy, ref zz);

                double L = JointFrameAxisLength;

                // 更新局部坐标系 X/Y/Z 三条轴线
                vtkLineSource[] frameSources = jointFrameAxisSources[i];

                // X轴，红色
                UpdateLineSource(
                    frameSources[0],
                    ox, oy, oz,
                    ox + L * xx,
                    oy + L * xy,
                    oz + L * xz
                );

                // Y轴，绿色
                UpdateLineSource(
                    frameSources[1],
                    ox, oy, oz,
                    ox + L * yx,
                    oy + L * yy,
                    oz + L * yz
                );

                // Z轴，蓝色
                UpdateLineSource(
                    frameSources[2],
                    ox, oy, oz,
                    ox + L * zx,
                    oy + L * zy,
                    oz + L * zz
                );

                // 额外更新“关节转动轴线”
                // 注意：如果关节绕局部Z轴转动，那么这条线本身不会表现出绕自身旋转。
                UpdateLineSource(
                    jointAxisLineSources[i],
                    ox, oy, oz,
                    ox + JointAxisLineLength * zx,
                    oy + JointAxisLineLength * zy,
                    oz + JointAxisLineLength * zz
                );
            }

            if (render)
                Render();
        }
        private void Normalize3(ref double x, ref double y, ref double z)
        {
            double n = Math.Sqrt(x * x + y * y + z * z);

            if (n < 1e-9)
                return;

            x /= n;
            y /= n;
            z /= n;
        }
        private void SetActorColor(vtkActor actor, int id)
        {
            double[,] colors = new double[,]
            {
                {0.41 , 0.89 , 0.61},
                {0.9 , 0.44 , 0.83},
                {0.62 , 0.41 , 0.72},
                {0.48 , 0.63 , 0.61},
                {0.43 , 0.81 , 0.95},
                {0.67 , 0.97 , 0.44},
                {0.27 , 0.97 , 0.44},
                {0.62 , 0.41 , 0.42},
                {0.02 , 0.41 , 0.42},
                {0.62 , 0.80 , 0.80},
            };

            int colorId = id;

            if (colorId < 0 || colorId >= colors.GetLength(0))
                colorId = 0;

            actor.GetProperty().SetAmbientColor(colors[colorId, 0], colors[colorId, 1], colors[colorId, 2]);
            actor.GetProperty().SetDiffuseColor(colors[colorId, 0], colors[colorId, 1], colors[colorId, 2]);
            actor.GetProperty().SetDiffuse(0.8);
            actor.GetProperty().SetSpecular(0.5);
            actor.GetProperty().SetSpecularColor(0.7, 0.7, 0.7);
            actor.GetProperty().SetSpecularPower(30.0);
        }

        public vtkRenderer GetRenderer()
        {
            return renderer;
        }

        public vtkRenderWindow GetRenderWindow()
        {
            return renderWindow;
        }

        public vtkRenderWindowInteractor GetInteractor()
        {
            return interactor;
        }

        public void Dispose()
        {
            if (hostPanel != null)
            {
                hostPanel.Resize -= HostPanel_Resize;
                hostPanel.MouseUp -= HostPanel_MouseUp;
            }

            try
            {
                if (orientationMarkerWidget != null)
                {
                    orientationMarkerWidget.SetEnabled(0);
                    orientationMarkerWidget = null;
                }

                orientationCubeActor = null;
                orientationAxesActor = null;
                orientationMarkerAssembly = null;
            }
            catch
            {
            }

            try
            {
                interactor?.TerminateApp();
            }
            catch
            {
                // 忽略VTK退出异常
            }

            interactor = null;
            style = null;
            panStyle = null;
            renderer = null;
            renderWindow = null;
            hostPanel = null;
        }

        public void SetCameraView(CameraViewMode mode, bool render = true)
        {
            if (renderer == null)
                return;

            // 自由视角不强制重置相机，保留鼠标旋转/缩放后的当前状态
            if (mode == CameraViewMode.Free)
                return;

            vtkCamera camera = renderer.GetActiveCamera();

            double[] center = GetVisibleSceneCenter();
            double distance = GetVisibleSceneDistance();

            double cx = center[0];
            double cy = center[1];
            double cz = center[2];

            camera.SetFocalPoint(cx, cy, cz);

            switch (mode)
            {
                case CameraViewMode.Front:
                    // 正视：从 -Y 方向看向机器人
                    camera.SetPosition(cx, cy - distance, cz);
                    camera.SetViewUp(0, 0, 1);
                    break;

                case CameraViewMode.Top:
                    // 俯视：从 +Z 方向向下看
                    camera.SetPosition(cx, cy, cz + distance);
                    camera.SetViewUp(0, 1, 0);
                    break;

                case CameraViewMode.Side:
                    // 侧视：从 +X 方向看向机器人
                    camera.SetPosition(cx + distance, cy, cz);
                    camera.SetViewUp(0, 0, 1);
                    break;
            }

            camera.OrthogonalizeViewUp();
            renderer.ResetCameraClippingRange();

            if (render)
                Render();
        }

        private bool TryComputeVisiblePropBounds(out double[] bounds)
        {
            bounds = new double[6];

            if (renderer == null)
                return false;

            IntPtr boundsPtr = IntPtr.Zero;

            try
            {
                boundsPtr = Marshal.AllocHGlobal(sizeof(double) * 6);

                renderer.ComputeVisiblePropBounds(boundsPtr);

                Marshal.Copy(boundsPtr, bounds, 0, 6);

                if (double.IsNaN(bounds[0]) ||
                    bounds[0] > bounds[1] ||
                    bounds[2] > bounds[3] ||
                    bounds[4] > bounds[5])
                {
                    return false;
                }

                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                if (boundsPtr != IntPtr.Zero)
                {
                    Marshal.FreeHGlobal(boundsPtr);
                }
            }
        }
        private double[] GetVisibleSceneCenter()
        {
            if (!TryComputeVisiblePropBounds(out double[] bounds))
            {
                return new double[] { 0, 0, 300 };
            }

            return new double[]
            {
                0.5 * (bounds[0] + bounds[1]),
                0.5 * (bounds[2] + bounds[3]),
                0.5 * (bounds[4] + bounds[5])
            };
        }
        private double GetVisibleSceneDistance()
        {
            if (!TryComputeVisiblePropBounds(out double[] bounds))
            {
                return 3000.0;
            }

            double dx = bounds[1] - bounds[0];
            double dy = bounds[3] - bounds[2];
            double dz = bounds[5] - bounds[4];

            double diagonal = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            return Math.Max(2500.0, diagonal * 1.8);
        }
    }
}