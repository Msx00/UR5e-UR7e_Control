using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.ObjectModel;
using System.Net.Sockets;
using System.Windows;
using System.Windows.Controls;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;
using System.Net.NetworkInformation;
using WpfRobot.command;
using WpfRobot.kinematics;
using WpfRobot.inquiry;
using RcmSolver = WpfRobot.rcm.rcm;
using static WpfRobot.simulation_mode;

namespace WpfRobot
{
    public partial class MainWindow : Window
    {
        /// <summary>
        /// subWindow
        /// </summary>
        //private AiControlWindow _aiControlWindow;
        private robParameter _statusParameterWindow;
        private calibrationWindow _calibrationWindow;


        public ObservableCollection<JointStateRow> JointRows { get; set; }

        private const string RobotModelFolder = "./ur7e_ply";

        private bool _uiReady = false;
        private bool _isUpdatingJointUi = false;
        private bool _hasReceivedActualRobotState = false;

        /// <summary>
        /// vtk窗口实现左键按下拖动平移
        /// </summary>
        private bool _isVtkPanning = false;
        private System.Windows.Point _lastPanPoint;

        /// <summary>
        /// 运动控制器
        /// </summary>
        private RobotMotionCoordinator _motionCoordinator;


        /// <summary>
        /// 解决UI指令，在机器人端响应慢的问题
        /// </summary>
        private readonly object _rtdeUiThrottleLock = new object();
        private DateTime _lastRtdeUiPostTime = DateTime.MinValue;
        // RTDE 可以 125Hz 接收，但 UI/VTK 不要 125Hz 刷新。
        // 50ms = 20Hz，已经足够流畅，而且不会堵住按钮点击。
        private const int RtdeUiUpdateIntervalMs = 50;

        public MainWindow()
        {
            InitializeComponent();

            Loaded += MainWindow_Loaded;
            Closed += MainWindow_Closed;
            LoadToolRcmSettings();

            InitJointTable();

            _uiReady = true;

            //初始化未联机关节状态
            UpdateJointUiFromGlobal();

            // 初始化运动协调器
            _motionCoordinator = new RobotMotionCoordinator(Dispatcher);
            _motionCoordinator.LogMessage += msg =>
            {
                AddLog(msg);
            };

            TxtConnectionState.Text = "未连接";
            TxtConnectionState.Foreground = System.Windows.Media.Brushes.Red;

            TxtRobotCommState.Text = "未连接";
            TxtRobotCommState.Foreground = System.Windows.Media.Brushes.Red;


            //初始化机器人运动参数
            RobotParameterRuntime.LoadFromSettings();

            AddLog("[INFO] 软件界面初始化完成");


            //// 订阅 NDI 日志
            //NDISDK.NDI_self_Class.LogReceived -= NdiLogReceived;
            //NDISDK.NDI_self_Class.LogReceived += NdiLogReceived;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            InitializeRobotSystem();
        }
        private void NdiLogReceived(string msg)
        {
            AddLogByDispatcher(msg);
        }

        private async void MainWindow_Closed(object sender, EventArgs e)
        {
            try
            {
                global_socket.CloseAll();
            }
            catch
            {
                // 程序关闭时忽略通信关闭异常
            }
            try
            {
                if (global_variable.simulationRealTime != null &&
                    global_variable.simulationRealTime.IsInitialized)
                {
                    global_variable.simulationRealTime.Dispose();
                }
            }
            catch
            {
                // 程序关闭时忽略 VTK 释放异常
            }
            try
            {

            }
            catch
            {
                // 程序关闭时忽略 NDI 关闭异常
            }
        }
        private void LoadToolRcmSettings()
        {
            double toolX = Properties.Settings.Default.ToolX;
            double toolY = Properties.Settings.Default.ToolY;
            double toolZ = Properties.Settings.Default.ToolZ;

            double rcmX = Properties.Settings.Default.RcmX;
            double rcmY = Properties.Settings.Default.RcmY;
            double rcmZ = Properties.Settings.Default.RcmZ;

            TxtToolX.Text = toolX.ToString("F2");
            TxtToolY.Text = toolY.ToString("F2");
            TxtToolZ.Text = toolZ.ToString("F2");

            TxtRcmX.Text = rcmX.ToString("F2");
            TxtRcmY.Text = rcmY.ToString("F2");
            TxtRcmZ.Text = rcmZ.ToString("F2");

            ChkRcmMode.IsChecked = Properties.Settings.Default.RcmMode;

            // 新增：同步到运行时参数
            RobotParameterRuntime.RcmX = rcmX;
            RobotParameterRuntime.RcmY = rcmY;
            RobotParameterRuntime.RcmZ = rcmZ;
            RobotParameterRuntime.RcmEnabled = Properties.Settings.Default.RcmMode;
        }

        private void InitializeRobotSystem()
        {
            try
            {
                _simulationDriveMode = SimulationDriveMode.TargetCommand;

                UpdateGlobalToolFromUi(false);

                UpdateJointUiFromGlobal();

                InitializeVtkSimulation();

                RefreshRcmPointInVtk();//RCM蓝色提示点

                UpdateRobotFromGlobalJoint();

                BtnViewActualRobot.Content = "仿";
                BtnViewActualRobot.ToolTip = "当前：UI目标仿真。点击切换到真实机器人反馈仿真";

                if (TxtSimulationTitle != null)
                {
                    TxtSimulationTitle.Text = "机器人仿真显示：UI目标模式";
                    TxtSimulationTitle.Foreground = System.Windows.Media.Brushes.Black;
                }

                AddLog("[INFO] 机器人系统初始化完成，当前为UI目标仿真模式");
            }
            catch (Exception ex)
            {
                AddLog("[ERROR] 机器人系统初始化失败：" + ex.Message);
            }
        }
        private bool UpdateGlobalToolFromUi(bool showMessage = true)
        {
            if (!double.TryParse(TxtToolX.Text, out double toolX) ||
                !double.TryParse(TxtToolY.Text, out double toolY) ||
                !double.TryParse(TxtToolZ.Text, out double toolZ))
            {
                if (showMessage)
                {
                    System.Windows.MessageBox.Show(
                        "工具参数输入格式错误，请检查 Tool_X / Tool_Y / Tool_Z 是否都是有效数字。",
                        "工具参数错误",
                        MessageBoxButton.OK,
                        MessageBoxImage.Warning
                    );
                }

                AddLog("[ERROR] 工具参数输入格式错误");
                return false;
            }

            global_variable.SetGlobalToolVector(toolX, toolY, toolZ);

            AddLog(
                $"[TOOL] 工具参数已更新: " +
                $"Tool_X={toolX:F2}, Tool_Y={toolY:F2}, Tool_Z={toolZ:F2}"
            );

            return true;
        }
        private void InitializeVtkSimulation()
        {
            try
            {
                if (global_variable.simulationRealTime == null)
                {
                    global_variable.simulationRealTime =
                        new WpfRobot.simulation.SimulationRealTime();
                }

                if (!global_variable.simulationRealTime.IsInitialized)
                {
                    global_variable.simulationRealTime.Initialize(
                        VtkRobotHost,
                        RobotModelFolder
                    );

                    AddLog("[SIM] VTK机器人仿真初始化完成");
                }

                RefreshSimulationByCurrentMode();
                simulation_mode.SetGridSize(GetSelectedGridSizeMm());
                SimulationDisplayOption_Changed(null, null);
                CollisionDetection_Changed(null, null);
                simulation_mode.SetCameraView(GetSelectedViewModeText());

                // 默认鼠标左键拖动旋转
                simulation_mode.SetRotateMode();
            }
            catch (Exception ex)
            {
                AddLog("[ERROR] VTK机器人仿真初始化失败：" + ex.Message);
                System.Windows.MessageBox.Show(
                    "VTK机器人仿真初始化失败：\n" + ex.Message,
                    "VTK初始化错误",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error
                );
            }
        }
        private void InitJointTable()
        {
            JointRows = new ObservableCollection<JointStateRow>
            {
                new JointStateRow { Joint = "J1", Current = "90.00",  Target = "90.00",  Error = "0.00" },
                new JointStateRow { Joint = "J2", Current = "-90.00", Target = "-90.00", Error = "0.00" },
                new JointStateRow { Joint = "J3", Current = "90.00",  Target = "90.00",  Error = "0.00" },
                new JointStateRow { Joint = "J4", Current = "-90.00", Target = "-90.00", Error = "0.00" },
                new JointStateRow { Joint = "J5", Current = "-90.00", Target = "-90.00", Error = "0.00" },
                new JointStateRow { Joint = "J6", Current = "90.00",   Target = "90.00",   Error = "0.00" }
            };

            DgJointState.ItemsSource = JointRows;
        }

        private void UpdateJointUiFromGlobal()
        {
            if (!_uiReady)
                return;

            if (global_variable.globalJointDeg == null ||
                global_variable.globalJointDeg.Length < 6)
                return;

            _isUpdatingJointUi = true;

            double[] q = global_variable.globalJointDeg;

            SliderJ1.Value = q[0];
            SliderJ2.Value = q[1];
            SliderJ3.Value = q[2];
            SliderJ4.Value = q[3];
            SliderJ5.Value = q[4];
            SliderJ6.Value = q[5];
            
            TxtJ1.Text = q[0].ToString("F2");
            TxtJ2.Text = q[1].ToString("F2");
            TxtJ3.Text = q[2].ToString("F2");
            TxtJ4.Text = q[3].ToString("F2");
            TxtJ5.Text = q[4].ToString("F2");
            TxtJ6.Text = q[5].ToString("F2");
            
            //RefreshJointTable(q, q);
            UpdateJointTableTargetOnly(q);

            _isUpdatingJointUi = false;
        }

        private bool UpdateGlobalJointFromTextBox(bool showMessage = true)
        {
            if (!TryReadDouble(TxtJ1, out double q1, showMessage)) return false;
            if (!TryReadDouble(TxtJ2, out double q2, showMessage)) return false;
            if (!TryReadDouble(TxtJ3, out double q3, showMessage)) return false;
            if (!TryReadDouble(TxtJ4, out double q4, showMessage)) return false;
            if (!TryReadDouble(TxtJ5, out double q5, showMessage)) return false;
            if (!TryReadDouble(TxtJ6, out double q6, showMessage)) return false;

            q1 = Clamp(q1, -360, 360);
            q2 = Clamp(q2, -360, 360);
            q3 = Clamp(q3, -360, 360);
            q4 = Clamp(q4, -360, 360);
            q5 = Clamp(q5, -360, 360);
            q6 = Clamp(q6, -360, 360);

            global_variable.SetGlobalJointDeg(
                q1, q2, q3, q4, q5, q6
            );

            return true;
        }

        private void UpdateTextBoxFromSlider()
        {
            if (!_uiReady || _isUpdatingJointUi)
                return;

            _isUpdatingJointUi = true;

            TxtJ1.Text = SliderJ1.Value.ToString("F2");
            TxtJ2.Text = SliderJ2.Value.ToString("F2");
            TxtJ3.Text = SliderJ3.Value.ToString("F2");
            TxtJ4.Text = SliderJ4.Value.ToString("F2");
            TxtJ5.Text = SliderJ5.Value.ToString("F2");
            TxtJ6.Text = SliderJ6.Value.ToString("F2");

            global_variable.SetGlobalJointDeg(
                SliderJ1.Value,
                SliderJ2.Value,
                SliderJ3.Value,
                SliderJ4.Value,
                SliderJ5.Value,
                SliderJ6.Value
            );

            UpdateJointTableTargetOnly(global_variable.globalJointDeg);

            _isUpdatingJointUi = false;
        }

        private void UpdateSliderFromTextBox()
        {
            if (!_uiReady || _isUpdatingJointUi)
                return;

            if (!UpdateGlobalJointFromTextBox(true))
                return;

            UpdateJointUiFromGlobal();
        }

        private void RefreshForwardKinematicsCache()
        {
            try
            {
                if (global_variable.globalJointDeg == null ||
                    global_variable.globalJointDeg.Length < 6)
                    return;

                double[] qRad = global_variable.globalJointDeg
                    .Take(6)
                    .Select(deg => deg * Math.PI / 180.0)
                    .ToArray();

                var fk = Forward.ForwardKinematicsMatrix6_All(qRad);

                global_variable.globalT06 = fk.T06;
                global_variable.globalT07 = fk.T07;
                global_variable.globalT0Tcp = fk.T0Tcp;

                global_variable.globalFkValid = true;
            }
            catch (Exception ex)
            {
                global_variable.globalFkValid = false;
                AddLog("[FK] 正运动学缓存刷新失败：" + ex.Message);
            }
        }

        private void UpdateRobotFromGlobalJoint()
        {
            global_variable.SyncToolToForward();

            // 始终更新目标值表格
            UpdateJointTableTargetOnly(global_variable.globalJointDeg);

            // 同时计算 T06 T07 T0Tcp 的正运动学缓存，供 UI 显示和逆运动学输入使用
            RefreshForwardKinematicsCache();

            // 新增：根据当前 UI 目标关节角，计算 TCP 位姿，并写入逆运动学输入框
            UpdateTcpPoseFromForwardKinematics();
            UpdateT06PoseFromForwardKinematics();
            UpdateT07PoseFromForwardKinematics();

            // 只有 UI目标模式 才用 globalJointDeg 驱动 VTK
            if (_simulationDriveMode == SimulationDriveMode.TargetCommand)
            {
                if (global_variable.simulationRealTime != null &&
                    global_variable.simulationRealTime.IsInitialized)
                {
                    global_variable.simulationRealTime.UpdateJointAngles(
                        global_variable.globalJointDeg
                    );
                }
            }
        }

        private void RefreshJointTable(double[] currentJointDeg, double[] targetJointDeg)
        {
            if (JointRows == null)
                return;

            if (currentJointDeg == null || targetJointDeg == null)
                return;

            if (currentJointDeg.Length < 6 || targetJointDeg.Length < 6)
                return;

            for (int i = 0; i < 6; i++)
            {
                double current = currentJointDeg[i];
                double target = targetJointDeg[i];

                JointRows[i].Current = current.ToString("F2");
                JointRows[i].Target = target.ToString("F2");
                JointRows[i].Error = (target - current).ToString("F2");
            }

            DgJointState.Items.Refresh();
        }

        private bool TryReadDouble(System.Windows.Controls.TextBox textBox, out double value, bool showMessage = true)
        {
            value = 0.0;

            if (textBox == null)
            {
                if (showMessage)
                    System.Windows.MessageBox.Show("TextBox 为空。");

                return false;
            }

            if (!double.TryParse(textBox.Text, out value))
            {
                if (showMessage)
                    System.Windows.MessageBox.Show("请输入有效数字：" + textBox.Name);

                return false;
            }

            return true;
        }

        public double Clamp(double value, double min, double max)
        {
            if (value < min)
                return min;

            if (value > max)
                return max;

            return value;
        }

        private void AddLog(string message)
        {
            string time = DateTime.Now.ToString("HH:mm:ss.fff");
            TxtLogBox.AppendText(Environment.NewLine + $"[{time}] {message}");
            TxtLogBox.ScrollToEnd();
        }

        private void JointSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!_uiReady || _isUpdatingJointUi)
                return;

            UpdateTextBoxFromSlider();

            // 拖动滑块时是否实时驱动仿真：
            // 如果你希望拖动时实时动，就保留下面这句。
            // 如果你只想点击“发送关节指令”后再动，就注释掉下面这句。
            UpdateRobotFromGlobalJoint();
        }

        private void JointTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            if (!_uiReady || _isUpdatingJointUi)
                return;

            UpdateSliderFromTextBox();

            // TextBox 输入完成后同步仿真
            UpdateRobotFromGlobalJoint();
        }

        //public void RtdeClient_ur7e(string ip)
        //{
        //    try
        //    {
        //        global_socket._inquiryState.StateUpdated -= InquiryState_StateUpdated;
        //        global_socket._inquiryState.RtdeDisconnected -= InquiryState_RtdeDisconnected;

        //        global_socket._inquiryState.StateUpdated += InquiryState_StateUpdated;
        //        global_socket._inquiryState.RtdeDisconnected += InquiryState_RtdeDisconnected;

        //        string log = global_socket._inquiryState.Connect(ip, 10);

        //        AddLog(log);
        //        AddLog("[INFO] RTDE机器人状态查询已启动");
        //    }
        //    catch (Exception ex)
        //    {
        //        AddLog("[ERROR] RTDE连接失败：" + ex.Message);
        //        System.Windows.MessageBox.Show("RTDE连接失败：\n" + ex.Message);
        //    }
        //}
        private void UpdateJointTableTargetOnly(double[] targetJointDeg)
        {
            if (JointRows == null)
                return;

            if (targetJointDeg == null || targetJointDeg.Length < 6)
                return;

            for (int i = 0; i < 6; i++)
            {
                global_variable._targetJointDegForTable[i] = targetJointDeg[i];
            }

            RefreshJointTable(
                global_variable._actualJointDegForTable,
                global_variable._targetJointDegForTable
            );
        }

        private void UpdateJointTableCurrentOnly(double[] actualQDeg)
        {
            if (JointRows == null)
                return;

            if (actualQDeg == null || actualQDeg.Length < 6)
                return;

            // 前六个关节来自 UR RTDE actual_q
            for (int i = 0; i < 6; i++)
            {
                global_variable._actualJointDegForTable[i] = actualQDeg[i];
            }

            RefreshJointTable(
                global_variable._actualJointDegForTable,
                global_variable._targetJointDegForTable
            );
        }




        /// <summary>
        /// 机器人状态
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void StartDashboardMonitor(string ip)
        {
            global_socket.PrepareDashboardClient();

            UrDashboardClient client = global_socket.DashboardClient;
            CancellationToken token = global_socket.DashboardCts.Token;

            _ = Task.Run(async () =>
            {
                try
                {
                    //await client.ConnectAsync(ip);
                    await client.ConnectAsync(ip, global_socket.DashboardPort);

                    AddLogByDispatcher("[Dashboard] 已连接");

                    while (!token.IsCancellationRequested)
                    {
                        UrDashboardSnapshot snapshot = await client.ReadSnapshotAsync();

                        // 保存到全局，供 robParameter 状态窗口读取
                        global_socket.UpdateDashboardState(snapshot);

                        await Dispatcher.InvokeAsync(() =>
                        {
                            UpdateDashboardUi(snapshot);
                        });

                        // Dashboard 是低频运维状态，500ms 刷新就够了
                        await Task.Delay(500, token);
                    }
                }
                catch (TaskCanceledException)
                {
                }
                catch (ObjectDisposedException)
                {
                }
                catch (Exception ex)
                {
                    AddLogByDispatcher("[Dashboard ERROR] " + ex.Message);
                }
            });
        }
        public void StartRtdeMonitor(string ip)
        {
            global_socket.PrepareRtdeClient();

            UrRtdeClient client = global_socket.RtdeClient;
            CancellationToken token = global_socket.RtdeCts.Token;

            client.LogMessage += msg =>
            {
                AddLogByDispatcher(msg);
            };

            client.StateReceived += state =>
            {
                global_socket.UpdateRtdeState(state);

                bool shouldPostUi = false;
                DateTime now = DateTime.Now;

                lock (_rtdeUiThrottleLock)
                {
                    if ((now - _lastRtdeUiPostTime).TotalMilliseconds >= RtdeUiUpdateIntervalMs)
                    {
                        _lastRtdeUiPostTime = now;
                        shouldPostUi = true;
                    }
                }

                if (!shouldPostUi)
                    return;

                Dispatcher.BeginInvoke(
                    new Action(() =>
                    {
                        UrRtdeState latest = global_socket.GetLatestRtdeState();
                        if (latest != null)
                        {
                            UpdateRtdeUi(latest);
                        }
                    }),
                    DispatcherPriority.Background
                );
            };

            _ = Task.Run(async () =>
            {
                try
                {
                    await client.ConnectAndStartAsync(
                        ip,
                        global_socket.RtdePort,
                        125.0,
                        token
                    );
                }
                catch (TaskCanceledException)
                {
                }
                catch (ObjectDisposedException)
                {
                }
                catch (Exception ex)
                {
                    AddLogByDispatcher("[RTDE ERROR] " + ex.Message);

                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        OnRtdeDisconnectedUi();
                    }));
                }
            });
        }
        public void UpdateDashboardUi(UrDashboardSnapshot s)
        {
            if (s == null)
                return;

            TxtRobotMode.Text = $"{TranslateOperationalMode(s.OperationalMode)} / {s.RobotMode}";

            if (s.RobotMode.IndexOf("RUNNING", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                TxtEnableState.Text = "● 已使能";
                TxtEnableState.Foreground = System.Windows.Media.Brushes.Green;
            }
            else
            {
                TxtEnableState.Text = "● 未使能";
                TxtEnableState.Foreground = System.Windows.Media.Brushes.OrangeRed;
            }

            if (!s.IsEmergencyOrFault)
            {
                TxtEmergencyState.Text = "● 正常";
                TxtEmergencyState.Foreground = System.Windows.Media.Brushes.Green;
            }
            else
            {
                TxtEmergencyState.Text = "● " + s.SafetyStatus;
                TxtEmergencyState.Foreground = System.Windows.Media.Brushes.Red;
            }

            if (s.IsProgramRunning)
            {
                TxtRunState.Text = "● 运行中";
                TxtRunState.Foreground = System.Windows.Media.Brushes.Green;
            }
            else
            {
                TxtRunState.Text = "● 空闲";
                TxtRunState.Foreground = System.Windows.Media.Brushes.Orange;
            }

            TxtProgramState.Text = SimplifyProgramState(s.ProgramState);
            TxtSystemTime.Text = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
        }
        public string TranslateOperationalMode(string mode)
        {
            if (string.IsNullOrWhiteSpace(mode))
                return "未知";

            string upper = mode.ToUpperInvariant();

            if (upper.Contains("AUTOMATIC"))
                return "自动";

            if (upper.Contains("MANUAL"))
                return "手动";

            if (upper.Contains("NONE"))
                return "未设置";

            return mode;
        }

        public string SimplifyProgramState(string raw)
        {
            if (string.IsNullOrWhiteSpace(raw))
                return "未知";

            string upper = raw.ToUpperInvariant();

            if (upper.StartsWith("STOPPED"))
                return "无程序运行";

            if (upper.StartsWith("PLAYING"))
                return "程序运行中";

            if (upper.StartsWith("PAUSED"))
                return "程序暂停";

            return raw;
        }

        public void UpdateRtdeUi(UrRtdeState s)
        {
            if (s == null)
                return;

            _hasReceivedActualRobotState = true;

            // =====================================================
            // 1. 更新当前关节角表格
            // =====================================================
            double[] actualQDeg = ConvertActualQToDeg6(s.ActualQ);

            if (actualQDeg != null)
            {
                UpdateJointTableCurrentOnly(actualQDeg);

                if (_simulationDriveMode == SimulationDriveMode.ActualRobot)
                {
                    if (global_variable.simulationRealTime != null &&
                        global_variable.simulationRealTime.IsInitialized)
                    {
                        global_variable.simulationRealTime.UpdateJointAngles(
                            global_variable._actualJointDegForTable
                        );
                    }
                }
            }

            // =====================================================
            // 2. 更新当前 TCP 位姿
            // actual_TCP_pose = [x,y,z,rx,ry,rz]
            // x/y/z 单位 m，rx/ry/rz 是旋转向量 rad
            // =====================================================
            double[] pose = s.ActualTcpPose;

            if (pose != null && pose.Length >= 6)
            {
                TxtCurrentX.Text = (pose[0] * 1000.0).ToString("F2");
                TxtCurrentY.Text = (pose[1] * 1000.0).ToString("F2");
                TxtCurrentZ.Text = (pose[2] * 1000.0).ToString("F2");

                TxtCurrentRx.Text = pose[3].ToString("F4");
                TxtCurrentRy.Text = pose[4].ToString("F4");
                TxtCurrentRz.Text = pose[5].ToString("F4");
            }

            // =====================================================
            // 3. TCP速度 / 加速度
            // =====================================================
            TxtTcpSpeed.Text = $"{s.TcpLinearSpeedMmPerSec:F1} mm/s";
            TxtTcpAcceleration.Text = $"{s.TcpLinearAccelerationMmPerSec2:F1} mm/s2";

            // =====================================================
            // 4. 控制频率 / 周期
            // =====================================================
            if (global_socket.LastRtdeTimestamp > 0.0 &&
                s.Timestamp > global_socket.LastRtdeTimestamp)
            {
                double dt = s.Timestamp - global_socket.LastRtdeTimestamp;

                if (dt > 1e-6)
                {
                    double hz = 1.0 / dt;

                    TxtControlFrequency.Text = $"{hz:F0} Hz";
                    TxtStatusControlPeriod.Text = $"控制周期：{1000.0 / hz:F1} ms";
                }
            }

            global_socket.LastRtdeTimestamp = s.Timestamp;
            global_socket.LastRtdeReceiveTime = DateTime.Now;

            // =====================================================
            // 5. RTDE 状态覆盖部分 Dashboard 状态
            // =====================================================
            string robotModeText = UrRtdeText.RobotModeToText(s.RobotMode);
            string safetyText = UrRtdeText.SafetyStatusToText(s.SafetyStatus);
            string runtimeText = UrRtdeText.RuntimeStateToText(s.RuntimeState);

            if (s.RobotMode.HasValue)
            {
                if (s.RobotMode.Value == 7)
                {
                    TxtEnableState.Text = "● 已使能";
                    TxtEnableState.Foreground = System.Windows.Media.Brushes.Green;
                }
                else
                {
                    TxtEnableState.Text = "● " + robotModeText;
                    TxtEnableState.Foreground = System.Windows.Media.Brushes.OrangeRed;
                }
            }

            if (s.IsSafetyNormal || safetyText == "NORMAL")
            {
                TxtEmergencyState.Text = "● 正常";
                TxtEmergencyState.Foreground = System.Windows.Media.Brushes.Green;
            }
            else if (s.IsEmergencyStopped ||
                     s.IsRobotEmergencyStopped ||
                     s.IsSystemEmergencyStopped ||
                     s.IsProtectiveStopped ||
                     s.IsSafetyFault ||
                     s.IsSafetyViolation)
            {
                TxtEmergencyState.Text = "● " + safetyText;
                TxtEmergencyState.Foreground = System.Windows.Media.Brushes.Red;
            }

            if (s.IsProgramRunning || runtimeText == "RUNNING")
            {
                TxtRunState.Text = "● 运行中";
                TxtRunState.Foreground = System.Windows.Media.Brushes.Green;
            }
            else
            {
                TxtRunState.Text = "● 空闲";
                TxtRunState.Foreground = System.Windows.Media.Brushes.Orange;
            }

            TxtSystemTime.Text = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
        }
        public double[] ConvertActualQToDeg6(double[] actualQRad)
        {
            if (actualQRad == null || actualQRad.Length < 6)
                return null;

            double[] qDeg6 = new double[6];

            for (int i = 0; i < 6; i++)
            {
                qDeg6[i] = actualQRad[i] * 180.0 / Math.PI;
            }

            return qDeg6;
        }
        public void AddLogByDispatcher(string msg)
        {
            Dispatcher.BeginInvoke(new Action(() =>
            {
                AddLog(msg);
            }));
        }
        public void OnRtdeDisconnectedUi()
        {
            AddLog("[WARN] RTDE状态连接已断开");

            _hasReceivedActualRobotState = false;
            _simulationDriveMode = SimulationDriveMode.TargetCommand;

            BtnViewActualRobot.Content = "仿";
            BtnViewActualRobot.ToolTip = "当前：UI目标仿真。点击切换到真实机器人反馈仿真";

            if (TxtSimulationTitle != null)
            {
                TxtSimulationTitle.Text = "机器人仿真显示：UI目标模式";
                TxtSimulationTitle.Foreground = System.Windows.Media.Brushes.Black;
            }

            TxtRobotCommState.Text = "RTDE断开";
            TxtRobotCommState.Foreground = System.Windows.Media.Brushes.Red;

            RefreshSimulationByCurrentMode();
        }

        private void StartNetworkMonitor(string ip)
        {
            global_socket.PrepareNetworkMonitor();

            CancellationToken token = global_socket.NetworkCts.Token;

            _ = Task.Run(async () =>
            {
                using (Ping ping = new Ping())
                {
                    while (!token.IsCancellationRequested)
                    {
                        try
                        {
                            global_socket.PingTotalCount++;

                            PingReply reply = await ping.SendPingAsync(ip, 1000);

                            if (reply.Status == IPStatus.Success)
                            {
                                global_socket.LastNetworkDelayMs = reply.RoundtripTime;
                            }
                            else
                            {
                                global_socket.PingLostCount++;
                                global_socket.LastNetworkDelayMs = -1.0;
                            }

                            if (global_socket.PingTotalCount > 0)
                            {
                                global_socket.PacketLossRate =
                                    100.0 * global_socket.PingLostCount / global_socket.PingTotalCount;
                            }

                            await Dispatcher.InvokeAsync(() =>
                            {
                                UpdateNetworkUi();
                            });

                            await Task.Delay(1000, token);
                        }
                        catch (TaskCanceledException)
                        {
                            break;
                        }
                        catch
                        {
                            global_socket.PingLostCount++;

                            if (global_socket.PingTotalCount > 0)
                            {
                                global_socket.PacketLossRate =
                                    100.0 * global_socket.PingLostCount / global_socket.PingTotalCount;
                            }

                            await Dispatcher.InvokeAsync(() =>
                            {
                                TxtNetworkDelay.Text = "网络延迟：异常";
                                TxtPacketLoss.Text = $"丢包率：{global_socket.PacketLossRate:F1} %";
                            });

                            await Task.Delay(1000, token);
                        }
                    }
                }
            });
        }
        private void UpdateNetworkUi()
        {
            if (global_socket.LastNetworkDelayMs >= 0.0)
            {
                TxtNetworkDelay.Text = $"网络延迟：{global_socket.LastNetworkDelayMs:F1} ms";
            }
            else
            {
                TxtNetworkDelay.Text = "网络延迟：超时";
            }

            TxtPacketLoss.Text = $"丢包率：{global_socket.PacketLossRate:F1} %";
        }

        private async void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            string ip = TxtRobotIp.Text.Trim();

            AddLog($"[CMD] 正在连接机器人 IP={ip}");
            AddLog(
                $"[PORT] Motion={global_socket.MotionScriptPort}, " +
                $"Dashboard={global_socket.DashboardPort}, " +
                $"RTDE={global_socket.RtdePort}"
            );

            try
            {
                // 重新连接前关闭全部旧连接
                global_socket.CloseAll();

                // 1. 连接运动控制 Socket
                await global_socket.ConnectMotionSocketAsync(ip);

                AddLog(
                    $"[INFO] 运动控制 Socket 连接成功：" +
                    $"{ip}:{global_socket.MotionScriptPort}"
                );

                TxtConnectionState.Text = "已连接";
                TxtConnectionState.Foreground = System.Windows.Media.Brushes.Green;

                TxtRobotCommState.Text = "已连接";
                TxtRobotCommState.Foreground = System.Windows.Media.Brushes.Green;

                // 2. Dashboard：低频状态
                StartDashboardMonitor(ip);

                // 3. RTDE：高频实时状态
                StartRtdeMonitor(ip);

                // 4. Ping：网络延迟和丢包率
                StartNetworkMonitor(ip);

                AddLog("[INFO] Motion / Dashboard / RTDE / 网络状态监听已启动");
            }
            catch (Exception ex)
            {
                global_socket.CloseAll();

                AddLog("[ERROR] 机器人连接失败：" + ex.Message);

                TxtConnectionState.Text = "未连接";
                TxtConnectionState.Foreground = System.Windows.Media.Brushes.Red;

                TxtRobotCommState.Text = "未连接";
                TxtRobotCommState.Foreground = System.Windows.Media.Brushes.Red;

                System.Windows.MessageBox.Show(
                    "机器人连接失败：\n" + ex.Message,
                    "连接错误",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error
                );
            }
        }

        private void BtnDisconnect_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[CMD] 断开机器人连接");

            try
            {
                global_socket.CloseAll();

                _hasReceivedActualRobotState = false;
                _simulationDriveMode = SimulationDriveMode.TargetCommand;

                BtnViewActualRobot.Content = "仿";
                BtnViewActualRobot.ToolTip = "当前：UI目标仿真。点击切换到真实机器人反馈仿真";

                if (TxtSimulationTitle != null)
                {
                    TxtSimulationTitle.Text = "机器人仿真显示：UI目标模式";
                    TxtSimulationTitle.Foreground = System.Windows.Media.Brushes.Black;
                }

                RefreshSimulationByCurrentMode();

                TxtConnectionState.Text = "未连接";
                TxtConnectionState.Foreground = System.Windows.Media.Brushes.Red;

                TxtRobotCommState.Text = "未连接";
                TxtRobotCommState.Foreground = System.Windows.Media.Brushes.Red;

                TxtRobotMode.Text = "未知";
                TxtEnableState.Text = "● 未使能";
                TxtEnableState.Foreground = System.Windows.Media.Brushes.OrangeRed;
                TxtEmergencyState.Text = "● 未连接";
                TxtEmergencyState.Foreground = System.Windows.Media.Brushes.OrangeRed;
                TxtRunState.Text = "● 未连接";
                TxtRunState.Foreground = System.Windows.Media.Brushes.OrangeRed;
                TxtProgramState.Text = "无程序运行";

                TxtTcpSpeed.Text = "0.0 mm/s";
                TxtTcpAcceleration.Text = "0.0 mm/s2";
                TxtControlFrequency.Text = "0 Hz";
                TxtStatusControlPeriod.Text = "控制周期：-- ms";
                TxtNetworkDelay.Text = "网络延迟：-- ms";
                TxtPacketLoss.Text = "丢包率：-- %";

                AddLog("[INFO] 机器人 Socket / Dashboard / RTDE 已全部断开");
            }
            catch (Exception ex)
            {
                AddLog("[ERROR] 断开连接失败：" + ex.Message);
            }
        }


        private void BtnInitialize_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[CMD] 初始化机器人系统");

            global_variable.SetCustomJointDeg();

            // 从 UI 读取工具参数，而不是写死一个长度，例如 0,0,100
            if (!UpdateGlobalToolFromUi(true))
                return;

            UpdateJointUiFromGlobal();
            InitializeVtkSimulation();
            UpdateRobotFromGlobalJoint();

            AddLog("[SIM] 已恢复默认关节角：90, -90, 90, -90, -90, 90");
        }

        private void BtnZero_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[CMD] 机器人回零");

            global_variable.SetGlobalJointDeg(
                90, -90, 90, -90, -90, 90
            );

            UpdateJointUiFromGlobal();
            UpdateRobotFromGlobalJoint();

            AddLog("[SIM] 仿真机器人已回零");
        }

        private void BtnHome_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[CMD] 机器人回到Home位姿");

            global_variable.SetDefaultJointDeg();

            UpdateJointUiFromGlobal();
            UpdateRobotFromGlobalJoint();

            AddLog("[SIM] 仿真机器人已回到默认Home位姿");
        }

        private void BtnStartSimulation_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[SIM] 启动机器人仿真显示");

            InitializeVtkSimulation();
            UpdateRobotFromGlobalJoint();
        }

        private void BtnStop_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[CMD] 用户点击停止");

            if (_motionCoordinator == null)
            {
                AddLog("[ERROR] 运动调度器未初始化");
                return;
            }

            // 不 await，立即发起停止
            _ = _motionCoordinator.StopAsync();
        }

        private void BtnSetting_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[UI] 打开参数设置");

            // TODO:
            // 打开参数设置窗口
        }

        private void BtnReadPose_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[CMD] 读取当前机器人位姿");

            try
            {
                // 1. 从 UI 读取 7 个关节角，单位 degree
                if (!TryReadDouble(TxtJ1, out double q1)) { AddLog("[ERROR] J1 输入无效"); return; }
                if (!TryReadDouble(TxtJ2, out double q2)) { AddLog("[ERROR] J2 输入无效"); return; }
                if (!TryReadDouble(TxtJ3, out double q3)) { AddLog("[ERROR] J3 输入无效"); return; }
                if (!TryReadDouble(TxtJ4, out double q4)) { AddLog("[ERROR] J4 输入无效"); return; }
                if (!TryReadDouble(TxtJ5, out double q5)) { AddLog("[ERROR] J5 输入无效"); return; }
                if (!TryReadDouble(TxtJ6, out double q6)) { AddLog("[ERROR] J6 输入无效"); return; }

                double[] qDeg = new double[]
                {
                    q1, q2, q3, q4, q5, q6
                };

                // 2. degree 转 rad，因为正运动学函数要求输入弧度
                double[] qRad = qDeg
                    .Select(deg => deg * Math.PI / 180.0)
                    .ToArray();


                // 4. 调用正运动学，获取 T06、T07、T0Tcp
                var fk = Forward.ForwardKinematicsMatrix(qRad);

                // 5. 取 T07
                var T06 = fk;

                if (T06 == null)
                {
                    AddLog("[ERROR] 正运动学计算失败：T07 为空");
                    return;
                }

                // 6. MathNet Matrix<double> 转 double[,]
                double[,] T07Array = T06.ToArray();

                // 7. 4x4 矩阵转 UR PoseArray: { x, y, z, rx, ry, rz }
                double[] poseArray = euler2vector.Matrix4x4ToUrPoseArray(T07Array);

                // 8. 打印 T07 矩阵
                AddLog("[FK] T07 矩阵：");
                AddLog(string.Format(
                    "       [{0:F6}, {1:F6}, {2:F6}, {3:F6}]",
                    T06[0, 0], T06[0, 1], T06[0, 2], T06[0, 3]));
                AddLog(string.Format(
                    "       [{0:F6}, {1:F6}, {2:F6}, {3:F6}]",
                    T06[1, 0], T06[1, 1], T06[1, 2], T06[1, 3]));
                AddLog(string.Format(
                    "       [{0:F6}, {1:F6}, {2:F6}, {3:F6}]",
                    T06[2, 0], T06[2, 1], T06[2, 2], T06[2, 3]));
                AddLog(string.Format(
                    "       [{0:F6}, {1:F6}, {2:F6}, {3:F6}]",
                    T06[3, 0], T06[3, 1], T06[3, 2], T06[3, 3]));

                // 9. 打印 PoseArray
                AddLog(string.Format(
                    "[FK] T07 PoseArray = x={0:F6}, y={1:F6}, z={2:F6}, rx={3:F6}, ry={4:F6}, rz={5:F6}",
                    poseArray[0],
                    poseArray[1],
                    poseArray[2],
                    poseArray[3],
                    poseArray[4],
                    poseArray[5]));

                // 10. 打印成 URScript p[...] 格式
                AddLog(string.Format(
                    "[FK] T07 UR Pose = p[{0:F6},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6}]",
                    poseArray[0],
                    poseArray[1],
                    poseArray[2],
                    poseArray[3],
                    poseArray[4],
                    poseArray[5]));

                // 11. 可选：更新 UI 和 VTK
                UpdateJointUiFromGlobal();
                UpdateRobotFromGlobalJoint();
            }
            catch (Exception ex)
            {
                AddLog("[ERROR] 读取/计算当前机器人位姿失败：" + ex.Message);
            }
        }


        private void BtnSyncSimulation_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[SIM] 同步机器人仿真位姿");

            if (!UpdateGlobalJointFromTextBox(true))
                return;

            UpdateJointUiFromGlobal();
            UpdateRobotFromGlobalJoint();

            AddLog(
                $"[SIM] 已同步仿真关节角: " +
                $"J1={global_variable.globalJointDeg[0]:F2}, " +
                $"J2={global_variable.globalJointDeg[1]:F2}, " +
                $"J3={global_variable.globalJointDeg[2]:F2}, " +
                $"J4={global_variable.globalJointDeg[3]:F2}, " +
                $"J5={global_variable.globalJointDeg[4]:F2}, " +
                $"J6={global_variable.globalJointDeg[5]:F2}"
            );
        }

     

        private void BtnSendJointCommand_Click(object sender, RoutedEventArgs e)
        {
            if (!UpdateGlobalJointFromTextBox(true))
                return;

            // 只更新目标表格，不在这里刷新 VTK
            UpdateJointUiFromGlobal();

            AddLog(
                $"[CMD] 发送关节角: " +
                $"J1={global_variable.globalJointDeg[0]:F2}, " +
                $"J2={global_variable.globalJointDeg[1]:F2}, " +
                $"J3={global_variable.globalJointDeg[2]:F2}, " +
                $"J4={global_variable.globalJointDeg[3]:F2}, " +
                $"J5={global_variable.globalJointDeg[4]:F2}, " +
                $"J6={global_variable.globalJointDeg[5]:F2}"
            );

            var cmd = new RobotMotionCommand
            {
                Source = MotionSourceType.UI,
                CommandType = MotionCommandType.JointTarget,
                JointDeg6 = global_variable.globalJointDeg.ToArray(),

                MoveJVelocity = RobotParameterRuntime.MoveJVelocity,
                MoveJAcceleration = RobotParameterRuntime.MoveJAcceleration,
                MoveJTime = RobotParameterRuntime.MoveJTime,

                // 单点运动强制 r=0，避免 blend 导致不到位
                MoveJBlendRadius = 0.0,

                Description = "UI关节角输入"
            };

            // 不 await，避免按钮事件一直挂在运动等待到位流程上
            _ = _motionCoordinator.ExecuteAsync(cmd);
        }
        //private async void BtnSendJointCommand_Click(object sender, RoutedEventArgs e)
        //{
        //    if (!UpdateGlobalJointFromTextBox(true))
        //        return;

        //    UpdateJointUiFromGlobal();
        //    UpdateRobotFromGlobalJoint();

        //    AddLog(
        //        $"[CMD] 发送关节角: " +
        //        $"J1={global_variable.globalJointDeg[0]:F2}, " +
        //        $"J2={global_variable.globalJointDeg[1]:F2}, " +
        //        $"J3={global_variable.globalJointDeg[2]:F2}, " +
        //        $"J4={global_variable.globalJointDeg[3]:F2}, " +
        //        $"J5={global_variable.globalJointDeg[4]:F2}, " +
        //        $"J6={global_variable.globalJointDeg[5]:F2}, " +
        //        $"J7={global_variable.globalJointDeg[6]:F2}"
        //    );

        //    var cmd = new RobotMotionCommand
        //    {
        //        Source = MotionSourceType.UI,
        //        CommandType = MotionCommandType.JointTarget,
        //        JointDeg6 = global_variable.globalJointDeg.ToArray(),

        //        MoveJVelocity = RobotParameterRuntime.MoveJVelocity,
        //        MoveJAcceleration = RobotParameterRuntime.MoveJAcceleration,
        //        MoveJTime = RobotParameterRuntime.MoveJTime,
        //        MoveJBlendRadius = RobotParameterRuntime.MoveJBlendRadius,

        //        Description = "UI关节角输入"
        //    };

        //    await _motionCoordinator.ExecuteAsync(cmd);
        //}
        private async Task OnOmegaJointCommandAsync(double[] omegaTargetJointDeg6)
        {
            var cmd = new RobotMotionCommand
            {
                Source = MotionSourceType.Omega,
                CommandType = MotionCommandType.JointTarget,
                JointDeg6 = omegaTargetJointDeg6.Take(6).ToArray(),

                MoveJVelocity = RobotParameterRuntime.MoveJVelocity,
                MoveJAcceleration = RobotParameterRuntime.MoveJAcceleration,
                MoveJTime = 0.0,
                MoveJBlendRadius = 0.0,

                Description = "Omega遥操作主手控制"
            };

            await _motionCoordinator.ExecuteAsync(cmd);
        }
        private async Task OnAiJointCommandAsync(double[] aiJointDeg6)
        {
            var cmd = new RobotMotionCommand
            {
                Source = MotionSourceType.AI,
                CommandType = MotionCommandType.JointTarget,
                JointDeg6 = aiJointDeg6.Take(6).ToArray(),

                MoveJVelocity = RobotParameterRuntime.MoveJVelocity,
                MoveJAcceleration = RobotParameterRuntime.MoveJAcceleration,
                MoveJTime = 0.0,
                MoveJBlendRadius = 0.0,

                Description = "AI自动控制指令"
            };

            await _motionCoordinator.ExecuteAsync(cmd);
        }

        private void BtnSolveIkAndMove_Click(object sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtTcpX.Text, out double x) ||
                !double.TryParse(TxtTcpY.Text, out double y) ||
                !double.TryParse(TxtTcpZ.Text, out double z) ||
                !double.TryParse(TxtTcpRx.Text, out double rx) ||
                !double.TryParse(TxtTcpRy.Text, out double ry) ||
                !double.TryParse(TxtTcpRz.Text, out double rz))
            {
                System.Windows.MessageBox.Show(
                    "TCP位姿输入格式错误，请检查 X/Y/Z/Rx/Ry/Rz 是否都是有效数字。",
                    "输入错误",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning
                );

                AddLog("[ERROR] TCP位姿输入格式错误");
                return;
            }

            try
            {
                AddLog(
                    $"[IK] TCP逆解目标位姿: " +
                    $"X={x:F2}, Y={y:F2}, Z={z:F2}, " +
                    $"Rx={rx:F2}, Ry={ry:F2}, Rz={rz:F2}"
                );

                Matrix<double> T0TcpTarget = Forward.FixedXYZRPYDegToTransform(
                    x, y, z,
                    rx, ry, rz
                );

                double[] bestJointDeg = Inverse.SolveBestIKForTcp(
                    T0TcpTarget,
                    global_variable.globalJointDeg,
                    1.0,
                    1.0
                );

                if (bestJointDeg == null)
                {
                    AddLog("[IK] 六轴逆解失败：当前目标位姿无可达解");
                    System.Windows.MessageBox.Show(
                        "六轴逆解失败：当前目标位姿无可达解。",
                        "IK失败",
                        MessageBoxButton.OK,
                        MessageBoxImage.Warning
                    );
                    return;
                }

                global_variable.SetGlobalJointDeg(
                    bestJointDeg[0],
                    bestJointDeg[1],
                    bestJointDeg[2],
                    bestJointDeg[3],
                    bestJointDeg[4],
                    bestJointDeg[5]
                );

                UpdateJointUiFromGlobal();
                UpdateRobotFromGlobalJoint();

                AddLog(
                    $"[IK] 六轴逆解成功: " +
                    $"J1={bestJointDeg[0]:F2}, " +
                    $"J2={bestJointDeg[1]:F2}, " +
                    $"J3={bestJointDeg[2]:F2}, " +
                    $"J4={bestJointDeg[3]:F2}, " +
                    $"J5={bestJointDeg[4]:F2}, " +
                    $"J6={bestJointDeg[5]:F2}"
                );
            }
            catch (Exception ex)
            {
                AddLog("[IK] 六轴逆解异常：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "六轴逆解异常：\n" + ex.Message,
                    "IK异常",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error
                );
            }
        }

        private void BtnApplyToolRcm_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[RCM] 应用工具参数与RCM设置");

            if (!UpdateGlobalToolFromUi(true))
                return;
            try
            {
                double toolX = double.Parse(TxtToolX.Text);
                double toolY = double.Parse(TxtToolY.Text);
                double toolZ = double.Parse(TxtToolZ.Text);

                double rcmX = double.Parse(TxtRcmX.Text);
                double rcmY = double.Parse(TxtRcmY.Text);
                double rcmZ = double.Parse(TxtRcmZ.Text);

                // 写入运行时 RCM 参数
                RobotParameterRuntime.RcmX = rcmX;
                RobotParameterRuntime.RcmY = rcmY;
                RobotParameterRuntime.RcmZ = rcmZ;
                RobotParameterRuntime.RcmEnabled = ChkRcmMode.IsChecked == true;

                // 1. 写入全局变量  无
                //global_variable.Tool_X = toolX;
                //global_variable.Tool_Y = toolY;
                //global_variable.Tool_Z = toolZ;

                //global_variable.RCM_X = rcmX;
                //global_variable.RCM_Y = rcmY;
                //global_variable.RCM_Z = rcmZ;

                // 2. 保存到本地配置
                Properties.Settings.Default.ToolX = toolX;
                Properties.Settings.Default.ToolY = toolY;
                Properties.Settings.Default.ToolZ = toolZ;

                Properties.Settings.Default.RcmX = rcmX;
                Properties.Settings.Default.RcmY = rcmY;
                Properties.Settings.Default.RcmZ = rcmZ;

                Properties.Settings.Default.RcmMode = ChkRcmMode.IsChecked == true;

                Properties.Settings.Default.Save();

                // 新增：刷新VTK中的蓝色RCM点
                RefreshRcmPointInVtk();

                AddLog($"[参数] 工具/RCM参数已保存：Tool=({toolX:F2}, {toolY:F2}, {toolZ:F2}), RCM=({rcmX:F2}, {rcmY:F2}, {rcmZ:F2})");
            }
            catch (Exception ex)
            {
                AddLog("[参数] 工具/RCM参数保存失败：" + ex.Message);
            }

            global_variable.SyncToolToForward();

            // 工具参数改变后，重新计算 TCP 位姿，并刷新目标点
            UpdateRobotFromGlobalJoint();

            AddLog("[RCM] 工具参数已应用到正运动学模型");
        }


        private void BtnClearLog_Click(object sender, RoutedEventArgs e)
        {
            TxtLogBox.Clear();
            AddLog("[LOG] 日志已清空");
        }

        private void BtnExportLog_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[LOG] 导出日志");

            // TODO:
            // 保存 TxtLogBox.Text 到本地 txt 文件
        }

        private void TabAIControlPage_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[UI] 切换到AI自动控制页面");
        }

       
        private void BtnViewActualRobot_Click(object sender, RoutedEventArgs e)
        {
            if (_simulationDriveMode == SimulationDriveMode.TargetCommand)
            {
                // 准备从【仿】切换到【实】
                if (!global_socket.socketsuccess)
                {
                    AddLog("[WARN] 尚未连接机器人，无法切换到【实】模式。当前保持【仿】模式。");

                    System.Windows.MessageBox.Show(
                        "请先连接机器人，再切换到真实机器人反馈模式。",
                        "未连接机器人",
                        MessageBoxButton.OK,
                        MessageBoxImage.Warning
                    );

                    return;
                }

                if (!global_socket.HasFreshRtdeState(1500))
                {
                    AddLog("[WARN] RTDE 尚未收到真实机器人关节角，无法切换到【实】模式。");

                    System.Windows.MessageBox.Show(
                        "已经连接机器人，但 RTDE 还没有收到实时关节角。\n请等待 1~2 秒后再切换。",
                        "RTDE 状态未就绪",
                        MessageBoxButton.OK,
                        MessageBoxImage.Warning
                    );

                    return;
                }

                _simulationDriveMode = SimulationDriveMode.ActualRobot;

                BtnViewActualRobot.Content = "实";
                BtnViewActualRobot.ToolTip = "当前：真实机器人反馈仿真。点击切换到 UI 目标仿真";

                if (TxtSimulationTitle != null)
                {
                    TxtSimulationTitle.Text = "机器人仿真显示：真实反馈模式";
                    TxtSimulationTitle.Foreground = System.Windows.Media.Brushes.Green;
                }

                AddLog("[SIM] 已切换到【实】模式：VTK 由 RTDE actual_q 驱动。");
            }
            else
            {
                // 从【实】切换回【仿】
                _simulationDriveMode = SimulationDriveMode.TargetCommand;

                BtnViewActualRobot.Content = "仿";
                BtnViewActualRobot.ToolTip = "当前：UI目标仿真。点击切换到真实机器人反馈仿真";

                if (TxtSimulationTitle != null)
                {
                    TxtSimulationTitle.Text = "机器人仿真显示：UI目标模式";
                    TxtSimulationTitle.Foreground = System.Windows.Media.Brushes.Black;
                }

                AddLog("[SIM] 已切换到【仿】模式：VTK 由 UI 目标关节角驱动。");
            }

            RefreshSimulationByCurrentMode();
        }
        public void UpdateTcpPoseFromForwardKinematics()
        {
            try
            {
                if (!global_variable.globalFkValid)
                    return;

                Matrix<double> T0Tool = global_variable.globalT0Tcp;

                double x = T0Tool[0, 3];
                double y = T0Tool[1, 3];
                double z = T0Tool[2, 3];

                double[] rpyDeg = MatrixToXYZRPYDeg(T0Tool);

                TxtTcpX.Text = x.ToString("F2");
                TxtTcpY.Text = y.ToString("F2");
                TxtTcpZ.Text = z.ToString("F2");

                TxtTcpRx.Text = rpyDeg[0].ToString("F2");
                TxtTcpRy.Text = rpyDeg[1].ToString("F2");
                TxtTcpRz.Text = rpyDeg[2].ToString("F2");

                simulation_mode.SetTargetPoint(x, y, z);
            }
            catch (Exception ex)
            {
                AddLog("[FK] TCP位姿刷新失败：" + ex.Message);
            }
        }
        public void UpdateT06PoseFromForwardKinematics()
        {
            try
            {
                if (!global_variable.globalFkValid)
                    return;

                Matrix<double> T06 = global_variable.globalT06;

                double x = T06[0, 3];
                double y = T06[1, 3];
                double z = T06[2, 3];

                double[] rpyDeg = MatrixToXYZRPYDeg(T06);

                //double[] urRotRad = MatrixToURRotVectorRad(T06);

                TxtJoint6PoseX.Text = x.ToString("F2");
                TxtJoint6PoseY.Text = y.ToString("F2");
                TxtJoint6PoseZ.Text = z.ToString("F2");

                TxtJoint6PoseRx.Text = rpyDeg[0].ToString("F2");
                TxtJoint6PoseRy.Text = rpyDeg[1].ToString("F2");
                TxtJoint6PoseRz.Text = rpyDeg[2].ToString("F2");
            }
            catch (Exception ex)
            {
                AddLog("[FK] T06位姿刷新失败：" + ex.Message);
            }
        }
        public void UpdateT07PoseFromForwardKinematics()
        {
            try
            {
                if (!global_variable.globalFkValid)
                    return;

                Matrix<double> T07 = global_variable.globalT07;

                double x = T07[0, 3];
                double y = T07[1, 3];
                double z = T07[2, 3];

                double[] rpyDeg = MatrixToXYZRPYDeg(T07);

                TxtJoint7PoseX.Text = x.ToString("F2");
                TxtJoint7PoseY.Text = y.ToString("F2");
                TxtJoint7PoseZ.Text = z.ToString("F2");

                TxtJoint7PoseRx.Text = rpyDeg[0].ToString("F2");
                TxtJoint7PoseRy.Text = rpyDeg[1].ToString("F2");
                TxtJoint7PoseRz.Text = rpyDeg[2].ToString("F2");
            }
            catch (Exception ex)
            {
                AddLog("[FK] T07位姿刷新失败：" + ex.Message);
            }
        }
        public double[] MatrixToXYZRPYDeg(Matrix<double> T)
        {
            // 旋转矩阵 R = Rx(rx) * Ry(ry) * Rz(rz)
            double r00 = T[0, 0];
            double r01 = T[0, 1];
            double r02 = T[0, 2];

            double r12 = T[1, 2];
            double r22 = T[2, 2];

            double ry = Math.Asin(Clamp(r02, -1.0, 1.0));

            double cosRy = Math.Cos(ry);

            double rx;
            double rz;

            if (Math.Abs(cosRy) > 1e-8)
            {
                rx = Math.Atan2(-r12, r22);
                rz = Math.Atan2(-r01, r00);
            }
            else
            {
                // 奇异情况：ry 接近 ±90°
                // 此时 rx 和 rz 会耦合，这里固定 rz=0，保留一个稳定解
                rx = Math.Atan2(T[1, 0], T[1, 1]);
                rz = 0.0;
            }

            return new double[]
            {
                rx * 180.0 / Math.PI,
                ry * 180.0 / Math.PI,
                rz * 180.0 / Math.PI
            };
        }
        public double[] MatrixToURRotVectorRad(Matrix<double> T)
        {
            double r00 = T[0, 0], r01 = T[0, 1], r02 = T[0, 2];
            double r10 = T[1, 0], r11 = T[1, 1], r12 = T[1, 2];
            double r20 = T[2, 0], r21 = T[2, 1], r22 = T[2, 2];

            double trace = r00 + r11 + r22;
            double cosTheta = Clamp((trace - 1.0) / 2.0, -1.0, 1.0);
            double theta = Math.Acos(cosTheta);

            // 接近零旋转
            if (theta < 1e-10)
            {
                return new double[]
                {
                    0.5 * (r21 - r12),
                    0.5 * (r02 - r20),
                    0.5 * (r10 - r01)
                };
            }

            // 普通情况
            if (Math.Abs(Math.PI - theta) > 1e-6)
            {
                double k = theta / (2.0 * Math.Sin(theta));

                return new double[]
                {
                    k * (r21 - r12),
                    k * (r02 - r20),
                    k * (r10 - r01)
                };
            }

            // 接近 180° 时的稳定处理
            double x, y, z;

            if (r00 >= r11 && r00 >= r22)
            {
                x = Math.Sqrt(Math.Max(0.0, (r00 + 1.0) / 2.0));
                y = (r01 + r10) / (4.0 * x);
                z = (r02 + r20) / (4.0 * x);
            }
            else if (r11 >= r22)
            {
                y = Math.Sqrt(Math.Max(0.0, (r11 + 1.0) / 2.0));
                x = (r01 + r10) / (4.0 * y);
                z = (r12 + r21) / (4.0 * y);
            }
            else
            {
                z = Math.Sqrt(Math.Max(0.0, (r22 + 1.0) / 2.0));
                x = (r02 + r20) / (4.0 * z);
                y = (r12 + r21) / (4.0 * z);
            }

            double norm = Math.Sqrt(x * x + y * y + z * z);
            if (norm < 1e-10)
            {
                return new double[] { 0.0, 0.0, 0.0 };
            }

            x /= norm;
            y /= norm;
            z /= norm;

            return new double[]
            {
                theta * x,
                theta * y,
                theta * z
            };
        }

        private void SimulationDisplayOption_Changed(object sender, RoutedEventArgs e)
        {
            if (!_uiReady)
                return;

            simulation_mode.SetDisplayOptions(
                ChkShowAxis.IsChecked == true,
                ChkShowGrid.IsChecked == true,
                ChkShowWorkspace.IsChecked == true,
                ChkShowTarget.IsChecked == true,
                ChkShowJointAxis.IsChecked == true
            );
        }
        private double GetSelectedGridSizeMm()
        {
            if (CmbGridSize == null)
                return 100.0;

            if (CmbGridSize.SelectedItem is ComboBoxItem item)
            {
                string text = item.Content?.ToString() ?? "100mm";
                text = text.Replace("mm", "").Trim();

                if (double.TryParse(text, out double value))
                    return value;
            }

            return 100.0;
        }
        private void CmbGridSize_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (!_uiReady)
                return;

            double gridSizeMm = GetSelectedGridSizeMm();

            simulation_mode.SetGridSize(gridSizeMm);

            AddLog($"[SIM] 网格大小已切换为 {gridSizeMm:F0} mm");
        }
        private string GetSelectedViewModeText()
        {
            if (CmbViewMode == null)
                return "自由";

            if (CmbViewMode.SelectedItem is ComboBoxItem item)
            {
                return item.Content?.ToString() ?? "自由";
            }

            return "自由";
        }
        private void CmbViewMode_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (!_uiReady)
                return;

            string viewMode = GetSelectedViewModeText();

            simulation_mode.SetCameraView(viewMode);

            AddLog($"[SIM] 视角已切换为：{viewMode}");
        }

        private void BtnViewRotate_Click(object sender, RoutedEventArgs e)
        {
            simulation_mode.SetRotateMode();

            VtkPanOverlay.Visibility = Visibility.Collapsed;

            BtnViewRotate.FontWeight = FontWeights.Bold;
            BtnViewPan.FontWeight = FontWeights.Normal;

            AddLog("[SIM] 鼠标模式切换为：左键拖动旋转");
        }
        private void BtnViewPan_Click(object sender, RoutedEventArgs e)
        {
            simulation_mode.SetPanMode();

            // 不再依赖 WPF 透明覆盖层
            VtkPanOverlay.Visibility = Visibility.Collapsed;

            BtnViewRotate.FontWeight = FontWeights.Normal;
            BtnViewPan.FontWeight = FontWeights.Bold;

            AddLog("[SIM] 鼠标模式切换为：左键拖动平移");
        }

        private void BtnViewZoomIn_Click(object sender, RoutedEventArgs e)
        {
            simulation_mode.ZoomIn();

            AddLog("[SIM] 视角放大");
        }

        private void BtnViewZoomOut_Click(object sender, RoutedEventArgs e)
        {
            simulation_mode.ZoomOut();

            AddLog("[SIM] 视角缩小");
        }
        private void VtkPanOverlay_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            _isVtkPanning = true;
            _lastPanPoint = e.GetPosition(VtkPanOverlay);
            VtkPanOverlay.CaptureMouse();
        }

        private void VtkPanOverlay_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if (!_isVtkPanning)
                return;

            System.Windows.Point current = e.GetPosition(VtkPanOverlay);

            int dx = (int)(current.X - _lastPanPoint.X);
            int dy = (int)(current.Y - _lastPanPoint.Y);

            _lastPanPoint = current;

            simulation_mode.PanCameraByPixels(dx, dy);
        }

        private void VtkPanOverlay_MouseLeftButtonUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            _isVtkPanning = false;
            VtkPanOverlay.ReleaseMouseCapture();
        }

        private void BtnSolveIkJoint6AndMove_Click(object sender, RoutedEventArgs e)
        {
            // TODO：读取 TxtJoint6PoseX/Y/Z/Rx/Ry/Rz
            // 然后求解第六关节位姿对应的逆运动学
        }

        private void BtnSolveIkJoint7AndMove_Click(object sender, RoutedEventArgs e)
        {
            // TODO：读取 TxtJoint7PoseX/Y/Z/Rx/Ry/Rz
            // 然后求解第七关节位姿对应的逆运动学
        }
        private void TabStatusPage_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[UI] 打开机器人状态监测与参数设定界面");

            if (_statusParameterWindow == null)
            {
                _statusParameterWindow = new robParameter();
                _statusParameterWindow.Owner = this;

                _statusParameterWindow.Closed += (s, args) =>
                {
                    _statusParameterWindow = null;
                };

                _statusParameterWindow.Show();
            }
            else
            {
                if (_statusParameterWindow.WindowState == WindowState.Minimized)
                {
                    _statusParameterWindow.WindowState = WindowState.Normal;
                }

                _statusParameterWindow.Activate();
            }
        }

        private void calibrationPage_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[UI] 打开机器人状态监测与参数设定界面");

            if (_calibrationWindow == null)
            {
                _calibrationWindow = new calibrationWindow();
                _calibrationWindow.Owner = this;

                _calibrationWindow.Closed += (s, args) =>
                {
                    _calibrationWindow = null;
                };

                _calibrationWindow.Show();
            }
            else
            {
                if (_calibrationWindow.WindowState == WindowState.Minimized)
                {
                    _calibrationWindow.WindowState = WindowState.Normal;
                }

                _calibrationWindow.Activate();
            }
        }
        /// <summary>
        /// 碰撞检测开关改变事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CollisionDetection_Changed(object sender, RoutedEventArgs e)
        {
            if (!_uiReady)
                return;

            bool enabled = ChkCollisionDetection.IsChecked == true;

            simulation_mode.SetCollisionDetectionEnabled(enabled);

            if (enabled)
            {
                AddLog("[COLLISION] 碰撞检测已开启，当前使用包围盒检测。");
            }
            else
            {
                AddLog("[COLLISION] 碰撞检测已关闭。");
            }
        }
        private void CollisionBounds_Changed(object sender, RoutedEventArgs e)
        {
            if (!_uiReady)
                return;

            bool visible = ChkShowCollisionBounds.IsChecked == true;

            if (visible && ChkCollisionDetection.IsChecked != true)
            {
                ChkCollisionDetection.IsChecked = true;
            }

            if (global_variable.simulationRealTime != null &&
                global_variable.simulationRealTime.IsInitialized)
            {
                global_variable.simulationRealTime.SetCollisionBoundsVisible(visible);
            }

            AddLog(visible ? "[COLLISION] 已显示碰撞包围盒。" : "[COLLISION] 已隐藏碰撞包围盒。");
        }


        /// <summary>
        /// RCM
        /// </summary>
        private DispatcherTimer _rcmDemoTimer;
        private double _rcmDemoPhaseDeg = 0.0;
        private double[] _rcmDemoCenterAxis = null;
        private double _rcmDemoInsertionMm = 150;
        private const double RcmDemoConeAngleDeg = 10.0;
        private const double RcmDemoPhaseStepDeg = 2.0;
        private const int RcmDemoIntervalMs = 50;

        private void BtnStartRcmDemo_Click(object sender, RoutedEventArgs e)
        {
            StartRcmDemoInVtk();
        }

        private void BtnStopRcmDemo_Click(object sender, RoutedEventArgs e)
        {
            StopRcmDemoInVtk();
        }

        private void StartRcmDemoInVtk()
        {
            AddLog("[RCM DEMO] 纯六轴模式下已禁用 RCM 演示。");
            System.Windows.MessageBox.Show(
                "当前已切换为纯六轴模式，RCM 演示依赖原七轴逆解，已禁用。",
                "RCM演示不可用",
                MessageBoxButton.OK,
                MessageBoxImage.Information
            );
        }
        private void RefreshRcmPointInVtk()
        {
            try
            {
                if (global_variable.simulationRealTime == null ||
                    !global_variable.simulationRealTime.IsInitialized)
                {
                    return;
                }

                global_variable.simulationRealTime.SetRcmPoint(
                    RobotParameterRuntime.RcmX,
                    RobotParameterRuntime.RcmY,
                    RobotParameterRuntime.RcmZ
                );
            }
            catch (Exception ex)
            {
                AddLog("[RCM] RCM点显示刷新失败：" + ex.Message);
            }
        }
        private void RcmDemoTimer_Tick(object sender, EventArgs e)
        {
            StopRcmDemoInVtk();
        }
        private void StopRcmDemoInVtk()
        {
            if (_rcmDemoTimer != null)
                _rcmDemoTimer.Stop();

            AddLog("[RCM DEMO] 已停止。");
        }

    }

    public class JointStateRow
    {
        public string Joint { get; set; }
        public string Current { get; set; }
        public string Target { get; set; }
        public string Error { get; set; }
    }
}