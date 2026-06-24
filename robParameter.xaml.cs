using System;
using System.Collections.ObjectModel;
using System.Globalization;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using WpfRobot.inquiry;
using System.Collections.Generic;


namespace WpfRobot
{
    /// <summary>
    /// 机器人状态监测与参数设定窗口。
    ///
    /// 注意：
    /// 1. 本窗口不再新建 Dashboard / RTDE / Ping 连接；
    /// 2. MainWindow 负责启动 Dashboard、RTDE、网络监测；
    /// 3. 本窗口只从 global_socket 中读取最新状态并刷新 UI；
    /// 4. 这样可以避免多个 RTDE 客户端同时连接 30004 端口。
    /// </summary>
    public partial class robParameter : Window
    {
        private readonly DispatcherTimer _uiTimer;
        private double _lastRtdeTimestampInThisWindow = -1.0;

        public ObservableCollection<StatusJointRow> StatusJointRows { get; } =
            new ObservableCollection<StatusJointRow>();

        private static readonly System.Windows.Media.Brush BrushGreen = new SolidColorBrush(System.Windows.Media.Color.FromRgb(34, 164, 71));
        private static readonly System.Windows.Media.Brush BrushRed = new SolidColorBrush(System.Windows.Media.Color.FromRgb(229, 57, 53));
        private static readonly System.Windows.Media.Brush BrushOrange = new SolidColorBrush(System.Windows.Media.Color.FromRgb(245, 158, 11));
        private static readonly System.Windows.Media.Brush BrushBlue = new SolidColorBrush(System.Windows.Media.Color.FromRgb(47, 102, 233));
        private static readonly System.Windows.Media.Brush BrushGray = new SolidColorBrush(System.Windows.Media.Color.FromRgb(184, 192, 204));


        // 波形图纵轴最大延迟，单位 ms。
        // 超过这个值会被压到顶部
        private readonly Queue<double> _networkDelayHistory = new Queue<double>();
        private const int NetworkDelayHistoryMaxCount = 50;
        private const double NetworkDelayChartMaxMs = 100.0;

        public robParameter()
        {
            InitializeComponent();

            InitJointTable();
            BindButtonEvents();

            _uiTimer = new DispatcherTimer
            {
                // 只刷新 UI，不影响 RTDE 接收频率。
                // RTDE 125Hz 接收，但界面 10Hz 刷新即可。
                Interval = TimeSpan.FromMilliseconds(100)
            };

            _uiTimer.Tick += UiTimer_Tick;
            Loaded += RobParameter_Loaded;
            Closed += RobParameter_Closed;

            LoadParameterTextBoxesFromRuntime();
        }

        private void RobParameter_Loaded(object sender, RoutedEventArgs e)
        {
            _uiTimer.Start();

            AddStatusLog("[INFO] 状态监测窗口已打开");
            RefreshAllUi();
        }

        private void RobParameter_Closed(object sender, EventArgs e)
        {
            try
            {
                _uiTimer.Stop();
            }
            catch
            {
            }
        }

        private void UiTimer_Tick(object sender, EventArgs e)
        {
            RefreshAllUi();
        }

        private void RefreshAllUi()
        {
            UpdateConnectionUi();
            UpdateDashboardSection(global_socket.GetLatestDashboardSnapshot());
            UpdateRtdeSection(global_socket.GetLatestRtdeState());
            UpdateNetworkSection();
            UpdateSystemTime();
        }

        private void InitJointTable()
        {
            StatusJointRows.Clear();

            for (int i = 1; i <= 6; i++)
            {
                StatusJointRows.Add(new StatusJointRow
                {
                    Joint = "J" + i,
                    ActualDeg = "0.00",
                    SpeedRad = "0.000",
                    CurrentA = "0.000",
                    TemperatureC = "0.0"
                });
            }

            // 如果你在 XAML 中把静态关节 Grid 换成了 DataGrid x:Name="DgRtdeJointState"，
            // 这里会自动绑定；如果没换，不影响其他 UI 更新。
            if (FindName("DgRtdeJointState") is DataGrid dg)
            {
                dg.ItemsSource = StatusJointRows;
            }
        }

        private void BindButtonEvents()
        {
            if (BtnApplyRobotParams != null)
                BtnApplyRobotParams.Click += BtnApplyRobotParams_Click;

            if (BtnRestoreRobotParams != null)
                BtnRestoreRobotParams.Click += BtnRestoreRobotParams_Click;

            if (BtnSaveRobotParams != null)
                BtnSaveRobotParams.Click += BtnSaveRobotParams_Click;
        }

        // =========================================================
        // 1. 连接状态
        // =========================================================
        private void UpdateConnectionUi()
        {
            bool socketConnected = global_socket.socketsuccess;
            bool rtdeFresh = global_socket.HasFreshRtdeState(1500);

            if (TxtConnectionState != null)
            {
                TxtConnectionState.Text = socketConnected ? "已连接" : "未连接";
                TxtConnectionState.Foreground = socketConnected ? BrushGreen : BrushRed;
            }

            if (TxtRobotCommState != null)
            {
                TxtRobotCommState.Text = rtdeFresh ? "RTDE正常" : "RTDE未就绪";
                TxtRobotCommState.Foreground = rtdeFresh ? BrushGreen : BrushOrange;
            }

            SetText("TxtDashboardState", global_socket.DashboardClient != null ? "Dashboard正常" : "Dashboard未连接");
            SetText("TxtNetworkState", global_socket.LastNetworkDelayMs >= 0 ? "网络正常" : "网络异常");
        }

        // =========================================================
        // 2. Dashboard 基础状态
        // =========================================================
        private void UpdateDashboardSection(UrDashboardSnapshot s)
        {
            if (s == null)
            {
                if (TxtRobotMode != null)
                    TxtRobotMode.Text = "未知";

                if (TxtEmergencyState != null)
                    TxtEmergencyState.Text = "未知";

                if (TxtProgramState != null)
                    TxtProgramState.Text = "未知";

                SetText("TxtOperationalModeValue", "未知");
                SetText("TxtLoadedProgramValue", "未知");
                SetText("TxtRemoteControlValue", "未知");
                SetText("TxtRobotModelValue", "未知");
                SetText("TxtSerialNumberValue", "未知");
                SetText("TxtPolyScopeVersionValue", "未知");
                return;
            }

            string opMode = TranslateOperationalMode(s.OperationalMode);
            string robotMode = string.IsNullOrWhiteSpace(s.RobotMode) ? "未知" : s.RobotMode;
            string safety = string.IsNullOrWhiteSpace(s.SafetyStatus) ? "未知" : s.SafetyStatus;

            if (TxtRobotMode != null)
                TxtRobotMode.Text = robotMode;

            if (TxtEmergencyState != null)
                TxtEmergencyState.Text = safety;

            if (TxtProgramState != null)
                TxtProgramState.Text = SimplifyProgramState(s.ProgramState);

            SetText("TxtOperationalModeValue", opMode);
            SetText("TxtLoadedProgramValue", string.IsNullOrWhiteSpace(s.LoadedProgram) ? "无" : s.LoadedProgram);
            SetText("TxtRemoteControlValue", s.IsRemoteControl ? "是" : "否");
            SetText("TxtRobotModelValue", string.IsNullOrWhiteSpace(s.RobotModel) ? "未知" : s.RobotModel);
            SetText("TxtSerialNumberValue", string.IsNullOrWhiteSpace(s.SerialNumber) ? "未知" : s.SerialNumber);
            SetText("TxtPolyScopeVersionValue", string.IsNullOrWhiteSpace(s.PolyscopeVersion) ? "未知" : s.PolyscopeVersion);

            SetStatusDot("EllipseRobotModeDot", robotMode.IndexOf("RUNNING", StringComparison.OrdinalIgnoreCase) >= 0);
            SetStatusDot("EllipseSafetyDot", !s.IsEmergencyOrFault);
            SetStatusDot("EllipseProgramDot", s.IsProgramRunning);
            SetStatusDot("EllipseRemoteDot", s.IsRemoteControl);
        }

        // =========================================================
        // 3. RTDE 实时状态
        // =========================================================
        private void UpdateRtdeSection(UrRtdeState s)
        {
            if (s == null)
                return;

            UpdateJointTable(s);
            UpdateTcpPose(s);
            UpdateTcpSpeedAcceleration(s);
            UpdateTcpForce(s);
            UpdateIoSection(s);
            UpdateToolSection(s);
            UpdateRtdeFrequency(s);
        }

        private void UpdateJointTable(UrRtdeState s)
        {
            double[] actualQ = s.ActualQ;
            double[] targetQ = s.TargetQ;
            double[] actualQd = s.ActualQd;
            double[] current = s.ActualCurrent;
            double[] temp = s.JointTemperatures;

            for (int i = 0; i < 6; i++)
            {
                string actualDeg = GetArrayValue(actualQ, i, v => RadToDeg(v).ToString("F2"));
                string speedRad = GetArrayValue(actualQd, i, v => v.ToString("F3"));
                string currentA = GetArrayValue(current, i, v => v.ToString("F3"));
                string tempC = GetArrayValue(temp, i, v => v.ToString("F1"));

                // DataGrid 方式
                if (i < StatusJointRows.Count)
                {
                    StatusJointRows[i].ActualDeg = actualDeg;
                    StatusJointRows[i].SpeedRad = speedRad;
                    StatusJointRows[i].CurrentA = currentA;
                    StatusJointRows[i].TemperatureC = tempC;
                }

                // 静态 Grid + 命名 TextBlock 方式
                int j = i + 1;
                SetText($"TxtJ{j}Actual", actualDeg);
                SetText($"TxtJ{j}Speed", speedRad);
                SetText($"TxtJ{j}Current", currentA);
                SetText($"TxtJ{j}Temp", tempC);
            }

            if (FindName("DgRtdeJointState") is DataGrid dg)
                dg.Items.Refresh();
        }

        private void UpdateTcpPose(UrRtdeState s)
        {
            double[] p = s.ActualTcpPose;
            if (p == null || p.Length < 6)
                return;

            // UR RTDE actual_TCP_pose: x/y/z 单位是 m，这里转为 mm
            if (TxtCurrentX != null) TxtCurrentX.Text = (p[0] * 1000.0).ToString("F2");
            if (TxtCurrentY != null) TxtCurrentY.Text = (p[1] * 1000.0).ToString("F2");
            if (TxtCurrentZ != null) TxtCurrentZ.Text = (p[2] * 1000.0).ToString("F2");

            if (TxtCurrentRx != null) TxtCurrentRx.Text = p[3].ToString("F4");
            if (TxtCurrentRy != null) TxtCurrentRy.Text = p[4].ToString("F4");
            if (TxtCurrentRz != null) TxtCurrentRz.Text = p[5].ToString("F4");
        }

        private void UpdateTcpSpeedAcceleration(UrRtdeState s)
        {
            if (TxtTcpSpeed != null)
                TxtTcpSpeed.Text = $"{s.TcpLinearSpeedMmPerSec:F1}";

            if (TxtTcpAcceleration != null)
                TxtTcpAcceleration.Text = $"{s.TcpLinearAccelerationMmPerSec2:F1}";
        }

        private void UpdateTcpForce(UrRtdeState s)
        {
            double[] f = s.ActualTcpForce;
            if (f == null || f.Length < 6)
                return;

            SetText("TxtTcpFx", f[0].ToString("F2"));
            SetText("TxtTcpFy", f[1].ToString("F2"));
            SetText("TxtTcpFz", f[2].ToString("F2"));
            SetText("TxtTcpMx", f[3].ToString("F3"));
            SetText("TxtTcpMy", f[4].ToString("F3"));
            SetText("TxtTcpMz", f[5].ToString("F3"));
        }

        private void UpdateIoSection(UrRtdeState s)
        {
            for (int i = 0; i < 8; i++)
            {
                SetIoDot($"EllipseDI{i}", s.GetDigitalInputBit(i));
                SetIoDot($"EllipseDO{i}", s.GetDigitalOutputBit(i));
            }

            SetText("TxtAI0", s.StandardAnalogInput0?.ToString("F2") ?? "--");
            SetText("TxtAI1", s.StandardAnalogInput1?.ToString("F2") ?? "--");
            SetText("TxtAO0", s.StandardAnalogOutput0?.ToString("F2") ?? "--");
            SetText("TxtAO1", s.StandardAnalogOutput1?.ToString("F2") ?? "--");
        }

        private void UpdateToolSection(UrRtdeState s)
        {
            SetText("TxtToolVoltageValue", s.ToolOutputVoltage.HasValue ? $"{s.ToolOutputVoltage.Value} V" : "--");
            SetText("TxtToolCurrentValue", s.ToolOutputCurrent.HasValue ? $"{s.ToolOutputCurrent.Value:F2} A" : "--");
            SetText("TxtToolTemperatureValue", s.ToolTemperature.HasValue ? $"{s.ToolTemperature.Value:F1} ℃" : "--");
        }

        private void UpdateRtdeFrequency(UrRtdeState s)
        {
            if (_lastRtdeTimestampInThisWindow > 0.0 && s.Timestamp > _lastRtdeTimestampInThisWindow)
            {
                double dt = s.Timestamp - _lastRtdeTimestampInThisWindow;

                if (dt > 1e-6)
                {
                    double hz = 1.0 / dt;

                    if (TxtControlFrequency != null)
                        TxtControlFrequency.Text = $"{hz:F0} Hz";

                    if (TxtStatusControlPeriod != null)
                        TxtStatusControlPeriod.Text = $"{1000.0 / hz:F1} ms";
                }
            }

            _lastRtdeTimestampInThisWindow = s.Timestamp;

            SetText("TxtLastRtdeReceiveTime", global_socket.LastRtdeReceiveTime == DateTime.MinValue
                ? "--"
                : global_socket.LastRtdeReceiveTime.ToString("HH:mm:ss"));
        }

        // =========================================================
        // 4. 网络状态
        // =========================================================
        private void UpdateNetworkSection()
        {
            UpdateCommunicationUi();
            //if (TxtNetworkDelay != null)
            //{
            //    TxtNetworkDelay.Text = global_socket.LastNetworkDelayMs >= 0.0
            //        ? $"{global_socket.LastNetworkDelayMs:F1} ms"
            //        : "超时";
            //}

            //if (TxtPacketLoss != null)
            //    TxtPacketLoss.Text = $"{global_socket.PacketLossRate:F1} %";

            //SetText("TxtDashboardRefresh", "500 ms");
        }

        private void UpdateSystemTime()
        {
            if (TxtSystemTime != null)
                TxtSystemTime.Text = "本地时间：" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
        }

        // =========================================================
        // 5. 参数读取 / 应用
        // =========================================================
        private void LoadParameterTextBoxesFromRuntime()
        {
            // MoveJ 参数
            SetTextIfExists("TxtMoveJVelocity",
                RobotParameterRuntime.MoveJVelocity.ToString("F3", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtMoveJAcceleration",
                RobotParameterRuntime.MoveJAcceleration.ToString("F3", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtMoveJTime",
                RobotParameterRuntime.MoveJTime.ToString("F3", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtMoveJBlendRadius",
                RobotParameterRuntime.MoveJBlendRadius.ToString("F3", CultureInfo.InvariantCulture));

            // MoveL 参数
            SetTextIfExists("TxtMoveLVelocity",
                RobotParameterRuntime.MoveLVelocity.ToString("F3", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtMoveLAcceleration",
                RobotParameterRuntime.MoveLAcceleration.ToString("F3", CultureInfo.InvariantCulture));

            // SpeedJ / ServoJ 参数
            SetTextIfExists("TxtSpeedJDuration",
                RobotParameterRuntime.SpeedJDuration.ToString("F3", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtServoJLookahead",
                RobotParameterRuntime.ServoJLookaheadTime.ToString("F3", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtServoJGain",
                RobotParameterRuntime.ServoJGain.ToString("F0", CultureInfo.InvariantCulture));

            // 等待到位参数
            SetTextIfExists("TxtToleranceDeg",
                RobotParameterRuntime.ToleranceDeg.ToString("F3", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtTimeoutMsPerPoint",
                RobotParameterRuntime.TimeoutMsPerPoint.ToString(CultureInfo.InvariantCulture));

            SetTextIfExists("TxtPollIntervalMs",
                RobotParameterRuntime.PollIntervalMs.ToString(CultureInfo.InvariantCulture));

            SetTextIfExists("TxtStableCount",
                RobotParameterRuntime.StableCount.ToString(CultureInfo.InvariantCulture));

            // Tool 参数
            if (global_variable.globalToolVector != null &&
                global_variable.globalToolVector.Length >= 3)
            {
                SetTextIfExists("TxtToolX",
                    global_variable.globalToolVector[0].ToString("F2", CultureInfo.InvariantCulture));

                SetTextIfExists("TxtToolY",
                    global_variable.globalToolVector[1].ToString("F2", CultureInfo.InvariantCulture));

                SetTextIfExists("TxtToolZ",
                    global_variable.globalToolVector[2].ToString("F2", CultureInfo.InvariantCulture));
            }

            SetTextIfExists("TxtToolRx",
                global_variable.globalToolRxDeg.ToString("F2", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtToolRy",
                global_variable.globalToolRyDeg.ToString("F2", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtToolRz",
                global_variable.globalToolRzDeg.ToString("F2", CultureInfo.InvariantCulture));

            // RCM 参数
            SetTextIfExists("TxtRcmX",
                RobotParameterRuntime.RcmX.ToString("F2", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtRcmY",
                RobotParameterRuntime.RcmY.ToString("F2", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtRcmZ",
                RobotParameterRuntime.RcmZ.ToString("F2", CultureInfo.InvariantCulture));

            if (ChkRcmMode != null)
                ChkRcmMode.IsChecked = RobotParameterRuntime.RcmEnabled;

            // Payload 参数
            SetTextIfExists("TxtPayloadMass",
                RobotParameterRuntime.PayloadMass.ToString("F3", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtPayloadCogX",
                RobotParameterRuntime.PayloadCogX.ToString("F2", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtPayloadCogY",
                RobotParameterRuntime.PayloadCogY.ToString("F2", CultureInfo.InvariantCulture));

            SetTextIfExists("TxtPayloadCogZ",
                RobotParameterRuntime.PayloadCogZ.ToString("F2", CultureInfo.InvariantCulture));
        }

        private void BtnApplyRobotParams_Click(object sender, RoutedEventArgs e)
        {
            if (!TryApplyParameters(showMessage: true))
                return;

            LoadParameterTextBoxesFromRuntime();

            AddStatusLog("[INFO] 参数已应用到运行时");
            AddStatusLog("[PARAM] 当前 MoveJ: " + RobotParameterRuntime.GetMoveJText());
        }

        private void BtnRestoreRobotParams_Click(object sender, RoutedEventArgs e)
        {
            RobotParameterRuntime.ResetFactoryDefault();
            RobotParameterRuntime.SaveToSettings();

            LoadParameterTextBoxesFromRuntime();

            AddStatusLog("[INFO] 参数已恢复默认，并保存到本地配置");
            AddStatusLog("[PARAM] 默认 MoveJ: " + RobotParameterRuntime.GetMoveJText());
        }

        private void BtnSaveRobotParams_Click(object sender, RoutedEventArgs e)
        {
            if (!TryApplyParameters(showMessage: true))
                return;

            RobotParameterRuntime.SaveToSettings();

            LoadParameterTextBoxesFromRuntime();

            AddStatusLog("[INFO] 参数已保存到 Properties.Settings.Default");
            AddStatusLog("[PARAM] 已保存 MoveJ: " + RobotParameterRuntime.GetMoveJText());
        }


        /// <summary>
        /// clamp需要和实际去匹配一下
        /// </summary>
        /// <param name="showMessage"></param>
        /// <returns></returns>
        private bool TryApplyParameters(bool showMessage)
        {
            try
            {
                // =========================
                // MoveJ 参数
                // =========================
                RobotParameterRuntime.MoveJVelocity =
                    Clamp(ReadDouble("TxtMoveJVelocity", RobotParameterRuntime.MoveJVelocity), 0.01, 1.50);

                RobotParameterRuntime.MoveJAcceleration =
                    Clamp(ReadDouble("TxtMoveJAcceleration", RobotParameterRuntime.MoveJAcceleration), 0.01, 2.00);

                RobotParameterRuntime.MoveJTime =
                    Clamp(ReadDouble("TxtMoveJTime", RobotParameterRuntime.MoveJTime), 0.00, 120.00);

                RobotParameterRuntime.MoveJBlendRadius =
                    Clamp(ReadDouble("TxtMoveJBlendRadius", RobotParameterRuntime.MoveJBlendRadius), 0.00, 0.10);

                // =========================
                // MoveL 参数
                // =========================
                RobotParameterRuntime.MoveLVelocity =
                    Clamp(ReadDouble("TxtMoveLVelocity", RobotParameterRuntime.MoveLVelocity), 0.01, 1.50);

                RobotParameterRuntime.MoveLAcceleration =
                    Clamp(ReadDouble("TxtMoveLAcceleration", RobotParameterRuntime.MoveLAcceleration), 0.01, 2.00);

                // =========================
                // SpeedJ / ServoJ 参数
                // =========================
                RobotParameterRuntime.SpeedJDuration =
                    Clamp(ReadDouble("TxtSpeedJDuration", RobotParameterRuntime.SpeedJDuration), 0.01, 1.50);

                RobotParameterRuntime.ServoJLookaheadTime =
                    Clamp(ReadDouble("TxtServoJLookahead", RobotParameterRuntime.ServoJLookaheadTime), 0.01, 1.00);

                RobotParameterRuntime.ServoJGain =
                    Clamp(ReadDouble("TxtServoJGain", RobotParameterRuntime.ServoJGain), 1.0, 1000.0);

                // =========================
                // 等待到位参数
                // =========================
                RobotParameterRuntime.ToleranceDeg =
                    Clamp(ReadDouble("TxtToleranceDeg", RobotParameterRuntime.ToleranceDeg), 0.001, 0.50);

                RobotParameterRuntime.TimeoutMsPerPoint =
                    ClampInt(ReadInt("TxtTimeoutMsPerPoint", RobotParameterRuntime.TimeoutMsPerPoint), 1000, 60000);

                RobotParameterRuntime.PollIntervalMs =
                    ClampInt(ReadInt("TxtPollIntervalMs", RobotParameterRuntime.PollIntervalMs), 10, 1000);

                RobotParameterRuntime.StableCount =
                    ClampInt(ReadInt("TxtStableCount", RobotParameterRuntime.StableCount), 1, 10);

                // =========================
                // Tool 参数
                // =========================
                if (global_variable.globalToolVector != null &&
                    global_variable.globalToolVector.Length >= 3)
                {
                    double toolX = ReadDouble("TxtToolX", global_variable.globalToolVector[0]);
                    double toolY = ReadDouble("TxtToolY", global_variable.globalToolVector[1]);
                    double toolZ = ReadDouble("TxtToolZ", global_variable.globalToolVector[2]);

                    global_variable.SetGlobalToolVector(toolX, toolY, toolZ);
                }

                global_variable.globalToolRxDeg =
                    ReadDouble("TxtToolRx", global_variable.globalToolRxDeg);

                global_variable.globalToolRyDeg =
                    ReadDouble("TxtToolRy", global_variable.globalToolRyDeg);

                global_variable.globalToolRzDeg =
                    ReadDouble("TxtToolRz", global_variable.globalToolRzDeg);

                // =========================
                // RCM 参数
                // =========================
                RobotParameterRuntime.RcmX =
                    ReadDouble("TxtRcmX", RobotParameterRuntime.RcmX);

                RobotParameterRuntime.RcmY =
                    ReadDouble("TxtRcmY", RobotParameterRuntime.RcmY);

                RobotParameterRuntime.RcmZ =
                    ReadDouble("TxtRcmZ", RobotParameterRuntime.RcmZ);

                RobotParameterRuntime.RcmEnabled =
                    ChkRcmMode?.IsChecked == true;

                // =========================
                // Payload 参数
                // =========================
                RobotParameterRuntime.PayloadMass =
                    Clamp(ReadDouble("TxtPayloadMass", RobotParameterRuntime.PayloadMass), 0.0, 20.0);

                RobotParameterRuntime.PayloadCogX =
                    ReadDouble("TxtPayloadCogX", RobotParameterRuntime.PayloadCogX);

                RobotParameterRuntime.PayloadCogY =
                    ReadDouble("TxtPayloadCogY", RobotParameterRuntime.PayloadCogY);

                RobotParameterRuntime.PayloadCogZ =
                    ReadDouble("TxtPayloadCogZ", RobotParameterRuntime.PayloadCogZ);

                return true;
            }
            catch (Exception ex)
            {
                if (showMessage)
                {
                    System.Windows.MessageBox.Show(
                        "参数应用失败：\n" + ex.Message,
                        "参数错误",
                        MessageBoxButton.OK,
                        MessageBoxImage.Warning
                    );
                }

                AddStatusLog("[ERROR] 参数应用失败：" + ex.Message);
                return false;
            }
        }

        private double Clamp(double value, double min, double max)
        {
            if (value < min)
                return min;

            if (value > max)
                return max;

            return value;
        }

        private int ClampInt(int value, int min, int max)
        {
            if (value < min)
                return min;

            if (value > max)
                return max;

            return value;
        }

        // =========================================================
        // 6. 通用工具函数
        // =========================================================
        private string TranslateOperationalMode(string mode)
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

        private string SimplifyProgramState(string raw)
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

        private static double RadToDeg(double rad)
        {
            return rad * 180.0 / Math.PI;
        }

        private string GetArrayValue(double[] arr, int index, Func<double, string> formatter)
        {
            if (arr == null || arr.Length <= index)
                return "--";

            return formatter(arr[index]);
        }

        private void SetText(string name, string text)
        {
            if (FindName(name) is TextBlock tb)
            {
                tb.Text = text;
            }
            else if (FindName(name) is System.Windows.Controls.TextBox box)
            {
                box.Text = text;
            }
        }

        private void SetTextIfExists(string name, string text)
        {
            SetText(name, text);
        }

        private void SetStatusDot(string name, bool ok)
        {
            if (FindName(name) is Ellipse ellipse)
                ellipse.Fill = ok ? BrushGreen : BrushRed;
        }

        private void SetIoDot(string name, bool on)
        {
            if (FindName(name) is Ellipse ellipse)
                ellipse.Fill = on ? BrushBlue : BrushGray;
        }

        private double ReadDouble(string textBoxName, double defaultValue)
        {
            if (!(FindName(textBoxName) is System.Windows.Controls.TextBox box))
                return defaultValue;

            if (double.TryParse(box.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
                return v;

            if (double.TryParse(box.Text, out v))
                return v;

            throw new FormatException($"{textBoxName} 不是有效数字。");
        }

        private int ReadInt(string textBoxName, int defaultValue)
        {
            if (!(FindName(textBoxName) is System.Windows.Controls.TextBox box))
                return defaultValue;

            if (int.TryParse(box.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int v))
                return v;

            if (int.TryParse(box.Text, out v))
                return v;

            throw new FormatException($"{textBoxName} 不是有效整数。");
        }

        private void AddStatusLog(string message)
        {
            if (TxtStatusLogBox == null)
                return;

            string time = DateTime.Now.ToString("HH:mm:ss");
            TxtStatusLogBox.AppendText(Environment.NewLine + $"[{time}] {message}");
            TxtStatusLogBox.ScrollToEnd();
        }
        private void UpdateNetworkDelayWave(double delayMs)
        {
            if (PolylineNetworkDelay == null || GridNetworkChart == null)
                return;

            // delayMs < 0 表示超时/异常，显示为最高延迟
            double value = delayMs >= 0.0 ? delayMs : NetworkDelayChartMaxMs;

            if (value > NetworkDelayChartMaxMs)
                value = NetworkDelayChartMaxMs;

            _networkDelayHistory.Enqueue(value);

            while (_networkDelayHistory.Count > NetworkDelayHistoryMaxCount)
            {
                _networkDelayHistory.Dequeue();
            }

            double width = GridNetworkChart.ActualWidth - 16.0;
            double height = GridNetworkChart.ActualHeight - 16.0;

            if (width <= 5.0 || height <= 5.0)
                return;

            PointCollection points = new PointCollection();

            double[] values = _networkDelayHistory.ToArray();

            if (values.Length == 1)
            {
                double y0 = 8.0 + height * 0.5;
                points.Add(new System.Windows.Point(8.0, y0));
                points.Add(new System.Windows.Point(8.0 + width, y0));
            }
            else
            {
                for (int i = 0; i < values.Length; i++)
                {
                    double x = 8.0 + i * width / (values.Length - 1);

                    double normalized = values[i] / NetworkDelayChartMaxMs;
                    normalized = Math.Max(0.0, Math.Min(1.0, normalized));

                    // WPF 坐标系 y 越小越靠上。
                    // 延迟越大，曲线越靠上。
                    double y = 8.0 + (1.0 - normalized) * height;

                    points.Add(new System.Windows.Point(x, y));
                }
            }

            PolylineNetworkDelay.Points = points;
        }
        private void UpdateSignalQualityBars(double delayMs, double packetLossRate)
        {
            int level = 5;

            if (delayMs < 0.0)
            {
                level = 1;
            }
            else if (packetLossRate >= 20.0 || delayMs >= 200.0)
            {
                level = 1;
            }
            else if (packetLossRate >= 10.0 || delayMs >= 120.0)
            {
                level = 2;
            }
            else if (packetLossRate >= 5.0 || delayMs >= 80.0)
            {
                level = 3;
            }
            else if (packetLossRate >= 1.0 || delayMs >= 40.0)
            {
                level = 4;
            }
            else
            {
                level = 5;
            }

            System.Windows.Shapes.Rectangle[] bars =
            {
                RectSignal1,
                RectSignal2,
                RectSignal3,
                RectSignal4,
                RectSignal5
            };

            System.Windows.Media.Brush activeBrush;
            if (level >= 4)
                activeBrush = new SolidColorBrush(System.Windows.Media.Color.FromRgb(34, 164, 71));      // 绿色
            else if (level >= 2)
                activeBrush = new SolidColorBrush(System.Windows.Media.Color.FromRgb(245, 158, 11));     // 黄色
            else
                activeBrush = new SolidColorBrush(System.Windows.Media.Color.FromRgb(229, 57, 53));      // 红色

            System.Windows.Media.Brush inactiveBrush = new SolidColorBrush(System.Windows.Media.Color.FromRgb(184, 192, 204));

            for (int i = 0; i < bars.Length; i++)
            {
                if (bars[i] == null)
                    continue;

                bars[i].Fill = i < level ? activeBrush : inactiveBrush;
            }
        }
        private void UpdateCommunicationUi()
        {
            double delayMs = global_socket.LastNetworkDelayMs;
            double lossRate = global_socket.PacketLossRate;

            if (delayMs >= 0.0)
            {
                TxtNetworkDelay.Text = $"{delayMs:F1} ms";
            }
            else
            {
                TxtNetworkDelay.Text = "超时";
            }

            TxtPacketLoss.Text = $"{lossRate:F1} %";

            UpdateNetworkDelayWave(delayMs);
            UpdateSignalQualityBars(delayMs, lossRate);

            if (global_socket.LastRtdeReceiveTime != DateTime.MinValue)
            {
                TxtLastRtdeReceiveTime.Text =
                    global_socket.LastRtdeReceiveTime.ToString("HH:mm:ss.fff");
            }
            else
            {
                TxtLastRtdeReceiveTime.Text = "--";
            }

            TxtDashboardRefresh.Text = "500 ms";
        }
    }

    public class StatusJointRow
    {
        public string Joint { get; set; }
        public string ActualDeg { get; set; }
        public string TargetDeg { get; set; }
        public string SpeedRad { get; set; }
        public string CurrentA { get; set; }
        public string TemperatureC { get; set; }
    }

}
