using System;
using System.Collections.ObjectModel;
using System.Globalization;
using System.IO;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using WpfRobot.joint7;

namespace WpfRobot.calibration
{
    /// <summary>
    /// motor7Cali.xaml 的交互逻辑
    /// </summary>
    public partial class motor7Cali : System.Windows.Controls.UserControl
    {
        private Action<string> _externalLog;

        private readonly ObservableCollection<Motor7ZeroRecord> _records =
            new ObservableCollection<Motor7ZeroRecord>();

        private readonly DispatcherTimer _monitorTimer;

        private bool _isMotorConnected = false;
        private bool _isMotorBusy = false;
        private bool _isReadingState = false;

        private readonly IMotor7Driver _motor7Driver =
            new NimMotor7Driver(
                commType: 0,
                commParam: "1001",
                unitFactor: 10000.0, // 10000单位对应360度，1 用户单位 = 27.777777 编码器脉冲 = 1°
                profileVelocity: 3.0,
                profileAccel: 5.0,
                profileDecel: 5.0);

        /// <summary>
        /// 第七电机原始编码器角度，单位 deg
        /// </summary>
        private double _encoderAngleDeg = 0.0;

        /// <summary>
        /// 第七电机零点偏移，单位 deg
        /// q7 = encoder - zeroOffset
        /// </summary>
        private double _zeroOffsetDeg = 0.0;

        private string CalibrationParamPath =>
            Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "calibrationJson", "motor7_calibration_params.json");

        public motor7Cali() : this(null)
        {
        }

        public motor7Cali(Action<string> externalLog)
        {
            _externalLog = externalLog;

            InitializeComponent();

            ZeroRecordDataGrid.ItemsSource = _records;

            _monitorTimer = new DispatcherTimer();
            _monitorTimer.Interval = TimeSpan.FromMilliseconds(500);
            _monitorTimer.Tick += MonitorTimer_Tick;

            UpdateUiValues();
            SetConnectedUi(false);

            LoadZeroSilently();

            AddLog("[Motor7] 第七电机零位标定页面已打开。");
        }

        private async void BtnConnectMotor7_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                BtnConnectMotor7.IsEnabled = false;
                BtnDisconnectMotor7.IsEnabled = false;

                AddLog("[Motor7] 正在连接第七电机...");

                await Task.Run(() =>
                {
                    _motor7Driver.Connect();
                });

                _isMotorConnected = _motor7Driver.IsConnected;

                TxtConnectStatus.Text = "已连接";
                TxtConnectStatus.Foreground =
                    new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));

                SetConnectedUi(true);

                await ReadMotor7StateOnceAsync(true);

                AddLog($"[Motor7] 第七电机连接成功，NodeId={_motor7Driver.NodeId}");
            }
            catch (Exception ex)
            {
                _isMotorConnected = false;

                TxtConnectStatus.Text = "连接失败";
                TxtConnectStatus.Foreground =
                    new SolidColorBrush(System.Windows.Media.Color.FromRgb(220, 38, 38));

                TxtMotorState.Text =
                    "Motor State: Connect Failed\n" +
                    "Servo: --\n" +
                    "Alarm: --\n" +
                    "Limit: --";

                SetConnectedUi(false);

                AddLog("[Motor7] 连接失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "第七电机连接失败：\n" + ex.Message,
                    "连接失败",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Error);
            }
        }

        private async void BtnDisconnectMotor7_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                BtnConnectMotor7.IsEnabled = false;
                BtnDisconnectMotor7.IsEnabled = false;

                StopMonitor(false);

                AddLog("[Motor7] 正在断开第七电机...");

                await Task.Run(() =>
                {
                    _motor7Driver.Disconnect();
                });

                _isMotorConnected = false;

                TxtConnectStatus.Text = "未连接";
                TxtConnectStatus.Foreground =
                    new SolidColorBrush(System.Windows.Media.Color.FromRgb(100, 116, 139));

                TxtMotorState.Text =
                    "Motor State: Disconnected\n" +
                    "Servo: --\n" +
                    "Alarm: --\n" +
                    "Limit: --";

                SetConnectedUi(false);

                AddLog("[Motor7] 第七电机已断开连接。");
            }
            catch (Exception ex)
            {
                AddLog("[Motor7] 断开连接异常：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "断开第七电机失败：\n" + ex.Message,
                    "断开失败",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Warning);

                SetConnectedUi(_isMotorConnected);
            }
        }

        private async void BtnReadMotor7_Click(object sender, RoutedEventArgs e)
        {
            if (!CheckMotorConnected())
                return;

            await ReadMotor7StateOnceAsync(true);
        }

        private void BtnStartMonitor_Click(object sender, RoutedEventArgs e)
        {
            if (!CheckMotorConnected())
                return;

            if (_isMotorBusy)
            {
                AddLog("[Monitor] 第七电机正在运动，运动完成后再开启监测。");
                return;
            }

            _monitorTimer.Start();

            TxtMonitorStatus.Text = "实时监测中";
            TxtMonitorStatus.Foreground =
                new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));

            AddLog("[Monitor] 开始实时监测第七电机状态。");
        }

        private void BtnStopMonitor_Click(object sender, RoutedEventArgs e)
        {
            StopMonitor(true);
        }

        private void StopMonitor(bool writeLog)
        {
            if (_monitorTimer != null && _monitorTimer.IsEnabled)
                _monitorTimer.Stop();

            if (TxtMonitorStatus != null)
            {
                TxtMonitorStatus.Text = "未监测";
                TxtMonitorStatus.Foreground =
                    new SolidColorBrush(System.Windows.Media.Color.FromRgb(100, 116, 139));
            }

            if (writeLog)
                AddLog("[Monitor] 停止实时监测。");
        }

        private async void MonitorTimer_Tick(object sender, EventArgs e)
        {
            if (!_isMotorConnected)
                return;

            if (_isMotorBusy || _isReadingState)
                return;

            await ReadMotor7StateOnceAsync(false);
        }

        private async Task ReadMotor7StateOnceAsync(bool writeLog)
        {
            if (!_motor7Driver.IsConnected)
                return;

            if (_isReadingState || _isMotorBusy)
                return;

            try
            {
                _isReadingState = true;

                var result = await Task.Run(() =>
                {
                    double encoder = _motor7Driver.ReadEncoderAngleDeg();
                    Motor7State state = _motor7Driver.ReadState();
                    return new
                    {
                        Encoder = encoder,
                        State = state
                    };
                });

                _encoderAngleDeg = result.Encoder;
                TxtMotorState.Text = result.State.ToString();

                UpdateUiValues();

                if (writeLog)
                {
                    AddLog(
                        $"[Read] Encoder={_encoderAngleDeg:F3} deg, " +
                        $"ZeroOffset={_zeroOffsetDeg:F3} deg, " +
                        $"q7={GetSoftwareAngleDeg():F3} deg");
                }
            }
            catch (Exception ex)
            {
                AddLog("[Read] 读取第七电机失败：" + ex.Message);
            }
            finally
            {
                _isReadingState = false;
            }
        }

        private async void BtnSetCurrentZero_Click(object sender, RoutedEventArgs e)
        {
            if (!CheckMotorConnected())
                return;

            if (_isMotorBusy)
            {
                AddLog("[Zero] 第七电机正在运动，不能设置零点。");
                return;
            }

            await ReadMotor7StateOnceAsync(false);

            System.Windows.MessageBoxResult result = System.Windows.MessageBox.Show(
                "确定将当前第七电机位置设置为机械零点吗？\n\n设置后：zero_offset = 当前编码器角度，软件角度 q7 将变为 0。",
                "确认设置零点",
                System.Windows.MessageBoxButton.YesNo,
                System.Windows.MessageBoxImage.Question);

            if (result != System.Windows.MessageBoxResult.Yes)
                return;

            _zeroOffsetDeg = _encoderAngleDeg;

            UpdateUiValues();

            AddZeroRecord("设置当前位置为零点");

            AddLog($"[Zero] 已设置当前位置为零点：zero_offset = {_zeroOffsetDeg:F3} deg");
        }

        private async void BtnClearEncoderHardwareZero_Click(object sender, RoutedEventArgs e)
        {
            if (!CheckMotorConnected())
                return;

            if (_isMotorBusy)
            {
                AddLog("[Zero] 第七电机正在运动，不能执行编码器硬件清零。");
                return;
            }

            System.Windows.MessageBoxResult result = System.Windows.MessageBox.Show(
                "确定要将驱动器内部当前位置清零吗？\n\n" +
                "这会改变电机驱动器内部的当前位置 H6063/H6064，" +
                "不是普通的软件 zero_offset。\n\n" +
                "建议只在第七轴已经处于机械零位时执行。",
                "确认编码器清零",
                System.Windows.MessageBoxButton.YesNo,
                System.Windows.MessageBoxImage.Warning);

            if (result != System.Windows.MessageBoxResult.Yes)
                return;

            bool resumeMonitor = _monitorTimer.IsEnabled;

            try
            {
                StopMonitor(false);
                SetBusyUi(true);

                AddLog("[Zero] 开始执行第七电机编码器硬件清零...");

                await Task.Run(() =>
                {
                    _motor7Driver.ClearEncoderHardwareZero();
                });

                // 硬件编码器已经清零，软件零偏也应该同步清零
                _zeroOffsetDeg = 0.0;

                SetBusyUi(false);

                await ReadMotor7StateOnceAsync(false);

                UpdateUiValues();

                AddZeroRecord("编码器硬件清零");

                AddLog(
                    $"[Zero] 编码器硬件清零完成：Encoder={_encoderAngleDeg:F3} deg，" +
                    $"ZeroOffset={_zeroOffsetDeg:F3} deg，q7={GetSoftwareAngleDeg():F3} deg");
            }
            catch (Exception ex)
            {
                AddLog("[Zero] 编码器硬件清零失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "编码器硬件清零失败：\n" + ex.Message,
                    "清零失败",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Error);
            }
            finally
            {
                SetBusyUi(false);

                if (resumeMonitor && _isMotorConnected)
                {
                    _monitorTimer.Start();
                    TxtMonitorStatus.Text = "实时监测中";
                    TxtMonitorStatus.Foreground =
                        new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));
                }
            }
        }

        private void BtnApplyManualOffset_Click(object sender, RoutedEventArgs e)
        {
            if (!TryReadDouble(TxtManualOffset.Text, out double value))
            {
                System.Windows.MessageBox.Show(
                    "请输入有效的 zero_offset 数值。",
                    "输入错误",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Warning);
                return;
            }

            _zeroOffsetDeg = value;

            UpdateUiValues();

            AddZeroRecord("手动应用 zero_offset");

            AddLog($"[Zero] 已手动应用 zero_offset = {_zeroOffsetDeg:F3} deg");
        }

        private async void BtnJogMinus_Click(object sender, RoutedEventArgs e)
        {
            if (!CheckMotorConnected())
                return;

            if (!TryReadJogStep(out double stepDeg))
                return;

            await JogMotor7Async(-Math.Abs(stepDeg));
        }

        private async void BtnJogPlus_Click(object sender, RoutedEventArgs e)
        {
            if (!CheckMotorConnected())
                return;

            if (!TryReadJogStep(out double stepDeg))
                return;

            await JogMotor7Async(Math.Abs(stepDeg));
        }

        private async Task JogMotor7Async(double deltaDeg)
        {
            if (_isMotorBusy)
            {
                AddLog("[Jog] 第七电机正在运动，请等待当前动作完成。");
                return;
            }

            bool resumeMonitor = _monitorTimer.IsEnabled;

            try
            {
                StopMonitor(false);
                SetBusyUi(true);

                AddLog($"[Jog] 第七电机准备点动 {deltaDeg:F3} deg");

                await Task.Run(() =>
                {
                    _motor7Driver.MoveRelativeEncoderDeg(deltaDeg);
                });

                await ReadMotor7StateOnceAsync(false);

                AddLog($"[Jog] 第七电机点动完成，当前 Encoder={_encoderAngleDeg:F3} deg，q7={GetSoftwareAngleDeg():F3} deg");
            }
            catch (Exception ex)
            {
                AddLog("[Jog] 第七电机点动失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "第七电机点动失败：\n" + ex.Message,
                    "点动失败",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Error);
            }
            finally
            {
                SetBusyUi(false);

                if (resumeMonitor && _isMotorConnected)
                {
                    _monitorTimer.Start();
                    TxtMonitorStatus.Text = "实时监测中";
                    TxtMonitorStatus.Foreground =
                        new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));
                }
            }
        }

        private async void BtnMoveToZero_Click(object sender, RoutedEventArgs e)
        {
            if (!CheckMotorConnected())
                return;

            double targetEncoderDeg = _zeroOffsetDeg;

            await MoveMotor7ToEncoderDegAsync(
                targetEncoderDeg,
                $"回到软件零位 q7=0，目标 encoder={targetEncoderDeg:F3} deg");
        }

        private async void BtnMoveToTarget_Click(object sender, RoutedEventArgs e)
        {
            if (!CheckMotorConnected())
                return;

            if (!TryReadDouble(TxtTargetAngle.Text, out double targetQ7Deg))
            {
                System.Windows.MessageBox.Show(
                    "请输入有效的目标 q7 角度。",
                    "输入错误",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Warning);
                return;
            }

            double targetEncoderDeg = targetQ7Deg + _zeroOffsetDeg;

            await MoveMotor7ToEncoderDegAsync(
                targetEncoderDeg,
                $"运动到目标软件角 q7={targetQ7Deg:F3} deg，目标 encoder={targetEncoderDeg:F3} deg");
        }

        private async Task MoveMotor7ToEncoderDegAsync(double targetEncoderDeg, string actionName)
        {
            if (_isMotorBusy)
            {
                AddLog("[Move] 第七电机正在运动，请等待当前动作完成。");
                return;
            }

            bool resumeMonitor = _monitorTimer.IsEnabled;

            try
            {
                StopMonitor(false);
                SetBusyUi(true);

                AddLog($"[Move] 第七电机准备{actionName}");

                await Task.Run(() =>
                {
                    _motor7Driver.MoveAbsoluteEncoderDeg(targetEncoderDeg);
                });

                await ReadMotor7StateOnceAsync(false);

                AddLog($"[Move] 第七电机运动完成，当前 Encoder={_encoderAngleDeg:F3} deg，q7={GetSoftwareAngleDeg():F3} deg");
            }
            catch (Exception ex)
            {
                AddLog("[Move] 第七电机运动失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "第七电机运动失败：\n" + ex.Message,
                    "运动失败",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Error);
            }
            finally
            {
                SetBusyUi(false);

                if (resumeMonitor && _isMotorConnected)
                {
                    _monitorTimer.Start();
                    TxtMonitorStatus.Text = "实时监测中";
                    TxtMonitorStatus.Foreground =
                        new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));
                }
            }
        }

        private void BtnLoadZero_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (!File.Exists(CalibrationParamPath))
                {
                    System.Windows.MessageBox.Show(
                        "没有找到 calibration_params.json。",
                        "加载零点",
                        System.Windows.MessageBoxButton.OK,
                        System.Windows.MessageBoxImage.Information);
                    return;
                }

                string json = File.ReadAllText(CalibrationParamPath);
                JsonNode node = JsonNode.Parse(json);

                JsonNode zeroNode = node?["motor7"]?["zero_offset_deg"];

                if (zeroNode == null)
                {
                    System.Windows.MessageBox.Show(
                        "calibration_params.json 中没有 motor7.zero_offset_deg。",
                        "加载零点",
                        System.Windows.MessageBoxButton.OK,
                        System.Windows.MessageBoxImage.Warning);
                    return;
                }

                _zeroOffsetDeg = zeroNode.GetValue<double>();

                UpdateUiValues();

                AddZeroRecord("加载 zero_offset");

                AddLog($"[Load] 已加载第七电机 zero_offset = {_zeroOffsetDeg:F3} deg");
            }
            catch (Exception ex)
            {
                AddLog("[Load] 加载第七电机零点失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "加载第七电机零点失败：\n" + ex.Message,
                    "加载失败",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Error);
            }
        }

        private async void BtnSaveZero_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_isMotorConnected && _motor7Driver.IsConnected && !_isMotorBusy)
                {
                    await ReadMotor7StateOnceAsync(false);
                }

                // 写入 Settings.settings
                Properties.Settings.Default.motor7JointFromEncoder = _encoderAngleDeg;
                Properties.Settings.Default.Save();

                SaveZeroToFile();

                AddZeroRecord("保存 zero_offset 和 motor7JointFromEncoder");

                AddLog(
                    $"[Save] 已保存第七电机参数：" +
                    $"zero_offset={_zeroOffsetDeg:F3} deg，" +
                    $"motor7JointFromEncoder={_encoderAngleDeg:F3} deg");
            }
            catch (Exception ex)
            {
                AddLog("[Save] 保存第七电机零点失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "保存第七电机零点失败：\n" + ex.Message,
                    "保存失败",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Error);
            }
        }

        private void BtnAddRecord_Click(object sender, RoutedEventArgs e)
        {
            AddZeroRecord("手动记录当前状态");
            AddLog("[Record] 已记录当前第七电机零点状态。");
        }

        private void BtnClearRecords_Click(object sender, RoutedEventArgs e)
        {
            if (_records.Count == 0)
                return;

            System.Windows.MessageBoxResult result = System.Windows.MessageBox.Show(
                "确定要清空所有零点标定记录吗？",
                "确认清空",
                System.Windows.MessageBoxButton.YesNo,
                System.Windows.MessageBoxImage.Question);

            if (result != System.Windows.MessageBoxResult.Yes)
                return;

            _records.Clear();

            AddLog("[Record] 已清空零点标定记录。");
        }

        private void BtnClearLog_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[Log] 当前日志显示在标定主窗口中。");
        }

        private void SaveZeroToFile()
        {
            JsonObject root;

            if (File.Exists(CalibrationParamPath))
            {
                try
                {
                    root = JsonNode.Parse(File.ReadAllText(CalibrationParamPath)) as JsonObject
                           ?? new JsonObject();
                }
                catch
                {
                    root = new JsonObject();
                }
            }
            else
            {
                root = new JsonObject();
            }

            JsonObject motor7Node;

            if (root["motor7"] is JsonObject existingMotor7)
            {
                motor7Node = existingMotor7;
            }
            else
            {
                motor7Node = new JsonObject();
                root["motor7"] = motor7Node;
            }

            motor7Node["zero_offset_deg"] = _zeroOffsetDeg;
            motor7Node["last_encoder_deg"] = _encoderAngleDeg;
            motor7Node["last_q7_deg"] = GetSoftwareAngleDeg();
            motor7Node["saved_time"] = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

            JsonSerializerOptions options = new JsonSerializerOptions
            {
                WriteIndented = true
            };

            File.WriteAllText(CalibrationParamPath, root.ToJsonString(options));
        }

        private void LoadZeroSilently()
        {
            try
            {
                if (!File.Exists(CalibrationParamPath))
                    return;

                string json = File.ReadAllText(CalibrationParamPath);
                JsonNode node = JsonNode.Parse(json);

                JsonNode zeroNode = node?["motor7"]?["zero_offset_deg"];

                if (zeroNode == null)
                    return;

                _zeroOffsetDeg = zeroNode.GetValue<double>();

                UpdateUiValues();

                AddLog($"[Load] 已自动加载第七电机 zero_offset = {_zeroOffsetDeg:F3} deg");
            }
            catch
            {
                // 自动加载失败不弹窗，避免打开页面时打断用户
            }
        }

        private void AddZeroRecord(string note)
        {
            Motor7ZeroRecord record = new Motor7ZeroRecord
            {
                Index = _records.Count + 1,
                Time = DateTime.Now.ToString("HH:mm:ss.fff"),
                EncoderDeg = _encoderAngleDeg.ToString("F3", CultureInfo.InvariantCulture),
                ZeroOffsetDeg = _zeroOffsetDeg.ToString("F3", CultureInfo.InvariantCulture),
                Note = note
            };

            _records.Add(record);
            ZeroRecordDataGrid.SelectedItem = record;
            ZeroRecordDataGrid.ScrollIntoView(record);
        }

        private void UpdateUiValues()
        {
            double q7Deg = GetSoftwareAngleDeg();

            TxtEncoderAngle.Text = _encoderAngleDeg.ToString("F3", CultureInfo.InvariantCulture);
            TxtZeroOffset.Text = _zeroOffsetDeg.ToString("F3", CultureInfo.InvariantCulture);
            TxtSoftwareAngle.Text = q7Deg.ToString("F3", CultureInfo.InvariantCulture);
            TxtManualOffset.Text = _zeroOffsetDeg.ToString("F3", CultureInfo.InvariantCulture);
        }

        private double GetSoftwareAngleDeg()
        {
            return _encoderAngleDeg - _zeroOffsetDeg;
        }

        private bool CheckMotorConnected()
        {
            if (_isMotorConnected && _motor7Driver.IsConnected)
                return true;

            AddLog("[Motor7] 第七电机未连接。");

            System.Windows.MessageBox.Show(
                "请先连接第七电机。",
                "第七电机未连接",
                System.Windows.MessageBoxButton.OK,
                System.Windows.MessageBoxImage.Warning);

            return false;
        }

        private bool TryReadJogStep(out double stepDeg)
        {
            stepDeg = 0.0;

            if (!TryReadDouble(TxtJogStep.Text, out stepDeg))
            {
                System.Windows.MessageBox.Show(
                    "请输入有效的点动步长。",
                    "输入错误",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Warning);
                return false;
            }

            stepDeg = Math.Abs(stepDeg);

            if (stepDeg <= 0)
            {
                System.Windows.MessageBox.Show(
                    "点动步长必须大于 0。",
                    "输入错误",
                    System.Windows.MessageBoxButton.OK,
                    System.Windows.MessageBoxImage.Warning);
                return false;
            }

            if (stepDeg > 30.0)
            {
                System.Windows.MessageBoxResult result = System.Windows.MessageBox.Show(
                    $"当前点动步长为 {stepDeg:F3} deg，角度较大。\n\n确定继续吗？",
                    "点动步长确认",
                    System.Windows.MessageBoxButton.YesNo,
                    System.Windows.MessageBoxImage.Question);

                if (result != System.Windows.MessageBoxResult.Yes)
                    return false;
            }

            return true;
        }

        private bool TryReadDouble(string text, out double value)
        {
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
                return true;

            return double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value);
        }

        private void SetConnectedUi(bool connected)
        {
            BtnConnectMotor7.IsEnabled = !connected;
            BtnDisconnectMotor7.IsEnabled = connected;

            BtnReadMotor7.IsEnabled = connected;
            BtnStartMonitor.IsEnabled = connected;
            BtnStopMonitor.IsEnabled = connected;

            BtnSetCurrentZero.IsEnabled = connected;

            BtnJogMinus.IsEnabled = connected;
            BtnJogPlus.IsEnabled = connected;
            BtnMoveToZero.IsEnabled = connected;
            BtnMoveToTarget.IsEnabled = connected;

            BtnApplyManualOffset.IsEnabled = true;
            BtnLoadZero.IsEnabled = true;
            BtnSaveZero.IsEnabled = true;
            BtnAddRecord.IsEnabled = true;
            BtnClearRecords.IsEnabled = true;
        }

        private void SetBusyUi(bool busy)
        {
            _isMotorBusy = busy;

            if (!_isMotorConnected)
            {
                SetConnectedUi(false);
                return;
            }

            BtnConnectMotor7.IsEnabled = false;
            BtnDisconnectMotor7.IsEnabled = !busy;

            BtnReadMotor7.IsEnabled = !busy;
            BtnStartMonitor.IsEnabled = !busy;
            BtnStopMonitor.IsEnabled = !busy;

            BtnSetCurrentZero.IsEnabled = !busy;

            BtnJogMinus.IsEnabled = !busy;
            BtnJogPlus.IsEnabled = !busy;
            BtnMoveToZero.IsEnabled = !busy;
            BtnMoveToTarget.IsEnabled = !busy;

            BtnApplyManualOffset.IsEnabled = !busy;
            BtnLoadZero.IsEnabled = !busy;
            BtnSaveZero.IsEnabled = !busy;
            BtnAddRecord.IsEnabled = !busy;
            BtnClearRecords.IsEnabled = !busy;
        }

        private void AddLog(string message)
        {
            if (_externalLog != null)
            {
                _externalLog(message);
            }
        }
    }

    public class Motor7ZeroRecord
    {
        public int Index { get; set; }

        public string Time { get; set; }

        public string EncoderDeg { get; set; }

        public string ZeroOffsetDeg { get; set; }

        public string Note { get; set; }
    }
}