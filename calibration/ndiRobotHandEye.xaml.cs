using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using WpfRobot.kinematics;

namespace WpfRobot.calibration
{
    /// <summary>
    /// ndiRobotHandEye.xaml 的交互逻辑
    /// </summary>
    public partial class ndiRobotHandEye : System.Windows.Controls.UserControl
    {
        private readonly ObservableCollection<NdiRobotSampleItem> _samples =
            new ObservableCollection<NdiRobotSampleItem>();

        private bool _isRobotConnected = false;
        private bool _isNdiConnected = false;

        private bool _hasRobotPose = false;
        private bool _hasNdiPose = false;

        private Matrix<double> _currentRobotMatrix = null;
        private Matrix<double> _currentNdiMatrix = null;
        private Matrix<double> _lastCalibrationResult = null;

        private string _currentRobotPoseFull = "";
        private string _currentNdiPoseFull = "";

        private string _currentRobotPoseShort = "";
        private string _currentNdiPoseShort = "";

        /// <summary>
        /// true：机器人位姿使用真实关节角计算标准 T06。
        /// false：机器人位姿使用真实关节角计算 T0Tcp。
        /// 
        /// 如果 NDI 39 刚体安装在第六轴法兰附近，建议 true。
        /// 如果 NDI 39 刚体安装在第七轴工具 / 器械上，建议 false。
        /// </summary>
        private bool _useActualRobotT06 = true;

        /// <summary>
        /// 外部日志回调，由 calibrationWindow 传入。
        /// 这里不再使用页面内部 TxtLog。
        /// </summary>
        private readonly Action<string> _externalLog;

        public ndiRobotHandEye() : this(null)
        {
        }

        public ndiRobotHandEye(Action<string> externalLog)
        {
            InitializeComponent();

            _externalLog = externalLog;

            SampleDataGrid.ItemsSource = _samples;

            UpdateSampleCount();

            AddLog("[NDI-Robot] NDI-机械臂手眼标定页面已打开。");
            AddLog("[NDI-Robot] 机器人位姿来源：global_variable._actualJointDegForTable -> FK。");
            AddLog("[NDI-Robot] NDI 位姿来源：global_posture.TryGet8700339。");
        }

        private void BtnConnectRobot_Click(object sender, RoutedEventArgs e)
        {
            _isRobotConnected = true;

            TxtRobotStatus.Text = "可读取";
            TxtRobotStatus.Foreground =
                new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));

            AddLog("[Robot] 机器人状态已设置为可读取。实际数据来自 global_variable._actualJointDegForTable。");
        }

        private void BtnConnectNdi_Click(object sender, RoutedEventArgs e)
        {

        }

        private void BtnReadRobotPose_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                AddLog("[Robot] 读取机器人当前实际关节角，并计算正运动学矩阵。");

                Matrix<double> robotPose;

                if (_useActualRobotT06)
                {
                    if (!TryGetActualRobotT06(out robotPose))
                    {
                        AddLog("[Robot] 机器人 T06 计算失败。");
                        return;
                    }
                }
                else
                {
                    if (!TryGetActualRobotT0Tcp(out robotPose))
                    {
                        AddLog("[Robot] 机器人 T0Tcp 计算失败。");
                        return;
                    }
                }

                _currentRobotMatrix = robotPose.Clone();
                _currentRobotPoseFull = FormatMatrix(
                    _useActualRobotT06 ? "T_base_T06" : "T_base_T0Tcp",
                    _currentRobotMatrix);

                _currentRobotPoseShort = FormatPoseShort(_currentRobotMatrix);

                TxtRobotJoint.Text = FormatJointText(global_variable._actualJointDegForTable);
                TxtRobotPose.Text = _currentRobotPoseFull;

                _hasRobotPose = true;
                _isRobotConnected = true;

                TxtRobotStatus.Text = _useActualRobotT06 ? "已读取 T06" : "已读取 T0Tcp";
                TxtRobotStatus.Foreground =
                    new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));

                AddLog("[Robot] 机器人位姿读取完成：" + _currentRobotPoseShort);
            }
            catch (Exception ex)
            {
                AddLog("[Robot ERROR] 读取机器人位姿失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "读取机器人位姿失败：\n" + ex.Message,
                    "机器人位姿错误",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }
        }

        private bool TryGetActualRobotT06(out Matrix<double> robotT06)
        {
            robotT06 = null;

            if (global_variable._actualJointDegForTable == null ||
                global_variable._actualJointDegForTable.Length < 6)
            {
                AddLog("[Robot] _actualJointDegForTable 为空或长度不足 6。");
                return false;
            }

            double[] q6Rad = global_variable._actualJointDegForTable
                .Take(6)
                .Select(deg => deg * Math.PI / 180.0)
                .ToArray();

            robotT06 = Forward.ForwardKinematicsMatrix(q6Rad);

            return robotT06 != null;
        }

        private bool TryGetActualRobotT0Tcp(out Matrix<double> robotT0Tcp)
        {
            robotT0Tcp = null;

            if (global_variable._actualJointDegForTable == null ||
                global_variable._actualJointDegForTable.Length < 7)
            {
                AddLog("[Robot] _actualJointDegForTable 为空或长度不足 7。");
                return false;
            }

            double[] q7Rad = global_variable._actualJointDegForTable
                .Take(7)
                .Select(deg => deg * Math.PI / 180.0)
                .ToArray();

            robotT0Tcp = Forward.ForwardKinematicsMatrix7_TCP(q7Rad);

            return robotT0Tcp != null;
        }

        private void BtnReadNdiPose_Click(object sender, RoutedEventArgs e)
        {
           
        }

        private void BtnClearRobotPose_Click(object sender, RoutedEventArgs e)
        {
            _hasRobotPose = false;
            _currentRobotMatrix = null;
            _currentRobotPoseFull = "";
            _currentRobotPoseShort = "";

            TxtRobotJoint.Text = "q1=--, q2=--, q3=--, q4=--, q5=--, q6=--, q7=--";
            TxtRobotPose.Text = "尚未读取机器人位姿";

            AddLog("[Robot] 已清空当前机器人位姿。");
        }

        private void BtnClearNdiPose_Click(object sender, RoutedEventArgs e)
        {
            _hasNdiPose = false;
            _currentNdiMatrix = null;
            _currentNdiPoseFull = "";
            _currentNdiPoseShort = "";

            TxtNdiMarkerInfo.Text =
                "Marker ID=--\n" +
                "Tracking State=--\n" +
                "Error=--";

            TxtNdiPose.Text = "尚未读取 NDI marker 位姿";

            AddLog("[NDI] 已清空当前 NDI 位姿。");
        }

        private void BtnCapturePair_Click(object sender, RoutedEventArgs e)
        {
            if (!_hasRobotPose || _currentRobotMatrix == null)
            {
                AddLog("[Sample] 尚未读取机器人位姿，无法采集样本。");

                System.Windows.MessageBox.Show(
                    "请先读取机器人当前位姿。",
                    "无法采集",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);

                return;
            }

            if (!_hasNdiPose || _currentNdiMatrix == null)
            {
                AddLog("[Sample] 尚未读取 NDI 39 位姿，无法采集样本。");

                System.Windows.MessageBox.Show(
                    "请先读取 NDI 39 当前位姿。",
                    "无法采集",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);

                return;
            }

            int index = _samples.Count + 1;

            NdiRobotSampleItem item = new NdiRobotSampleItem
            {
                Index = index,
                CaptureTime = DateTime.Now.ToString("HH:mm:ss.fff"),

                RobotMatrix = _currentRobotMatrix.Clone(),
                NdiMatrix = _currentNdiMatrix.Clone(),

                RobotPoseFull = _currentRobotPoseFull,
                NdiPoseFull = _currentNdiPoseFull,
                RobotPoseShort = _currentRobotPoseShort,
                NdiPoseShort = _currentNdiPoseShort,

                StateText = "有效"
            };

            _samples.Add(item);

            SampleDataGrid.SelectedItem = item;
            SampleDataGrid.ScrollIntoView(item);

            UpdateSampleCount();

            AddLog($"[Sample] 已采集第 {index} 组 NDI-机器人标定样本。");

            if (_samples.Count < 4)
            {
                AddLog($"[Sample] 当前样本数量 {_samples.Count}，至少建议 4 组，最好 10～20 组。");
            }
        }

        private void SampleDataGrid_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            NdiRobotSampleItem item = SampleDataGrid.SelectedItem as NdiRobotSampleItem;

            if (item == null)
            {
                TxtSelectedSample.Text = "未选择";
                return;
            }

            TxtSelectedSample.Text = $"当前选择：#{item.Index}";

            TxtRobotPose.Text = item.RobotPoseFull;
            TxtNdiPose.Text = item.NdiPoseFull;
        }

        private void BtnDeleteSample_Click(object sender, RoutedEventArgs e)
        {
            NdiRobotSampleItem item = SampleDataGrid.SelectedItem as NdiRobotSampleItem;

            if (item == null)
            {
                AddLog("[Sample] 未选择需要删除的样本。");
                return;
            }

            int deletedIndex = item.Index;

            _samples.Remove(item);

            RenumberSamples();
            UpdateSampleCount();

            TxtSelectedSample.Text = "未选择";

            AddLog($"[Sample] 已删除样本 #{deletedIndex}。");
        }

        private void BtnClearSamples_Click(object sender, RoutedEventArgs e)
        {
            if (_samples.Count == 0)
                return;

            MessageBoxResult result = System.Windows.MessageBox.Show(
                "确定要清空所有 NDI-机器人标定样本吗？",
                "确认清空",
                MessageBoxButton.YesNo,
                MessageBoxImage.Question);

            if (result != MessageBoxResult.Yes)
                return;

            _samples.Clear();

            _lastCalibrationResult = null;

            UpdateSampleCount();

            TxtSelectedSample.Text = "未选择";
            TxtCalibrationResult.Text = "尚未执行标定。";

            AddLog("[Sample] 已清空所有样本。");
        }

        private void BtnSolveCalibration_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_samples.Count < 4)
                {
                    AddLog("[Solve] 样本数量较少，至少需要 4 组，建议 10～20 组。");

                    System.Windows.MessageBox.Show(
                        "当前样本数量较少。\n至少建议采集 4 组，最好采集 10～20 组不同姿态的数据。",
                        "样本数量不足",
                        MessageBoxButton.OK,
                        MessageBoxImage.Warning);

                    return;
                }

                List<NdiRobotSampleItem> validSamples = _samples
                    .Where(s => s.RobotMatrix != null && s.NdiMatrix != null)
                    .ToList();

                if (validSamples.Count < 4)
                {
                    AddLog("[Solve] 有效样本数量不足。");
                    return;
                }

                List<Matrix<double>> robotPoseList = validSamples
                    .Select(s => s.RobotMatrix.Clone())
                    .ToList();

                List<Matrix<double>> ndiPoseList = validSamples
                    .Select(s => s.NdiMatrix.Clone())
                    .ToList();

                if (robotPoseList.Count != ndiPoseList.Count)
                {
                    AddLog("[Solve] 机器人样本数量和 NDI 样本数量不一致。");
                    return;
                }

                AddLog($"[Solve] 开始执行手眼标定，有效样本数={robotPoseList.Count}。");

                Matrix<double> calibrationResult =
                    HandEyeCalibration_NDI.SolveHandEye(robotPoseList, ndiPoseList);

                if (calibrationResult == null)
                {
                    AddLog("[Solve ERROR] HandEyeCalibration.SolveHandEye 返回 null。");
                    return;
                }

                _lastCalibrationResult = calibrationResult.Clone();

                string resultText =
                    "NDI-机器人手眼标定结果：\n\n" +
                    FormatMatrix("T_robot_ndi / 39ToRobot", _lastCalibrationResult) +
                    "\n\n" +
                    FormatEulerXYZDeg(_lastCalibrationResult) +
                    "\n" +
                    $"样本数量：{robotPoseList.Count}\n" +
                    $"机器人位姿源：{(_useActualRobotT06 ? "ActualRobot_T06" : "ActualRobot_T0Tcp")}\n" +
                    $"生成时间：{DateTime.Now:yyyy-MM-dd HH:mm:ss}\n";

                TxtCalibrationResult.Text = resultText;

                AddLog("[Solve] 手眼标定完成。");
                AddLog("[Solve] " + FormatPoseShort(_lastCalibrationResult));
            }
            catch (Exception ex)
            {
                AddLog("[Solve ERROR] 手眼标定求解失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "手眼标定求解失败：\n" + ex.Message,
                    "求解失败",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }
        }

        private void BtnSaveCalibration_Click(object sender, RoutedEventArgs e)
        {
            if (_lastCalibrationResult == null)
            {
                AddLog("[Save] 当前没有可保存的标定结果，请先执行求解。");

                System.Windows.MessageBox.Show(
                    "当前没有可保存的标定结果，请先执行求解。",
                    "无标定结果",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);

                return;
            }

            SaveMarker2RobotJson(_lastCalibrationResult);
        }

        private void SaveMarker2RobotJson(Matrix<double> calibrationResult)
        {
            try
            {
                int rows = calibrationResult.RowCount;
                int cols = calibrationResult.ColumnCount;

                double[][] matrixData = new double[rows][];

                for (int r = 0; r < rows; r++)
                {
                    matrixData[r] = new double[cols];

                    for (int c = 0; c < cols; c++)
                    {
                        matrixData[r][c] = calibrationResult[r, c];
                    }
                }

                var saveData = new
                {
                    name = "NDI39_To_Robot",
                    tool = "8700339",
                    toolHandle = "01",
                    robotPoseSource = _useActualRobotT06 ? "ActualRobot_T06" : "ActualRobot_T0Tcp",
                    sampleCount = _samples.Count,
                    createdTime = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"),
                    matrix = matrixData
                };

                string baseDir = Path.Combine(
                    AppDomain.CurrentDomain.BaseDirectory,
                    "MemoryLog",
                    "NdiToRobot"
                );

                if (!Directory.Exists(baseDir))
                {
                    Directory.CreateDirectory(baseDir);
                }

                string fileName = $"39ToRobot_{DateTime.Now:yyyyMMdd_HHmmss}.json";
                string fullPath = Path.Combine(baseDir, fileName);

                JsonSerializerOptions options = new JsonSerializerOptions
                {
                    WriteIndented = true
                };

                string json = JsonSerializer.Serialize(saveData, options);

                File.WriteAllText(fullPath, json);

                AddLog("[Save] 手眼标定结果已保存至：" + fullPath);

                TxtCalibrationResult.Text =
                    TxtCalibrationResult.Text +
                    "\n保存路径：\n" +
                    fullPath;

                System.Windows.MessageBox.Show(
                    "手眼标定结果已保存：\n" + fullPath,
                    "保存成功",
                    MessageBoxButton.OK,
                    MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                AddLog("[Save ERROR] 保存手眼标定结果失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "保存手眼标定结果失败：\n" + ex.Message,
                    "保存失败",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }
        }

        /// <summary>
        /// 当前 XAML 里日志文本框和清空日志按钮已经注释掉。
        /// 这里保留函数，防止以后恢复按钮时缺少事件函数。
        /// 公共日志框由 calibrationWindow 管理，这里不主动清空。
        /// </summary>
        private void BtnClearLog_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[Log] 当前页面使用公共日志框，清空操作请由主日志框处理。");
        }

        private Matrix<double> ConvertSystemNumericsToMathNet(Matrix4x4 m)
        {
            return DenseMatrix.OfArray(new double[,]
            {
                { m.M11, m.M12, m.M13, m.M14 },
                { m.M21, m.M22, m.M23, m.M24 },
                { m.M31, m.M32, m.M33, m.M34 },
                { m.M41, m.M42, m.M43, m.M44 }
            });
        }

        private string FormatJointText(double[] qDeg)
        {
            if (qDeg == null || qDeg.Length < 7)
                return "q1=--, q2=--, q3=--, q4=--, q5=--, q6=--, q7=--";

            return
                $"q1={qDeg[0]:F2}°, q2={qDeg[1]:F2}°, q3={qDeg[2]:F2}°,\n" +
                $"q4={qDeg[3]:F2}°, q5={qDeg[4]:F2}°, q6={qDeg[5]:F2}°, q7={qDeg[6]:F2}°";
        }

        private string FormatPoseShort(Matrix<double> T)
        {
            if (T == null || T.RowCount < 4 || T.ColumnCount < 4)
                return "x=--, y=--, z=--";

            return $"x={T[0, 3]:F2}, y={T[1, 3]:F2}, z={T[2, 3]:F2}";
        }

        private string FormatMatrix(string name, Matrix<double> T)
        {
            if (T == null)
                return name + " = null";

            return
                $"{name} =\n" +
                $"[{T[0, 0],10:F6} {T[0, 1],10:F6} {T[0, 2],10:F6} {T[0, 3],10:F3}]\n" +
                $"[{T[1, 0],10:F6} {T[1, 1],10:F6} {T[1, 2],10:F6} {T[1, 3],10:F3}]\n" +
                $"[{T[2, 0],10:F6} {T[2, 1],10:F6} {T[2, 2],10:F6} {T[2, 3],10:F3}]\n" +
                $"[{T[3, 0],10:F6} {T[3, 1],10:F6} {T[3, 2],10:F6} {T[3, 3],10:F6}]";
        }

        private string FormatEulerXYZDeg(Matrix<double> T)
        {
            if (T == null)
                return "Euler XYZ = --";

            double r00 = T[0, 0];
            double r10 = T[1, 0];
            double r20 = T[2, 0];
            double r21 = T[2, 1];
            double r22 = T[2, 2];

            double ay = Math.Asin(Clamp(-r20, -1.0, 1.0));
            double ax = Math.Atan2(r21, r22);
            double az = Math.Atan2(r10, r00);

            double toDeg = 180.0 / Math.PI;

            return
                $"Euler XYZ deg = X={ax * toDeg:F3}, Y={ay * toDeg:F3}, Z={az * toDeg:F3}\n" +
                $"Translation mm = X={T[0, 3]:F3}, Y={T[1, 3]:F3}, Z={T[2, 3]:F3}";
        }

        private double Clamp(double value, double min, double max)
        {
            if (value < min)
                return min;

            if (value > max)
                return max;

            return value;
        }

        private void UpdateSampleCount()
        {
            TxtSampleCount.Text = _samples.Count.ToString();
        }

        private void RenumberSamples()
        {
            for (int i = 0; i < _samples.Count; i++)
            {
                _samples[i].Index = i + 1;
            }

            SampleDataGrid.Items.Refresh();
        }

        private void AddLog(string message)
        {
            _externalLog?.Invoke(message);
        }
    }

    public class NdiRobotSampleItem
    {
        public int Index { get; set; }

        public string CaptureTime { get; set; }

        public string RobotPoseFull { get; set; }

        public string NdiPoseFull { get; set; }

        public string RobotPoseShort { get; set; }

        public string NdiPoseShort { get; set; }

        public string StateText { get; set; }

        /// <summary>
        /// 实际参与标定的机器人矩阵。
        /// 默认是 T_base_T06。
        /// </summary>
        public Matrix<double> RobotMatrix { get; set; }

        /// <summary>
        /// 实际参与标定的 NDI 39 矩阵。
        /// </summary>
        public Matrix<double> NdiMatrix { get; set; }
    }
}