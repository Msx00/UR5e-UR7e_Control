using System;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace WpfRobot.calibration
{
    /// <summary>
    /// endoHandEye.xaml 的交互逻辑
    /// </summary>
    public partial class endoHandEye : System.Windows.Controls.UserControl
    {
        private readonly ObservableCollection<HandEyeSampleItem> _samples =
            new ObservableCollection<HandEyeSampleItem>();

        private bool _isCameraRunning = false;

        private bool _hasRobotPose = false;
        private bool _hasBoardPose = false;

        private string _currentRobotPoseText = "";
        private string _currentBoardPoseText = "";

        /// <summary>
        /// 外部日志回调，由主窗口 calibrationWindow 传入
        /// </summary>
        private Action<string> _externalLog;
        public endoHandEye() : this(null)
        {
        }
        public endoHandEye(Action<string> externalLog)
        {
            InitializeComponent();

            SampleDataGrid.ItemsSource = _samples;

            UpdateSampleCount();

            AddLog("[HandEye] 内窥镜-机械臂手眼标定页面已打开。");
        }

        /// <summary>
        /// 打开相机
        /// </summary>
        private void BtnOpenCamera_Click(object sender, RoutedEventArgs e)
        {
            _isCameraRunning = true;

            TxtCameraStatus.Text = "相机预览中";
            TxtCameraStatus.Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));
            TxtLivePlaceholder.Visibility = Visibility.Collapsed;

            AddLog("[Camera] 打开内窥镜相机预览。");

            // TODO:
            // 后续接入真实内窥镜相机。
            // 可以使用 OpenCvSharp / 相机 SDK 获取图像，
            // 然后调用 SetLiveFrame(bitmapSource) 更新实时窗口。
        }

        /// <summary>
        /// 停止相机
        /// </summary>
        private void BtnStopCamera_Click(object sender, RoutedEventArgs e)
        {
            _isCameraRunning = false;

            TxtCameraStatus.Text = "相机已停止";
            TxtCameraStatus.Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(100, 116, 139));

            AddLog("[Camera] 停止内窥镜相机预览。");

            // TODO:
            // 后续释放相机资源。
        }

        /// <summary>
        /// 外部相机线程可以调用这个函数刷新实时画面
        /// </summary>
        public void SetLiveFrame(BitmapSource frame)
        {
            if (frame == null)
                return;

            Dispatcher.Invoke(() =>
            {
                ImgLive.Source = frame;
                TxtLivePlaceholder.Visibility = Visibility.Collapsed;
            });
        }

        /// <summary>
        /// 检测标定板，得到 T_camera_board
        /// </summary>
        private void BtnDetectBoard_Click(object sender, RoutedEventArgs e)
        {
            if (ImgLive.Source == null)
            {
                AddLog("[Detect] 当前没有有效内窥镜图像，无法检测标定板。");
                System.Windows.MessageBox.Show(
                    "当前没有有效内窥镜图像。",
                    "无法检测",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
                return;
            }

            AddLog("[Detect] 开始检测标定板。");

            // TODO:
            // 后续在这里接入 OpenCV 标定板检测：
            // 1. 棋盘格：FindChessboardCorners + SolvePnP
            // 2. Charuco：Aruco/Charuco 检测 + EstimatePose
            // 3. 输出 T_camera_board

            // 这里先放一个模拟结果，保证 UI 流程能跑通。
            _currentBoardPoseText =
                "T_camera_board\n" +
                "x=0.0 mm, y=0.0 mm, z=120.0 mm\n" +
                "rx=0.0°, ry=0.0°, rz=0.0°";

            _hasBoardPose = true;

            TxtBoardPose.Text = _currentBoardPoseText;
            TxtBoardDetectStatus.Text = "检测状态：已检测到标定板";
            TxtBoardDetectStatus.Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));

            AddLog("[Detect] 标定板检测完成，已得到 T_camera_board。");
        }

        /// <summary>
        /// 保存当前内窥镜画面
        /// </summary>
        private void BtnCaptureImage_Click(object sender, RoutedEventArgs e)
        {
            if (ImgLive.Source == null)
            {
                AddLog("[Image] 当前没有实时图像，无法保存。");
                return;
            }

            AddLog("[Image] 当前内窥镜图像已保存到当前样本缓存。");

            // TODO:
            // 如果后续你希望每组样本都保存对应图片，
            // 可以在这里 Clone ImgLive.Source，并保存到当前样本对象中。
        }

        /// <summary>
        /// 读取机器人当前位姿，得到 T_base_tool / T_base_7 / T_base_tcp
        /// </summary>
        private void BtnReadRobotPose_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[Robot] 读取机器人当前位姿。");

            // TODO:
            // 后续在这里接入你的机器人状态：
            // 方式1：读取 global_variable.globalJointDeg，然后正运动学计算 T_base_7 / T_base_tool
            // 方式2：读取 RTDE actual_q，再用正运动学计算真实 T_base_tcp
            // 方式3：直接读取机器人当前 TCP pose

            // 这里先放一个模拟结果，保证 UI 流程能跑通。
            _currentRobotPoseText =
                "T_base_tool\n" +
                "x=300.0 mm, y=0.0 mm, z=250.0 mm\n" +
                "rx=0.0°, ry=0.0°, rz=0.0°";

            _hasRobotPose = true;

            TxtRobotPose.Text = _currentRobotPoseText;

            AddLog("[Robot] 机器人当前位姿读取完成。");
        }

        /// <summary>
        /// 采集一组手眼标定样本
        /// </summary>
        private void BtnCapturePair_Click(object sender, RoutedEventArgs e)
        {
            if (!_hasRobotPose)
            {
                AddLog("[Sample] 尚未读取机器人位姿，无法采集样本。");
                System.Windows.MessageBox.Show(
                    "请先读取机器人当前位姿。",
                    "无法采集",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
                return;
            }

            if (!_hasBoardPose)
            {
                AddLog("[Sample] 尚未检测标定板位姿，无法采集样本。");
                System.Windows.MessageBox.Show(
                    "请先检测标定板位姿。",
                    "无法采集",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
                return;
            }

            int index = _samples.Count + 1;

            HandEyeSampleItem item = new HandEyeSampleItem
            {
                Index = index,
                CaptureTime = DateTime.Now.ToString("HH:mm:ss.fff"),
                RobotPoseText = _currentRobotPoseText,
                BoardPoseText = _currentBoardPoseText,
                StateText = "有效"
            };

            _samples.Add(item);
            SampleDataGrid.SelectedItem = item;
            SampleDataGrid.ScrollIntoView(item);

            UpdateSampleCount();

            AddLog($"[Sample] 已采集第 {index} 组手眼标定样本。");
        }

        /// <summary>
        /// 清空当前暂存位姿
        /// </summary>
        private void BtnClearCurrent_Click(object sender, RoutedEventArgs e)
        {
            _hasRobotPose = false;
            _hasBoardPose = false;

            _currentRobotPoseText = "";
            _currentBoardPoseText = "";

            TxtRobotPose.Text = "尚未读取机器人位姿";
            TxtBoardPose.Text = "尚未检测标定板位姿";
            TxtBoardDetectStatus.Text = "检测状态：未检测";
            TxtBoardDetectStatus.Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(100, 116, 139));

            AddLog("[Current] 已清空当前机器人位姿和标定板位姿。");
        }

        /// <summary>
        /// 选择样本
        /// </summary>
        private void SampleDataGrid_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            HandEyeSampleItem item = SampleDataGrid.SelectedItem as HandEyeSampleItem;

            if (item == null)
            {
                TxtSelectedSample.Text = "未选择";
                return;
            }

            TxtSelectedSample.Text = $"当前选择：#{item.Index}";
        }

        /// <summary>
        /// 删除当前样本
        /// </summary>
        private void BtnDeleteSample_Click(object sender, RoutedEventArgs e)
        {
            HandEyeSampleItem item = SampleDataGrid.SelectedItem as HandEyeSampleItem;

            if (item == null)
            {
                AddLog("[Sample] 未选择需要删除的样本。");
                return;
            }

            _samples.Remove(item);
            RenumberSamples();
            UpdateSampleCount();

            AddLog($"[Sample] 已删除样本 #{item.Index}。");
        }

        /// <summary>
        /// 清空全部样本
        /// </summary>
        private void BtnClearSamples_Click(object sender, RoutedEventArgs e)
        {
            if (_samples.Count == 0)
                return;

            MessageBoxResult result = System.Windows.MessageBox.Show(
                "确定要清空所有手眼标定样本吗？",
                "确认清空",
                MessageBoxButton.YesNo,
                MessageBoxImage.Question);

            if (result != MessageBoxResult.Yes)
                return;

            _samples.Clear();
            UpdateSampleCount();

            TxtSelectedSample.Text = "未选择";

            AddLog("[Sample] 已清空所有手眼标定样本。");
        }

        /// <summary>
        /// 执行手眼标定求解
        /// </summary>
        private void BtnSolveHandEye_Click(object sender, RoutedEventArgs e)
        {
            if (_samples.Count < 5)
            {
                AddLog("[Solve] 样本数量较少，建议至少采集 10 组以上。");

                System.Windows.MessageBox.Show(
                    "当前样本数量较少，建议至少采集 10 组以上不同姿态的数据。",
                    "样本数量不足",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
                return;
            }

            AddLog("[Solve] 开始执行内窥镜-机械臂手眼标定。");

            // TODO:
            // 后续在这里接入真正的手眼标定算法：
            //
            // 已知：
            // 1. 多组机器人位姿：T_base_tool_i
            // 2. 多组相机观测位姿：T_camera_board_i
            //
            // 求：
            // X = T_tool_camera
            //
            // 常见形式：
            // A_i X = X B_i
            //
            // 可使用 OpenCV CalibrateHandEye：
            // Cv2.CalibrateHandEye(
            //     R_gripper2base, t_gripper2base,
            //     R_target2cam, t_target2cam,
            //     out R_cam2gripper,
            //     out t_cam2gripper,
            //     HandEyeCalibrationMethod.Tsai
            // );

            //TxtHandEyeResult.Text =
            //    "手眼标定完成示例：\n" +
            //    "T_tool_camera =\n" +
            //    "[ 1.0000   0.0000   0.0000   0.0000 ]\n" +
            //    "[ 0.0000   1.0000   0.0000   0.0000 ]\n" +
            //    "[ 0.0000   0.0000   1.0000   0.0000 ]\n" +
            //    "[ 0.0000   0.0000   0.0000   1.0000 ]\n\n" +
            //    "平均重投影误差 / 位姿误差：待计算\n" +
            //    "当前为 UI 流程示例，后续接入 OpenCV CalibrateHandEye。";

            AddLog("[Solve] 手眼标定求解接口已触发。");
        }

        /// <summary>
        /// 保存手眼标定结果
        /// </summary>
        private void BtnSaveHandEye_Click(object sender, RoutedEventArgs e)
        {
            AddLog("[Save] 保存内窥镜-机械臂手眼标定结果。");

            // TODO:
            // 建议保存为 JSON：
            // T_tool_camera / T_7_camera / T_tcp_camera
            // calibration_method
            // sample_count
            // reprojection_error
            // created_time

            System.Windows.MessageBox.Show(
                "这里后续用于保存手眼标定矩阵。",
                "保存结果",
                MessageBoxButton.OK,
                MessageBoxImage.Information);
        }

        private void BtnClearLog_Click(object sender, RoutedEventArgs e)
        {
            //TxtLog.Clear();
            //AddLog("[Log] 日志已清空。");
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
            if (_externalLog != null)
            {
                _externalLog(message);
            }
        }
    }

    public class HandEyeSampleItem
    {
        public int Index { get; set; }

        public string CaptureTime { get; set; }

        public string RobotPoseText { get; set; }

        public string BoardPoseText { get; set; }

        public string StateText { get; set; }
    }
}