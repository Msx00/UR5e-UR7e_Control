using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using WpfRobot.calibration;

namespace WpfRobot
{
    /// <summary>
    /// calibrationWindow.xaml 的交互逻辑
    /// </summary>
    public partial class calibrationWindow : Window
    {
        /// <summary>
        /// 保存 XAML 中原本的默认欢迎界面，作为标定导览页
        /// </summary>
        private object _guidePage;

        /// <summary>
        /// 子页面缓存，避免每次点击都重新创建页面
        /// </summary>
        private endoCali _endoCaliPage;
        private endoHandEye _endoHandEyePage;
        private motor7Cali _motor7CaliPage;
        private ndiRobotHandEye _ndiRobotHandEyePage;

        private System.Windows.Controls.UserControl _toolLengthPage;
        private System.Windows.Controls.UserControl _laserPointPage;

        public calibrationWindow()
        {
            InitializeComponent();

            // 保存 XAML 里原本的默认欢迎界面，作为“标定导览页”
            _guidePage = CalibrationContent.Content;

            TxtCalibrationStatus.Text = "标定导览";
            AddCalibrationLog("[Calibration] 器械标定窗口已打开。");
        }

        /// <summary>
        /// 显示标定导览页
        /// </summary>
        private void ShowGuidePage()
        {
            CalibrationContent.Content = _guidePage;
            TxtCalibrationStatus.Text = "标定导览";
            AddCalibrationLog("[Calibration] 切换到：标定导览页面。");
        }

        private void BtnCalibrationGuide_Click(object sender, RoutedEventArgs e)
        {
            ShowGuidePage();
        }

        private void BtnEndoIntrinsic_Click(object sender, RoutedEventArgs e)
        {
            if (_endoCaliPage == null)
                _endoCaliPage = new endoCali(AddCalibrationLog);

            ShowCalibrationPage(
                _endoCaliPage,
                "当前任务：单目内窥镜内参标定",
                "[Calibration] 切换到：单目内窥镜内参标定页面。"
            );
        }

        private void BtnEndoHandEye_Click(object sender, RoutedEventArgs e)
        {
            if (_endoHandEyePage == null)
                _endoHandEyePage = new endoHandEye(AddCalibrationLog);

            ShowCalibrationPage(
                _endoHandEyePage,
                "当前任务：内窥镜-机械臂手眼标定",
                "[Calibration] 切换到：内窥镜-机械臂手眼标定页面。"
            );
        }

        private void BtnMotor7Zero_Click(object sender, RoutedEventArgs e)
        {
            if (_motor7CaliPage == null)
                _motor7CaliPage = new motor7Cali(AddCalibrationLog);

            ShowCalibrationPage(
                _motor7CaliPage,
                "当前任务：第七电机零点标定",
                "[Calibration] 切换到：第七电机零点标定页面。"
            );
        }

        private void BtnNdiRobot_Click(object sender, RoutedEventArgs e)
        {
            if (_ndiRobotHandEyePage == null)
                _ndiRobotHandEyePage = new ndiRobotHandEye(AddCalibrationLog);

            ShowCalibrationPage(
                _ndiRobotHandEyePage,
                "当前任务：NDI-机械臂手眼标定",
                "[Calibration] 切换到：NDI-机械臂手眼标定页面。"
            );
        }

        private void BtnToolLength_Click(object sender, RoutedEventArgs e)
        {
            if (_toolLengthPage == null)
            {
                _toolLengthPage = CreatePlaceholderPage(
                    "④ 手术工具长度标定",
                    "该页面后续用于标定第七电机转轴上安装的手术工具长度。",
                    "输出：Tool_X, Tool_Y, Tool_Z"
                );
            }

            ShowCalibrationPage(
                _toolLengthPage,
                "当前任务：手术工具长度标定",
                "[Calibration] 切换到：手术工具长度标定页面。"
            );
        }

        private void BtnLaserPoint_Click(object sender, RoutedEventArgs e)
        {
            if (_laserPointPage == null)
            {
                _laserPointPage = CreatePlaceholderPage(
                    "⑥ 激光发射点标定",
                    "该页面后续用于标定激光发射点相对于工具坐标系或第七轴坐标系的位置。",
                    "输出：T_tool_laser 或 p_laser_in_tool"
                );
            }

            ShowCalibrationPage(
                _laserPointPage,
                "当前任务：激光发射点标定",
                "[Calibration] 切换到：激光发射点标定页面。"
            );
        }

        /// <summary>
        /// 统一切换右侧页面
        /// </summary>
        private void ShowCalibrationPage(System.Windows.Controls.UserControl page, string statusText, string logText)
        {
            CalibrationContent.Content = page;
            TxtCalibrationStatus.Text = statusText;
            AddCalibrationLog(logText);
        }

        private void BtnLoadCalibrationParams_Click(object sender, RoutedEventArgs e)
        {
            AddCalibrationLog("[Calibration] 点击：加载标定参数。");

            System.Windows.MessageBox.Show(
                "这里后续用于加载标定参数。",
                "加载标定参数",
                System.Windows.MessageBoxButton.OK,
                System.Windows.MessageBoxImage.Information
            );
        }

        private void BtnSaveAllCalibrationParams_Click(object sender, RoutedEventArgs e)
        {
            AddCalibrationLog("[Calibration] 点击：保存全部标定参数。");

            System.Windows.MessageBox.Show(
                "这里后续用于保存全部标定参数。",
                "保存标定参数",
                System.Windows.MessageBoxButton.OK,
                System.Windows.MessageBoxImage.Information
            );
        }

        private void BtnClearCalibrationLog_Click(object sender, RoutedEventArgs e)
        {
            TxtCalibrationLog.Clear();
            AddCalibrationLog("[Calibration] 日志已清空。");
        }

        /// <summary>
        /// 添加标定日志
        /// </summary>
        private void AddCalibrationLog(string message)
        {
            if (TxtCalibrationLog == null)
                return;

            string time = DateTime.Now.ToString("HH:mm:ss.fff");
            TxtCalibrationLog.AppendText($"[{time}] {message}{Environment.NewLine}");
            TxtCalibrationLog.ScrollToEnd();
        }

        /// <summary>
        /// 临时占位页面：用于还没有单独设计 xaml 的页面
        /// </summary>
        private System.Windows.Controls.UserControl CreatePlaceholderPage(string title, string description, string output)
        {
            Grid root = new Grid
            {
                Margin = new Thickness(4)
            };

            root.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            root.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });

            StackPanel header = new StackPanel
            {
                Margin = new Thickness(0, 0, 0, 14)
            };

            TextBlock titleBlock = new TextBlock
            {
                Text = title,
                FontSize = 24,
                FontWeight = FontWeights.Bold,
                Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(15, 23, 42)),
                Margin = new Thickness(0, 0, 0, 6)
            };

            TextBlock descBlock = new TextBlock
            {
                Text = description,
                FontSize = 14,
                Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(100, 116, 139)),
                TextWrapping = TextWrapping.Wrap
            };

            header.Children.Add(titleBlock);
            header.Children.Add(descBlock);

            Grid.SetRow(header, 0);
            root.Children.Add(header);

            Border card = new Border
            {
                Background = new SolidColorBrush(System.Windows.Media.Color.FromRgb(248, 250, 252)),
                BorderBrush = new SolidColorBrush(System.Windows.Media.Color.FromRgb(226, 232, 240)),
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(14),
                Padding = new Thickness(18)
            };

            StackPanel cardStack = new StackPanel();

            cardStack.Children.Add(new TextBlock
            {
                Text = "标定输出",
                FontSize = 17,
                FontWeight = FontWeights.Bold,
                Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(30, 41, 59)),
                Margin = new Thickness(0, 0, 0, 10)
            });

            cardStack.Children.Add(new TextBlock
            {
                Text = output,
                FontSize = 14,
                Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(51, 65, 85)),
                TextWrapping = TextWrapping.Wrap
            });

            cardStack.Children.Add(new TextBlock
            {
                Text = "后续可以把这个占位页替换成独立的 xaml System.Windows.Controls.UserControl 页面。",
                FontSize = 13,
                Foreground = new SolidColorBrush(System.Windows.Media.Color.FromRgb(100, 116, 139)),
                TextWrapping = TextWrapping.Wrap,
                Margin = new Thickness(0, 14, 0, 0)
            });

            card.Child = cardStack;

            Grid.SetRow(card, 1);
            root.Children.Add(card);

            return new System.Windows.Controls.UserControl
            {
                Content = root
            };
        }
    }
}