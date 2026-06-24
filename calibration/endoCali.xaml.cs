using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Controls;
using System.Windows.Media.Imaging;
using System.Collections.ObjectModel;
using System.Collections.Generic;
using DirectShowLib;
using System.Threading;
using System.Threading.Tasks;
using OpenCvSharp;
using OpenCvSharp.WpfExtensions;

namespace WpfRobot.calibration
{
    /// <summary>
    /// endoCali.xaml 的交互逻辑
    /// </summary>
    public partial class endoCali : System.Windows.Controls.UserControl
    {
        private readonly ObservableCollection<CapturedImageItem> _capturedImages =
            new ObservableCollection<CapturedImageItem>();

        private bool _isCameraRunning = false;
        private VideoCapture _capture;
        private CancellationTokenSource _cameraCts;
        private Task _cameraTask;
        private readonly object _cameraLock = new object();


        private readonly endo _endoCalibrator = new endo();

        private EndoCalibrationResult _lastCalibrationResult;

        /// <summary>
        /// 外部日志回调，由主窗口 calibrationWindow 传入
        /// </summary>
        private Action<string> _externalLog;
        public endoCali() : this(null)
        {
        }
        public endoCali(Action<string> externalLog)
        {
            InitializeComponent();

            _externalLog = externalLog;

            CapturedImagesList.ItemsSource = _capturedImages;

            UpdateImageCount();

            check_cams_on_computer();

            Unloaded += EndoCali_Unloaded;

            AddLog("[EndoCali] 单目内窥镜标定页面已打开。");
        }
        private void BtnRefreshCamera_Click(object sender, RoutedEventArgs e)
        {
            check_cams_on_computer();
        }
        private async void EndoCali_Unloaded(object sender, RoutedEventArgs e)
        {
            await StopCameraAsync();
        }

        private void check_cams_on_computer()
        {
            Cam_comboBox.Items.Clear();

            try
            {
                CameraDetector cameraDetector = new CameraDetector();
                List<string> cameraDevices = cameraDetector.GetCameraDevices();

                foreach (string cameraDevice in cameraDevices)
                {
                    Cam_comboBox.Items.Add(cameraDevice);
                }

                if (Cam_comboBox.Items.Count > 0)
                {
                    Cam_comboBox.SelectedIndex = 0;

                    AddLog($"[Camera] 检测到 {Cam_comboBox.Items.Count} 个相机设备。当前选择：{Cam_comboBox.SelectedItem}");
                }
                else
                {
                    Cam_comboBox.Items.Add("未检测到相机");
                    Cam_comboBox.SelectedIndex = 0;

                    AddLog("[Camera] 未检测到电脑相机设备。");
                }
            }
            catch (Exception ex)
            {
                Cam_comboBox.Items.Add("相机检测失败");
                Cam_comboBox.SelectedIndex = 0;

                AddLog("[Camera] 检测相机失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "检测电脑相机失败：\n" + ex.Message,
                    "相机检测失败",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
            }
        }

        /// <summary>
        /// 打开相机
        /// </summary>
        private async void BtnOpenCamera_Click(object sender, RoutedEventArgs e)
        {
            if (Cam_comboBox.SelectedItem == null ||
                Cam_comboBox.SelectedItem.ToString() == "未检测到相机" ||
                Cam_comboBox.SelectedItem.ToString() == "相机检测失败")
            {
                System.Windows.MessageBox.Show(
                    "请先选择有效的相机设备。",
                    "未选择相机",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);

                AddLog("[Camera] 未选择有效相机，无法打开。");
                return;
            }

            if (_isCameraRunning)
            {
                AddLog("[Camera] 相机已经在预览中。");
                return;
            }

            string selectedCameraName = Cam_comboBox.SelectedItem.ToString();
            int selectedCameraIndex = Cam_comboBox.SelectedIndex;

            try
            {
                BtnOpenCamera.IsEnabled = false;

                AddLog($"[Camera] 正在打开相机：Index={selectedCameraIndex}, Name={selectedCameraName}");

                // 如果之前有残留资源，先释放
                await StopCameraAsync();

                _cameraCts = new CancellationTokenSource();

                _capture = new VideoCapture();

                // Windows 下建议使用 DirectShow，和 DirectShowLib 枚举顺序更接近
                bool opened = _capture.Open(selectedCameraIndex, VideoCaptureAPIs.DSHOW);

                if (!opened || !_capture.IsOpened())
                {
                    _capture?.Release();
                    _capture?.Dispose();
                    _capture = null;

                    throw new Exception($"OpenCV 打开相机失败：Index={selectedCameraIndex}, Name={selectedCameraName}");
                }

                // 可选：设置分辨率。根据你的相机支持情况调整。
                // 不支持时 OpenCV 会自动忽略或返回近似值。
                _capture.Set(VideoCaptureProperties.FrameWidth, 1280);
                _capture.Set(VideoCaptureProperties.FrameHeight, 720);
                _capture.Set(VideoCaptureProperties.Fps, 30);

                _isCameraRunning = true;

                TxtCameraStatus.Text = "相机预览中";
                TxtCameraStatus.Foreground =
                    new SolidColorBrush(System.Windows.Media.Color.FromRgb(37, 99, 235));
                TxtLivePlaceholder.Visibility = Visibility.Collapsed;

                _cameraTask = Task.Run(() =>
                {
                    CameraLoop(_cameraCts.Token);
                });

                AddLog($"[Camera] 相机预览已启动：Index={selectedCameraIndex}, Name={selectedCameraName}");
            }
            catch (Exception ex)
            {
                _isCameraRunning = false;

                TxtCameraStatus.Text = "相机打开失败";
                TxtCameraStatus.Foreground =
                    new SolidColorBrush(System.Windows.Media.Color.FromRgb(220, 38, 38));

                AddLog("[Camera] 打开相机失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    "打开相机失败：\n" + ex.Message,
                    "相机错误",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }
            finally
            {
                BtnOpenCamera.IsEnabled = true;
            }
        }

        private void CameraLoop(CancellationToken token)
        {
            using Mat frame = new Mat();

            while (!token.IsCancellationRequested)
            {
                try
                {
                    VideoCapture capture;

                    lock (_cameraLock)
                    {
                        capture = _capture;
                    }

                    if (capture == null || !capture.IsOpened())
                        break;

                    bool ok = capture.Read(frame);

                    if (!ok || frame.Empty())
                    {
                        Thread.Sleep(10);
                        continue;
                    }

                    BitmapSource bitmap = BitmapSourceConverter.ToBitmapSource(frame);
                    bitmap.Freeze();

                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        ImgLive.Source = bitmap;
                        TxtLivePlaceholder.Visibility = Visibility.Collapsed;
                    }));
                }
                catch (Exception ex)
                {
                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        AddLog("[Camera] 读取相机帧异常：" + ex.Message);
                    }));

                    Thread.Sleep(50);
                }

                Thread.Sleep(1);
            }
        }

        /// <summary>
        /// 停止相机
        /// </summary>
        private async void BtnStopCamera_Click(object sender, RoutedEventArgs e)
        {
            await StopCameraAsync();

            TxtCameraStatus.Text = "相机已停止";
            TxtCameraStatus.Foreground =
                new SolidColorBrush(System.Windows.Media.Color.FromRgb(100, 116, 139));

            TxtLivePlaceholder.Visibility = Visibility.Visible;

            AddLog("[Camera] 停止相机预览。");
        }


        private async Task StopCameraAsync()
        {
            try
            {
                _isCameraRunning = false;

                if (_cameraCts != null)
                {
                    _cameraCts.Cancel();
                }

                if (_cameraTask != null)
                {
                    try
                    {
                        await Task.WhenAny(_cameraTask, Task.Delay(1000));
                    }
                    catch
                    {
                        // 忽略相机线程退出异常
                    }
                }

                lock (_cameraLock)
                {
                    if (_capture != null)
                    {
                        try
                        {
                            _capture.Release();
                        }
                        catch { }

                        try
                        {
                            _capture.Dispose();
                        }
                        catch { }

                        _capture = null;
                    }
                }

                if (_cameraCts != null)
                {
                    _cameraCts.Dispose();
                    _cameraCts = null;
                }

                _cameraTask = null;
            }
            catch (Exception ex)
            {
                AddLog("[Camera] 停止相机异常：" + ex.Message);
            }
        }

        /// <summary>
        /// 外部相机线程可以调用这个函数更新实时画面
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
        /// 拍照采集当前帧
        /// </summary>
        private void BtnCapture_Click(object sender, RoutedEventArgs e)
        {
            if (!_isCameraRunning)
            {
                AddLog("[Capture] 相机未打开，无法采集图像。");
                System.Windows.MessageBox.Show(
                    "请先打开相机预览。",
                    "无法采集",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
                return;
            }

            BitmapSource currentFrame = ImgLive.Source as BitmapSource;

            if (currentFrame == null)
            {
                AddLog("[Capture] 当前没有有效实时画面。");
                System.Windows.MessageBox.Show(
                    "当前没有有效实时画面。",
                    "无法采集",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
                return;
            }

            BitmapSource snapshot = currentFrame.Clone();
            snapshot.Freeze();

            int index = _capturedImages.Count + 1;

            CapturedImageItem item = new CapturedImageItem
            {
                Name = $"Image_{index:D2}",
                CaptureTime = DateTime.Now.ToString("HH:mm:ss.fff"),
                ImageSource = snapshot
            };

            _capturedImages.Add(item);
            CapturedImagesList.SelectedItem = item;

            UpdateImageCount();

            AddLog($"[Capture] 已采集第 {index} 张标定图像。");
        }

        /// <summary>
        /// 检测标定板
        /// </summary>
        private void BtnDetectBoard_Click(object sender, RoutedEventArgs e)
        {
            BitmapSource currentFrame = ImgLive.Source as BitmapSource;

            if (currentFrame == null)
            {
                AddLog("[Detect] 当前没有实时图像，无法检测标定板。");
                return;
            }

            try
            {
                BoardDetectionResult result = _endoCalibrator.DetectBoard(currentFrame);

                if (result.Found)
                {
                    AddLog($"[Detect] 标定板检测成功，角点数量 = {result.CornerCount}。");

                    ImgCaptured.Source = result.DebugImage;
                    TxtCapturedPlaceholder.Visibility = Visibility.Collapsed;
                    TxtCapturedInfo.Text = $"检测成功：角点数量 = {result.CornerCount}";
                }
                else
                {
                    AddLog("[Detect] 未检测到完整棋盘格。请调整标定板角度、距离或光照。");
                }
            }
            catch (Exception ex)
            {
                AddLog("[Detect] 标定板检测失败：" + ex.Message);
                System.Windows.MessageBox.Show(
                    ex.Message,
                    "检测失败",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }
        }

        /// <summary>
        /// 点击列表切换静态预览图
        /// </summary>
        private void CapturedImagesList_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (CapturedImagesList.SelectedItem is CapturedImageItem item)
            {
                ImgCaptured.Source = item.ImageSource;
                TxtCapturedPlaceholder.Visibility = Visibility.Collapsed;

                int index = CapturedImagesList.SelectedIndex + 1;
                int total = _capturedImages.Count;

                TxtSelectedImageIndex.Text = $"{index} / {total}";
                TxtCapturedInfo.Text = $"当前查看：{item.Name}，采集时间：{item.CaptureTime}";
            }
            else
            {
                ImgCaptured.Source = null;
                TxtCapturedPlaceholder.Visibility = Visibility.Visible;
                TxtSelectedImageIndex.Text = "未选择";
                TxtCapturedInfo.Text = "当前未采集图像。建议采集 10～20 张不同角度、不同位置的标定板图像。";
            }
        }

        /// <summary>
        /// 上一张
        /// </summary>
        private void BtnPreviousImage_Click(object sender, RoutedEventArgs e)
        {
            if (_capturedImages.Count == 0)
                return;

            int index = CapturedImagesList.SelectedIndex;

            if (index <= 0)
                index = _capturedImages.Count - 1;
            else
                index--;

            CapturedImagesList.SelectedIndex = index;
            CapturedImagesList.ScrollIntoView(CapturedImagesList.SelectedItem);
        }

        /// <summary>
        /// 下一张
        /// </summary>
        private void BtnNextImage_Click(object sender, RoutedEventArgs e)
        {
            if (_capturedImages.Count == 0)
                return;

            int index = CapturedImagesList.SelectedIndex;

            if (index < 0 || index >= _capturedImages.Count - 1)
                index = 0;
            else
                index++;

            CapturedImagesList.SelectedIndex = index;
            CapturedImagesList.ScrollIntoView(CapturedImagesList.SelectedItem);
        }

        /// <summary>
        /// 删除当前图片
        /// </summary>
        private void BtnDeleteImage_Click(object sender, RoutedEventArgs e)
        {
            if (CapturedImagesList.SelectedItem is not CapturedImageItem item)
            {
                AddLog("[Image] 未选择需要删除的图像。");
                return;
            }

            int oldIndex = CapturedImagesList.SelectedIndex;

            _capturedImages.Remove(item);

            if (_capturedImages.Count > 0)
            {
                if (oldIndex >= _capturedImages.Count)
                    oldIndex = _capturedImages.Count - 1;

                CapturedImagesList.SelectedIndex = oldIndex;
            }
            else
            {
                CapturedImagesList.SelectedIndex = -1;
            }

            UpdateImageCount();

            AddLog($"[Image] 删除图像：{item.Name}");
        }

        /// <summary>
        /// 清空所有图片
        /// </summary>
        private void BtnClearImages_Click(object sender, RoutedEventArgs e)
        {
            if (_capturedImages.Count == 0)
                return;

            MessageBoxResult result = System.Windows.MessageBox.Show(
                "确定要清空所有已采集图像吗？",
                "确认清空",
                MessageBoxButton.YesNo,
                MessageBoxImage.Question);

            if (result != MessageBoxResult.Yes)
                return;

            _capturedImages.Clear();

            ImgCaptured.Source = null;
            TxtCapturedPlaceholder.Visibility = Visibility.Visible;
            TxtSelectedImageIndex.Text = "未选择";
            TxtCapturedInfo.Text = "当前未采集图像。建议采集 10～20 张不同角度、不同位置的标定板图像。";

            UpdateImageCount();

            AddLog("[Image] 已清空所有采集图像。");
        }

        /// <summary>
        /// 执行内参标定
        /// </summary>
        private void BtnRunCalibration_Click(object sender, RoutedEventArgs e)
        {
            if (_capturedImages.Count < 5)
            {
                AddLog("[Calibration] 图像数量较少，建议至少采集 10 张以上。");

                System.Windows.MessageBox.Show(
                    "当前采集图像数量较少，建议至少采集 10 张以上不同姿态的标定板图像。",
                    "图像数量不足",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
                return;
            }

            try
            {
                AddLog("[Calibration] 开始执行单目内窥镜内参标定。");

                List<BitmapSource> images = _capturedImages.Select(item => item.ImageSource as BitmapSource).Where(img => img != null).ToList();

                _lastCalibrationResult = _endoCalibrator.CalibrateFromBitmapSources(images);

                AddLog("[Calibration] 标定完成。");
                AddLog($"[Calibration] 有效图像：{_lastCalibrationResult.ValidImageCount} / {_lastCalibrationResult.TotalImageCount}");
                AddLog($"[Calibration] fx={_lastCalibrationResult.Fx:F3}, fy={_lastCalibrationResult.Fy:F3}, cx={_lastCalibrationResult.Cx:F3}, cy={_lastCalibrationResult.Cy:F3}");
                AddLog($"[Calibration] RMS reprojection error = {_lastCalibrationResult.RmsReprojectionError:F4} px");

                // 如果你把 XAML 里的 TxtCalibrationResult 取消注释，可以打开这一行：
                // TxtCalibrationResult.Text = _lastCalibrationResult.ToDisplayText();

                System.Windows.MessageBox.Show(
                    _lastCalibrationResult.ToDisplayText(),
                    "标定完成",
                    MessageBoxButton.OK,
                    MessageBoxImage.Information);

                // 标定完成后自动弹出保存对话框，建议用户保存标定结果
                BtnSaveCalibration_JSON();
            }
            catch (Exception ex)
            {
                AddLog("[Calibration] 标定失败：" + ex.Message);

                System.Windows.MessageBox.Show(
                    ex.Message,
                    "标定失败",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }

        }

        /// <summary>
        /// 保存标定结果
        /// </summary>
        private void BtnSaveCalibration_JSON()
        {
            if (_lastCalibrationResult == null)
            {
                System.Windows.MessageBox.Show(
                    "当前还没有标定结果，请先执行标定。",
                    "无法保存",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
                return;
            }

            Microsoft.Win32.SaveFileDialog dialog = new Microsoft.Win32.SaveFileDialog
            {
                Title = "保存单目内窥镜内参",
                Filter = "JSON 文件 (*.json)|*.json",
                FileName = $"endo_intrinsic_{DateTime.Now:yyyyMMdd_HHmmss}.json"
            };

            bool? result = dialog.ShowDialog();

            if (result == true)
            {
                _lastCalibrationResult.SaveAsJson(dialog.FileName);
                AddLog("[Calibration] 标定结果已保存：" + dialog.FileName);
            }
        }

        private void BtnClearLog_Click(object sender, RoutedEventArgs e)
        {
            //TxtLog.Clear();
            //AddLog("[Log] 日志已清空。");
        }

        private void UpdateImageCount()
        {
            TxtImageCount.Text = _capturedImages.Count.ToString();
        }

        private void AddLog(string message)
        {
            if (_externalLog != null)
            {
                _externalLog(message);
            }
        }
    }

    public class CapturedImageItem
    {
        public string Name { get; set; }

        public string CaptureTime { get; set; }

        public ImageSource ImageSource { get; set; }
    }
}