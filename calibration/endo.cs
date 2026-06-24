using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Windows.Media.Imaging;
using DirectShowLib;
using OpenCvSharp;
using OpenCvSharp.WpfExtensions;
using CvSize = OpenCvSharp.Size;

namespace WpfRobot.calibration
{
    public class CameraDetector
    {
        public List<string> GetCameraDevices()
        {
            List<string> cameraDevices = new List<string>();

            DsDevice[] devices = DsDevice.GetDevicesOfCat(FilterCategory.VideoInputDevice);

            foreach (DsDevice device in devices)
            {
                cameraDevices.Add(device.Name);
                device.Dispose();
            }

            return cameraDevices;
        }
    }
    /// <summary>
    /// 单目内窥镜内参标定算法模块。
    /// 
    /// 当前版本适用于：
    /// 1. 7 x 7 对称圆点标定板
    /// 2. 圆点中心间距 2.5 mm
    /// 3. OpenCV FindCirclesGrid + CalibrateCamera
    /// </summary>
    public class endo
    {
        public int PatternCols { get; }
        public int PatternRows { get; }
        public float SpacingMm { get; }

        /// <summary>
        /// 建议实际标定时至少 10 张以上有效图像。
        /// 这里设置为 5 是为了和你当前 UI 逻辑兼容。
        /// </summary>
        public int MinValidImages { get; set; } = 5;

        private CvSize PatternSize => new CvSize(PatternCols, PatternRows);

        /// <summary>
        /// 默认就是你的标定板：
        /// 7 x 7 对称圆点阵列，圆心间距 2.5 mm。
        /// </summary>
        public endo(int patternCols = 7, int patternRows = 7, float squareSizeMm = 2.5f)
        {
            if (patternCols <= 1 || patternRows <= 1)
                throw new ArgumentException("圆点标定板的行列点数必须大于 1。");

            if (squareSizeMm <= 0)
                throw new ArgumentException("圆点间距必须大于 0。");

            PatternCols = patternCols;
            PatternRows = patternRows;
            SpacingMm = squareSizeMm;
        }

        /// <summary>
        /// 检测单张 BitmapSource 图像中的圆点标定板。
        /// 可用于 UI 中的“检测标定板”按钮。
        /// </summary>
        public BoardDetectionResult DetectBoard(BitmapSource bitmapSource)
        {
            if (bitmapSource == null)
                throw new ArgumentNullException(nameof(bitmapSource));

            using Mat bgr = BitmapSourceToBgrMat(bitmapSource);
            return DetectBoard(bgr);
        }

        /// <summary>
        /// 检测单张 Mat 图像中的圆点标定板。
        /// </summary>
        public BoardDetectionResult DetectBoard(Mat inputImage)
        {
            if (inputImage == null || inputImage.Empty())
                throw new ArgumentException("输入图像为空。");

            using Mat gray = ToGray(inputImage);

            bool found = FindCircleGridCenters(gray, out Point2f[] centers);

            using Mat view = ToBgr(inputImage);

            if (found)
            {
                // 虽然函数名字叫 DrawChessboardCorners，
                // 但 OpenCV 也常用它来画圆点阵列检测结果。
                Cv2.DrawChessboardCorners(view, PatternSize, centers, true);
            }

            BitmapSource debugBitmap = BitmapSourceConverter.ToBitmapSource(view);
            debugBitmap.Freeze();

            return new BoardDetectionResult
            {
                Found = found,
                CornerCount = found ? centers.Length : 0,
                Corners = found ? centers : Array.Empty<Point2f>(),
                DebugImage = debugBitmap
            };
        }

        /// <summary>
        /// 从 WPF BitmapSource 列表执行单目标定。
        /// </summary>
        public EndoCalibrationResult CalibrateFromBitmapSources(IEnumerable<BitmapSource> bitmapSources)
        {
            if (bitmapSources == null)
                throw new ArgumentNullException(nameof(bitmapSources));

            List<Mat> mats = new List<Mat>();

            try
            {
                foreach (BitmapSource bitmap in bitmapSources)
                {
                    if (bitmap == null)
                        continue;

                    mats.Add(BitmapSourceToBgrMat(bitmap));
                }

                return CalibrateFromMats(mats);
            }
            finally
            {
                foreach (Mat mat in mats)
                {
                    mat.Dispose();
                }
            }
        }

        /// <summary>
        /// 从 Mat 图像列表执行单目标定。
        /// </summary>
        /// <summary>
        /// 从 Mat 图像列表执行单目标定。
        /// </summary>
        public EndoCalibrationResult CalibrateFromMats(IList<Mat> images)
        {
            if (images == null || images.Count == 0)
                throw new ArgumentException("没有输入标定图像。");

            // 注意：
            // 你当前 OpenCvSharp 版本的 Cv2.CalibrateCamera()
            // 要求 objectPoints / imagePoints 是 IEnumerable<Mat>
            List<Mat> objectPoints = new List<Mat>();
            List<Mat> imagePoints = new List<Mat>();

            List<EndoCalibrationImageInfo> imageInfos = new List<EndoCalibrationImageInfo>();

            Point3f[] objectPointTemplate = CreateObjectPointTemplate();

            CvSize imageSize = default;
            bool hasImageSize = false;

            try
            {
                for (int i = 0; i < images.Count; i++)
                {
                    Mat image = images[i];

                    if (image == null || image.Empty())
                    {
                        imageInfos.Add(new EndoCalibrationImageInfo
                        {
                            Index = i + 1,
                            Detected = false,
                            CornerCount = 0,
                            Message = "图像为空"
                        });
                        continue;
                    }

                    if (!hasImageSize)
                    {
                        imageSize = new CvSize(image.Width, image.Height);
                        hasImageSize = true;
                    }
                    else if (image.Width != imageSize.Width || image.Height != imageSize.Height)
                    {
                        throw new InvalidOperationException(
                            $"第 {i + 1} 张图像尺寸与第一张不一致。所有标定图像必须尺寸相同。");
                    }

                    using Mat gray = ToGray(image);

                    bool found = FindCircleGridCenters(gray, out Point2f[] centers);

                    if (found)
                    {
                        // OpenCV 标定需要：
                        // objectPoints: 每张图对应的三维圆点坐标，类型 CV_32FC3
                        // imagePoints : 每张图检测到的二维圆心坐标，类型 CV_32FC2
                        Mat objMat = Mat.FromArray(objectPointTemplate);
                        Mat imgMat = Mat.FromArray(centers);

                        objectPoints.Add(objMat);
                        imagePoints.Add(imgMat);

                        imageInfos.Add(new EndoCalibrationImageInfo
                        {
                            Index = i + 1,
                            Detected = true,
                            CornerCount = centers.Length,
                            Message = "圆点标定板检测成功"
                        });
                    }
                    else
                    {
                        imageInfos.Add(new EndoCalibrationImageInfo
                        {
                            Index = i + 1,
                            Detected = false,
                            CornerCount = 0,
                            Message = "未检测到完整 7x7 对称圆点标定板"
                        });
                    }
                }

                if (imagePoints.Count < MinValidImages)
                {
                    throw new InvalidOperationException(
                        $"有效标定图像数量不足：当前有效 {imagePoints.Count} 张，至少需要 {MinValidImages} 张。");
                }

                using Mat cameraMatrix = Mat.Eye(3, 3, MatType.CV_64FC1).ToMat();
                using Mat distCoeffs = new Mat();

                TermCriteria criteria = new TermCriteria(
                    CriteriaTypes.Eps | CriteriaTypes.MaxIter,
                    100,
                    1e-6);

                Mat[] rvecs;
                Mat[] tvecs;

                double rmsError = Cv2.CalibrateCamera(
                    objectPoints,
                    imagePoints,
                    imageSize,
                    cameraMatrix,
                    distCoeffs,
                    out rvecs,
                    out tvecs,
                    CalibrationFlags.None,
                    criteria);

                double fx = cameraMatrix.At<double>(0, 0);
                double fy = cameraMatrix.At<double>(1, 1);
                double cx = cameraMatrix.At<double>(0, 2);
                double cy = cameraMatrix.At<double>(1, 2);

                double[] cameraMatrixArray =
                {
            cameraMatrix.At<double>(0, 0), cameraMatrix.At<double>(0, 1), cameraMatrix.At<double>(0, 2),
            cameraMatrix.At<double>(1, 0), cameraMatrix.At<double>(1, 1), cameraMatrix.At<double>(1, 2),
            cameraMatrix.At<double>(2, 0), cameraMatrix.At<double>(2, 1), cameraMatrix.At<double>(2, 2)
        };

                double[] distArray = ExtractDistCoeffs(distCoeffs);

                foreach (Mat r in rvecs)
                    r.Dispose();

                foreach (Mat t in tvecs)
                    t.Dispose();

                return new EndoCalibrationResult
                {
                    ImageWidth = imageSize.Width,
                    ImageHeight = imageSize.Height,

                    PatternCols = PatternCols,
                    PatternRows = PatternRows,
                    SpacingMm = SpacingMm,

                    TotalImageCount = images.Count,
                    ValidImageCount = imagePoints.Count,

                    Fx = fx,
                    Fy = fy,
                    Cx = cx,
                    Cy = cy,

                    CameraMatrix = cameraMatrixArray,
                    DistCoeffs = distArray,

                    RmsReprojectionError = rmsError,
                    Images = imageInfos
                };
            }
            finally
            {
                // objectPoints / imagePoints 里面的 Mat 是我们手动 new 出来的，
                // 必须释放，避免内存泄漏。
                foreach (Mat mat in objectPoints)
                    mat.Dispose();

                foreach (Mat mat in imagePoints)
                    mat.Dispose();
            }
        }

        /// <summary>
        /// 检测 7x7 对称圆点阵列中心。
        /// 
        /// 你的标定板是规则排列：
        /// ● ● ● ● ● ● ●
        /// ● ● ● ● ● ● ●
        /// ...
        /// 所以使用 FindCirclesGridFlags.SymmetricGrid。
        /// </summary>
        private bool FindCircleGridCenters(Mat gray, out Point2f[] centers)
        {
            if (gray == null || gray.Empty())
            {
                centers = Array.Empty<Point2f>();
                return false;
            }

            using Mat gray8 = EnsureGray8(gray);

            bool found = Cv2.FindCirclesGrid(
                gray8,
                PatternSize,
                out centers,
                FindCirclesGridFlags.SymmetricGrid);

            return found;
        }

        /// <summary>
        /// 构造圆点标定板的三维物点。
        /// 对于 7x7，间距 2.5 mm：
        /// (0, 0, 0), (2.5, 0, 0), ..., (15, 15, 0)
        /// </summary>
        private Point3f[] CreateObjectPointTemplate()
        {
            List<Point3f> points = new List<Point3f>();

            for (int row = 0; row < PatternRows; row++)
            {
                for (int col = 0; col < PatternCols; col++)
                {
                    points.Add(new Point3f(
                        col * SpacingMm,
                        row * SpacingMm,
                        0.0f));
                }
            }

            return points.ToArray();
        }

        private static Mat BitmapSourceToBgrMat(BitmapSource bitmapSource)
        {
            using Mat src = BitmapSourceConverter.ToMat(bitmapSource);

            if (src.Empty())
                throw new ArgumentException("BitmapSource 转 Mat 失败。");

            Mat bgr = new Mat();

            if (src.Channels() == 1)
            {
                Cv2.CvtColor(src, bgr, ColorConversionCodes.GRAY2BGR);
            }
            else if (src.Channels() == 3)
            {
                bgr = src.Clone();
            }
            else if (src.Channels() == 4)
            {
                Cv2.CvtColor(src, bgr, ColorConversionCodes.BGRA2BGR);
            }
            else
            {
                throw new NotSupportedException($"不支持的图像通道数：{src.Channels()}");
            }

            return bgr;
        }

        private static Mat ToGray(Mat input)
        {
            Mat gray = new Mat();

            if (input.Channels() == 1)
            {
                gray = input.Clone();
            }
            else if (input.Channels() == 3)
            {
                Cv2.CvtColor(input, gray, ColorConversionCodes.BGR2GRAY);
            }
            else if (input.Channels() == 4)
            {
                Cv2.CvtColor(input, gray, ColorConversionCodes.BGRA2GRAY);
            }
            else
            {
                throw new NotSupportedException($"不支持的图像通道数：{input.Channels()}");
            }

            return gray;
        }

        private static Mat ToBgr(Mat input)
        {
            Mat bgr = new Mat();

            if (input.Channels() == 1)
            {
                Cv2.CvtColor(input, bgr, ColorConversionCodes.GRAY2BGR);
            }
            else if (input.Channels() == 3)
            {
                bgr = input.Clone();
            }
            else if (input.Channels() == 4)
            {
                Cv2.CvtColor(input, bgr, ColorConversionCodes.BGRA2BGR);
            }
            else
            {
                throw new NotSupportedException($"不支持的图像通道数：{input.Channels()}");
            }

            return bgr;
        }

        private static Mat EnsureGray8(Mat gray)
        {
            if (gray.Type() == MatType.CV_8UC1)
                return gray.Clone();

            Mat gray8 = new Mat();

            if (gray.Depth() == MatType.CV_8U)
            {
                gray8 = gray.Clone();
            }
            else
            {
                Cv2.Normalize(gray, gray8, 0, 255, NormTypes.MinMax);
                gray8.ConvertTo(gray8, MatType.CV_8UC1);
            }

            return gray8;
        }

        private static double[] ExtractDistCoeffs(Mat distCoeffs)
        {
            int n = (int)distCoeffs.Total();
            double[] values = new double[n];

            for (int i = 0; i < n; i++)
            {
                if (distCoeffs.Rows == 1)
                    values[i] = distCoeffs.At<double>(0, i);
                else
                    values[i] = distCoeffs.At<double>(i, 0);
            }

            return values;
        }
    }

    public class BoardDetectionResult
    {
        public bool Found { get; set; }

        public int CornerCount { get; set; }

        public Point2f[] Corners { get; set; }

        public BitmapSource DebugImage { get; set; }
    }

    public class EndoCalibrationResult
    {
        public int ImageWidth { get; set; }

        public int ImageHeight { get; set; }

        public int PatternCols { get; set; }

        public int PatternRows { get; set; }

        /// <summary>
        /// 圆点中心间距，单位 mm。
        /// </summary>
        public float SpacingMm { get; set; }

        public int TotalImageCount { get; set; }

        public int ValidImageCount { get; set; }

        public double Fx { get; set; }

        public double Fy { get; set; }

        public double Cx { get; set; }

        public double Cy { get; set; }

        /// <summary>
        /// 3x3 camera matrix，按行展开。
        /// [fx, 0, cx,
        ///  0, fy, cy,
        ///  0, 0, 1]
        /// </summary>
        public double[] CameraMatrix { get; set; }

        /// <summary>
        /// 默认普通畸变模型通常为：
        /// [k1, k2, p1, p2, k3]
        /// </summary>
        public double[] DistCoeffs { get; set; }

        public double RmsReprojectionError { get; set; }

        public List<EndoCalibrationImageInfo> Images { get; set; }

        public string ToDisplayText()
        {
            StringBuilder sb = new StringBuilder();

            sb.AppendLine("单目内窥镜内参标定完成");
            sb.AppendLine($"图像尺寸: {ImageWidth} x {ImageHeight}");
            sb.AppendLine($"圆点标定板: {PatternCols} x {PatternRows}");
            sb.AppendLine($"圆点中心间距: {SpacingMm:F3} mm");
            sb.AppendLine($"有效图像: {ValidImageCount} / {TotalImageCount}");
            sb.AppendLine();
            sb.AppendLine($"fx = {Fx:F6}");
            sb.AppendLine($"fy = {Fy:F6}");
            sb.AppendLine($"cx = {Cx:F6}");
            sb.AppendLine($"cy = {Cy:F6}");
            sb.AppendLine();
            sb.AppendLine("cameraMatrix = ");
            sb.AppendLine($"[{CameraMatrix[0]:F6}, {CameraMatrix[1]:F6}, {CameraMatrix[2]:F6}]");
            sb.AppendLine($"[{CameraMatrix[3]:F6}, {CameraMatrix[4]:F6}, {CameraMatrix[5]:F6}]");
            sb.AppendLine($"[{CameraMatrix[6]:F6}, {CameraMatrix[7]:F6}, {CameraMatrix[8]:F6}]");
            sb.AppendLine();
            sb.AppendLine("distCoeffs = ");
            sb.AppendLine("[" + string.Join(", ", DistCoeffs.Select(v => v.ToString("F8"))) + "]");
            sb.AppendLine();
            sb.AppendLine($"RMS reprojection error = {RmsReprojectionError:F6} px");

            return sb.ToString();
        }

        public void SaveAsJson(string filePath)
        {
            JsonSerializerOptions options = new JsonSerializerOptions
            {
                WriteIndented = true
            };

            string json = JsonSerializer.Serialize(this, options);
            File.WriteAllText(filePath, json, Encoding.UTF8);
        }
    }

    public class EndoCalibrationImageInfo
    {
        public int Index { get; set; }

        public bool Detected { get; set; }

        public int CornerCount { get; set; }

        public string Message { get; set; }
    }
}