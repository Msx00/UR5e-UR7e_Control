using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace WpfRobot.lowlevel_driver
{
    /// <summary>
    /// 底层运动驱动：
    /// 1. 通过 URScript Socket 发送 movej / stopj；
    /// 2. 通过 global_socket.LastRtdeState 读取 RTDE 实际关节角 actual_q；
    /// 3. 支持单点运动；
    /// 4. 支持轨迹点序列：发一个点 -> 等待到位 -> 再发下一个点。
    /// </summary>
    public class driver : IDisposable
    {
        private readonly object _sendLock = new object();

        private Socket _scriptSocket;

        private CancellationTokenSource _trajectoryCts;
        private bool _isTrajectoryRunning = false;

        private static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        public string IpAddress { get; private set; } = "";
        public int ScriptPort { get; private set; } = 30002;

        public bool IsScriptConnected
        {
            get { return _scriptSocket != null && _scriptSocket.Connected; }
        }

        public bool IsTrajectoryRunning
        {
            get { return _isTrajectoryRunning; }
        }

        /// <summary>
        /// 输出日志给 UI。
        /// </summary>
        public event Action<string> LogMessage;

        /// <summary>
        /// 轨迹执行进度。
        /// currentIndex: 当前点序号，从 1 开始
        /// totalCount: 总点数
        /// maxErrorRad: 当前最大关节误差，单位 rad
        /// </summary>
        public event Action<int, int, double> TrajectoryProgress;

        public driver()
        {
        }

        /// <summary>
        /// 连接 URScript Socket。
        /// 注意：这里不再连接 RTDE。
        /// RTDE 应该由 MainWindow 中的 StartRtdeMonitor(ip) 统一启动。
        /// </summary>
        public string Connect(
            string ip,
            int scriptPort = 30002)
        {
            if (string.IsNullOrWhiteSpace(ip))
                throw new ArgumentException("机器人 IP 不能为空。", nameof(ip));

            DisconnectScriptOnly();

            StringBuilder log = new StringBuilder();

            IpAddress = ip;
            ScriptPort = scriptPort;

            _scriptSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp)
            {
                NoDelay = true,
                SendTimeout = 3000,
                ReceiveTimeout = 3000
            };

            _scriptSocket.Connect(ip, scriptPort);
            log.AppendLine($"URScript Socket 连接成功：{ip}:{scriptPort}");
            log.AppendLine("RTDE 状态读取由 global_socket.RtdeClient 统一提供。");

            Log(log.ToString());
            return log.ToString();
        }

        /// <summary>
        /// 只断开 URScript Socket，不断开全局 RTDE。
        /// </summary>
        public void DisconnectScriptOnly()
        {
            try
            {
                if (_scriptSocket != null)
                {
                    try
                    {
                        if (_scriptSocket.Connected)
                            _scriptSocket.Shutdown(SocketShutdown.Both);
                    }
                    catch { }

                    try { _scriptSocket.Close(); } catch { }
                    try { _scriptSocket.Dispose(); } catch { }

                    _scriptSocket = null;
                }
            }
            catch
            {
                _scriptSocket = null;
            }
        }

        /// <summary>
        /// 断开运动控制连接。
        /// 注意：不关闭全局 RTDE，因为 RTDE 是 MainWindow / global_socket 管理的。
        /// </summary>
        public void Disconnect()
        {
            StopTrajectory();
            DisconnectScriptOnly();
        }

        /// <summary>
        /// 发送任意 URScript。
        /// </summary>
        public void SendScript(string script)
        {
            if (string.IsNullOrWhiteSpace(script))
                throw new ArgumentException("URScript 指令不能为空。");

            if (_scriptSocket == null || !_scriptSocket.Connected)
                throw new InvalidOperationException("URScript Socket 尚未连接。");

            if (!script.EndsWith("\n"))
                script += "\n";

            byte[] data = Encoding.ASCII.GetBytes(script);

            lock (_sendLock)
            {
                int sent = 0;

                while (sent < data.Length)
                {
                    sent += _scriptSocket.Send(
                        data,
                        sent,
                        data.Length - sent,
                        SocketFlags.None
                    );
                }
            }
        }

        /// <summary>
        /// 单点关节运动，输入角度制。
        /// </summary>
        public void MoveJDeg(double[] qDeg, double a = 0.5, double v = 0.3)
        {
            ValidateSixJointArray(qDeg, nameof(qDeg));

            double[] qRad = qDeg.Select(DegToRad).ToArray();
            MoveJRad(qRad, a, v);
        }

        /// <summary>
        /// 单点关节运动，输入弧度制。
        /// </summary>
        public void MoveJRad(double[] qRad, double a = 0.5, double v = 0.3)
        {
            ValidateSixJointArray(qRad, nameof(qRad));

            string script = string.Format(
                CI,
                "movej({0}, a={1}, v={2})",
                FormatArray(qRad),
                FormatDouble(a),
                FormatDouble(v)
            );

            SendScript(script);
        }

        /// <summary>
        /// 单点关节运动：发送 movej 后等待到位。
        /// 输入角度制。
        /// </summary>
        public async Task<bool> MoveJDegAndWaitAsync(
            double[] qDeg,
            double a = 0.5,
            double v = 0.3,
            double toleranceDeg = 0.5,
            int timeoutMs = 20000,
            int pollIntervalMs = 20)
        {
            ValidateMotionReady();
            ValidateSixJointArray(qDeg, nameof(qDeg));

            double[] qRad = qDeg.Select(DegToRad).ToArray();
            double toleranceRad = DegToRad(toleranceDeg);

            MoveJRad(qRad, a, v);

            return await WaitUntilReachedRadAsync(
                qRad,
                toleranceRad,
                timeoutMs,
                pollIntervalMs,
                stableCount: 3,
                CancellationToken.None
            );
        }

        /// <summary>
        /// 执行关节轨迹：
        /// 第 i 个角度点发送后，等待实际关节角 actual_q 到位；
        /// 到位后才发送第 i+1 个点。
        /// 
        /// 注意：这里要求每个点是 6 个关节角，单位 degree。
        /// 如果你的逆解结果是 7 个关节角，UR 本体只能接收前 6 轴，第 7 轴需要单独电机驱动控制。
        /// </summary>
        public async Task<TrajectoryResult> ExecuteJointTrajectoryDegAsync(
            IEnumerable<double[]> qDegSequence,
            double a = 0.5,
            double v = 0.3,
            double toleranceDeg = 0.5,
            int timeoutMsPerPoint = 20000,
            int pollIntervalMs = 20,
            int stableCount = 3,
            CancellationToken externalToken = default(CancellationToken))
        {
            ValidateMotionReady();

            if (qDegSequence == null)
                throw new ArgumentNullException(nameof(qDegSequence));

            if (_isTrajectoryRunning)
                throw new InvalidOperationException("当前已经有轨迹正在执行，请先停止或等待执行完成。");

            List<double[]> qDegList = qDegSequence
                .Select(q =>
                {
                    ValidateSixJointArray(q, nameof(qDegSequence));
                    return (double[])q.Clone();
                })
                .ToList();

            if (qDegList.Count == 0)
                throw new ArgumentException("轨迹点序列为空。", nameof(qDegSequence));

            List<double[]> qRadList = qDegList
                .Select(q => q.Select(DegToRad).ToArray())
                .ToList();

            double toleranceRad = DegToRad(toleranceDeg);

            _trajectoryCts = CancellationTokenSource.CreateLinkedTokenSource(externalToken);
            CancellationToken ct = _trajectoryCts.Token;

            _isTrajectoryRunning = true;

            try
            {
                Log($"开始执行关节轨迹，共 {qRadList.Count} 个点。");

                for (int i = 0; i < qRadList.Count; i++)
                {
                    ct.ThrowIfCancellationRequested();

                    double[] targetQRad = qRadList[i];

                    Log($"发送第 {i + 1}/{qRadList.Count} 个关节目标点。");

                    MoveJRad(targetQRad, a, v);

                    bool reached = await WaitUntilReachedRadAsync(
                        targetQRad,
                        toleranceRad,
                        timeoutMsPerPoint,
                        pollIntervalMs,
                        stableCount,
                        ct,
                        currentIndex: i + 1,
                        totalCount: qRadList.Count
                    );

                    if (!reached)
                    {
                        string msg = $"第 {i + 1} 个轨迹点超时未到位，轨迹停止。";
                        Log(msg);

                        try { StopJ(); } catch { }

                        return new TrajectoryResult
                        {
                            Success = false,
                            ReachedCount = i,
                            FailedIndex = i,
                            Message = msg
                        };
                    }

                    Log($"第 {i + 1}/{qRadList.Count} 个轨迹点已到位。");
                }

                Log("轨迹执行完成。");

                return new TrajectoryResult
                {
                    Success = true,
                    ReachedCount = qRadList.Count,
                    FailedIndex = -1,
                    Message = "轨迹执行完成。"
                };
            }
            catch (OperationCanceledException)
            {
                string msg = "轨迹执行被取消。";
                Log(msg);

                try { StopJ(); } catch { }

                return new TrajectoryResult
                {
                    Success = false,
                    ReachedCount = 0,
                    FailedIndex = -1,
                    Message = msg
                };
            }
            finally
            {
                _isTrajectoryRunning = false;

                if (_trajectoryCts != null)
                {
                    _trajectoryCts.Dispose();
                    _trajectoryCts = null;
                }
            }
        }

        /// <summary>
        /// 停止当前轨迹执行。
        /// </summary>
        public void StopTrajectory()
        {
            try
            {
                if (_trajectoryCts != null)
                    _trajectoryCts.Cancel();
            }
            catch { }

            try
            {
                if (IsScriptConnected)
                    StopJ();
            }
            catch { }
        }

        /// <summary>
        /// URScript stopj。
        /// </summary>
        public void StopJ(double a = 2.0)
        {
            string script = string.Format(
                CI,
                "stopj({0})",
                FormatDouble(a)
            );

            SendScript(script);
        }

        /// <summary>
        /// 等待实际关节角接近目标关节角。
        /// </summary>
        private async Task<bool> WaitUntilReachedRadAsync(
            double[] targetQRad,
            double toleranceRad,
            int timeoutMs,
            int pollIntervalMs,
            int stableCount,
            CancellationToken ct,
            int currentIndex = 0,
            int totalCount = 0)
        {
            ValidateSixJointArray(targetQRad, nameof(targetQRad));

            Stopwatch sw = Stopwatch.StartNew();

            int stableCounter = 0;
            int loopCounter = 0;

            while (sw.ElapsedMilliseconds < timeoutMs)
            {
                ct.ThrowIfCancellationRequested();

                double[] actualQRad = global_socket.GetActualQRad6();

                if (actualQRad == null)
                {
                    await Task.Delay(pollIntervalMs, ct);
                    continue;
                }

                double maxErrorRad = MaxAbsJointErrorRad(actualQRad, targetQRad);

                if (currentIndex > 0 && totalCount > 0 && loopCounter % 5 == 0)
                {
                    TrajectoryProgress?.Invoke(currentIndex, totalCount, maxErrorRad);
                }

                if (maxErrorRad <= toleranceRad)
                {
                    stableCounter++;

                    if (stableCounter >= stableCount)
                        return true;
                }
                else
                {
                    stableCounter = 0;
                }

                loopCounter++;

                await Task.Delay(pollIntervalMs, ct);
            }

            return false;
        }

        /// <summary>
        /// 判断运动系统是否准备好。
        /// </summary>
        private void ValidateMotionReady()
        {
            if (_scriptSocket == null || !_scriptSocket.Connected)
                throw new InvalidOperationException("URScript Socket 尚未连接，不能发送运动指令。");

            if (global_socket.RtdeClient == null)
                throw new InvalidOperationException("全局 RTDE Client 为空，请先连接机器人并启动 StartRtdeMonitor。");

            if (!global_socket.HasFreshRtdeState(1500))
                throw new InvalidOperationException("RTDE 尚未收到新鲜的 actual_q 状态，无法判断机器人是否到位。");
        }

        private static void ValidateSixJointArray(double[] q, string name)
        {
            if (q == null)
                throw new ArgumentNullException(name);

            if (q.Length != 6)
                throw new ArgumentException("当前 UR movej 关节数组长度必须为 6。", name);

            for (int i = 0; i < q.Length; i++)
            {
                if (double.IsNaN(q[i]) || double.IsInfinity(q[i]))
                    throw new ArgumentException($"第 {i + 1} 个关节角不是有效数字。", name);
            }
        }

        private static double MaxAbsJointErrorRad(double[] actualQRad, double[] targetQRad)
        {
            ValidateSixJointArray(actualQRad, nameof(actualQRad));
            ValidateSixJointArray(targetQRad, nameof(targetQRad));

            double maxError = 0.0;

            for (int i = 0; i < 6; i++)
            {
                double err = Math.Abs(actualQRad[i] - targetQRad[i]);

                if (err > maxError)
                    maxError = err;
            }

            return maxError;
        }

        private static double DegToRad(double deg)
        {
            return deg * Math.PI / 180.0;
        }

        private static string FormatDouble(double value)
        {
            return value.ToString("G17", CI);
        }

        private static string FormatArray(double[] values)
        {
            return "[" + string.Join(", ", values.Select(FormatDouble)) + "]";
        }

        private void Log(string msg)
        {
            LogMessage?.Invoke(msg);
        }

        public void Dispose()
        {
            Disconnect();
        }
    }

    /// <summary>
    /// 轨迹执行结果。
    /// </summary>
    public class TrajectoryResult
    {
        public bool Success { get; set; }

        /// <summary>
        /// 已经成功到达的点数量。
        /// </summary>
        public int ReachedCount { get; set; }

        /// <summary>
        /// 失败点下标，从 0 开始。
        /// 成功时为 -1。
        /// </summary>
        public int FailedIndex { get; set; } = -1;

        public string Message { get; set; } = "";
    }
}
