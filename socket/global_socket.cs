using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using WpfRobot.inquiry;

namespace WpfRobot
{
    ///<summary>
    ///UR机械臂通讯
    /// 全局通信资源管理类。
    /// 只负责保存/关闭通信对象，不直接操作 WPF UI。
    /// </summary>
    public static class global_socket
    {
        /// <summary>
        /// 通讯端口
        /// </summary>
        // Dashboard Server：低频状态、使能、程序状态、安全状态
        public const int DefaultDashboardPort = 29999;
        // Primary / Secondary Interface：可接收 URScript
        public const int DefaultPrimaryScriptPort = 30001;
        public const int DefaultSecondaryScriptPort = 30002;
        // Realtime Interface：通常不建议你当前这种方式直接用来发 movej
        public const int DefaultRealtimePort = 30003;
        // RTDE：高频状态反馈
        public const int DefaultRtdePort = 30004;
        // 实际运行时使用的端口
        public static int DashboardPort = DefaultDashboardPort;
        public static int RtdePort = DefaultRtdePort;
        // 运动控制建议默认用 30002
        public static int MotionScriptPort = DefaultPrimaryScriptPort;

        /// <summary>
        /// 通信对象。
        /// </summary>
        // =========================================================
        // 1. UR Dashboard Client：低频状态 / 程序状态 / 安全状态
        // =========================================================
        public static UrDashboardClient DashboardClient = null;
        public static CancellationTokenSource DashboardCts = null;

        // =========================================================
        // 2. UR RTDE Client：高频实时状态
        // =========================================================
        public static UrRtdeClient RtdeClient = null;
        public static CancellationTokenSource RtdeCts = null;

        public static double LastRtdeTimestamp = -1.0;
        public static DateTime LastRtdeReceiveTime = DateTime.MinValue;

        // 新增：保存最新 RTDE 状态
        private static readonly object _rtdeStateLock = new object();
        public static UrRtdeState LastRtdeState = null;

        // 新增：保存最新 Dashboard 状态
        private static readonly object _dashboardStateLock = new object();
        public static UrDashboardSnapshot LastDashboardSnapshot = null;

        // =========================================================
        // 3. 原始 Socket：保留给你的运动控制指令使用
        // =========================================================
        public static Socket _socket;
        public static bool socketsuccess = false;

        public static string RobotIp = "";
        public static int RobotPort = 0;

        public static volatile bool _isClosingSocket = false;

        private static readonly object _lockObj = new object();

        // =========================
        // 网络状态监测，丢包和延迟统计
        // =========================
        public static CancellationTokenSource NetworkCts = null;
        public static int PingTotalCount = 0;
        public static int PingLostCount = 0;
        public static double LastNetworkDelayMs = -1.0;
        public static double PacketLossRate = 0.0;

        /// <summary>
        /// 快速设定与机械臂的运动控制 Socket 连接。
        /// </summary>
        /// <param name="robotIp"></param>
        /// <returns></returns>
        public static async Task ConnectMotionSocketAsync(string robotIp)
        {
            CloseSocket();

            Socket socket = new Socket(
                AddressFamily.InterNetwork,
                SocketType.Stream,
                ProtocolType.Tcp
            );

            socket.NoDelay = true;
            socket.SendTimeout = 3000;
            socket.ReceiveTimeout = 3000;

            await socket.ConnectAsync(robotIp, MotionScriptPort);

            _isClosingSocket = false;
            _socket = socket;
            socketsuccess = true;

            RobotIp = robotIp;
            RobotPort = MotionScriptPort;
        }

        // =========================================================
        // 4. 初始化 Dashboard
        // =========================================================
        public static void PrepareDashboardClient()
        {
            lock (_lockObj)
            {
                StopDashboardClient();

                DashboardCts = new CancellationTokenSource();
                DashboardClient = new UrDashboardClient();
            }
        }

        // =========================================================
        // 5. 初始化 RTDE
        // =========================================================
        public static void PrepareRtdeClient()
        {
            lock (_lockObj)
            {
                StopRtdeClient();

                RtdeCts = new CancellationTokenSource();
                RtdeClient = new UrRtdeClient();

                LastRtdeTimestamp = -1.0;
                LastRtdeReceiveTime = DateTime.MinValue;
            }
        }

        public static void UpdateRtdeState(UrRtdeState state)
        {
            if (state == null)
                return;

            lock (_rtdeStateLock)
            {
                LastRtdeState = state;
                LastRtdeReceiveTime = DateTime.Now;
            }
        }
        public static void UpdateDashboardState(UrDashboardSnapshot snapshot)
        {
            if (snapshot == null)
                return;

            lock (_dashboardStateLock)
            {
                LastDashboardSnapshot = snapshot;
            }
        }

        public static UrDashboardSnapshot GetLatestDashboardSnapshot()
        {
            lock (_dashboardStateLock)
            {
                return LastDashboardSnapshot;
            }
        }

        public static UrRtdeState GetLatestRtdeState()
        {
            lock (_rtdeStateLock)
            {
                return LastRtdeState;
            }
        }
        public static bool HasFreshRtdeState(int maxAgeMs = 1000)
        {
            lock (_rtdeStateLock)
            {
                if (LastRtdeState == null)
                    return false;

                double ageMs = (DateTime.Now - LastRtdeReceiveTime).TotalMilliseconds;
                return ageMs <= maxAgeMs;
            }
        }
        public static double[] GetActualQRad6()
        {
            lock (_rtdeStateLock)
            {
                if (LastRtdeState == null)
                    return null;

                double[] q = LastRtdeState.ActualQ;

                if (q == null || q.Length < 6)
                    return null;

                return q.Take(6).ToArray();
            }
        }
        // =========================================================
        // 6. 停止 Dashboard
        // =========================================================
        public static void StopDashboardClient()
        {
            try
            {
                DashboardCts?.Cancel();
                DashboardCts?.Dispose();
            }
            catch
            {
                //
            }
            finally
            {
                DashboardCts = null;
            }

            try
            {
                DashboardClient?.Dispose();
            }
            catch
            {
                //
            }
            finally
            {
                DashboardClient = null;
            }

            lock (_dashboardStateLock)
            {
                LastDashboardSnapshot = null;
            }
        }

        // =========================================================
        // 7. 停止 RTDE
        // =========================================================
        public static void StopRtdeClient()
        {
            try
            {
                RtdeCts?.Cancel();
                RtdeCts?.Dispose();
            }
            catch
            {
                //
            }
            finally
            {
                RtdeCts = null;
            }

            try
            {
                RtdeClient?.Dispose();
            }
            catch
            {
                //
            }
            finally
            {
                RtdeClient = null;
            }

            LastRtdeTimestamp = -1.0;
            LastRtdeReceiveTime = DateTime.MinValue;

            lock (_rtdeStateLock)
            {
                LastRtdeState = null;
            }
        }

        // =========================================================
        // 8. 关闭原始运动控制 Socket
        // =========================================================
        public static void CloseSocket()
        {
            _isClosingSocket = true;

            try
            {
                if (_socket != null)
                {
                    try
                    {
                        if (_socket.Connected)
                        {
                            _socket.Shutdown(SocketShutdown.Both);
                        }
                    }
                    catch
                    {
                        // 断开时 socket 可能已经失效，忽略即可
                    }

                    try
                    {
                        _socket.Close();
                    }
                    catch
                    {
                        //
                    }

                    try
                    {
                        _socket.Dispose();
                    }
                    catch
                    {
                        //
                    }

                    _socket = null;
                }
            }
            catch
            {
                _socket = null;
            }

            socketsuccess = false;
        }

        public static void PrepareNetworkMonitor()
        {
            StopNetworkMonitor();

            NetworkCts = new CancellationTokenSource();

            PingTotalCount = 0;
            PingLostCount = 0;
            LastNetworkDelayMs = -1.0;
            PacketLossRate = 0.0;
        }

        public static void StopNetworkMonitor()
        {
            try
            {
                NetworkCts?.Cancel();
                NetworkCts?.Dispose();
            }
            catch
            {
            }
            finally
            {
                NetworkCts = null;
            }
        }

        // =========================================================
        // 9. 关闭所有通信
        // =========================================================
        public static void CloseAll()
        {
            StopDashboardClient();
            StopRtdeClient();
            CloseSocket();
            StopNetworkMonitor();

            RobotIp = "";
            RobotPort = 0;
            socketsuccess = false;
            _isClosingSocket = true;
        }
    }
}
