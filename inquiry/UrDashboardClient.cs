using System;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace WpfRobot.inquiry
{
    /// <summary>
    /// UR Dashboard 状态快照。
    /// Dashboard 主要用于低频状态查询，不适合读取实时关节/TCP数据。
    /// </summary>
    public class UrDashboardSnapshot
    {
        public DateTime LocalTime { get; set; } = DateTime.Now;

        public string OperationalModeRaw { get; set; }
        public string RobotModeRaw { get; set; }
        public string SafetyStatusRaw { get; set; }
        public string RunningRaw { get; set; }
        public string ProgramStateRaw { get; set; }
        public string LoadedProgramRaw { get; set; }
        public string RemoteControlRaw { get; set; }
        public string SerialNumberRaw { get; set; }
        public string RobotModelRaw { get; set; }
        public string PolyscopeVersionRaw { get; set; }

        public string OperationalMode => ParseValue(OperationalModeRaw);
        public string RobotMode => ParseValue(RobotModeRaw);
        public string SafetyStatus => ParseValue(SafetyStatusRaw);
        public string ProgramState => ProgramStateRaw ?? "";
        public string LoadedProgram => ParseValue(LoadedProgramRaw);
        public string SerialNumber => ParseValue(SerialNumberRaw);
        public string RobotModel => ParseValue(RobotModelRaw);
        public string PolyscopeVersion => ParseValue(PolyscopeVersionRaw);

        public bool IsProgramRunning
        {
            get
            {
                string s = RunningRaw ?? "";
                return s.IndexOf("true", StringComparison.OrdinalIgnoreCase) >= 0;
            }
        }

        public bool IsRemoteControl
        {
            get
            {
                string s = RemoteControlRaw ?? "";
                return s.IndexOf("true", StringComparison.OrdinalIgnoreCase) >= 0;
            }
        }

        public bool IsEmergencyOrFault
        {
            get
            {
                string s = SafetyStatus.ToUpperInvariant();
                return s.Contains("EMERGENCY") ||
                       s.Contains("PROTECTIVE_STOP") ||
                       s.Contains("FAULT") ||
                       s.Contains("VIOLATION");
            }
        }

        private static string ParseValue(string raw)
        {
            if (string.IsNullOrWhiteSpace(raw))
                return "";

            raw = raw.Trim();

            int index = raw.IndexOf(':');
            if (index >= 0 && index + 1 < raw.Length)
                return raw.Substring(index + 1).Trim();

            return raw;
        }
    }

    /// <summary>
    /// UR Dashboard Client.
    /// 默认端口：29999。
    /// </summary>
    public sealed class UrDashboardClient : IDisposable
    {
        private TcpClient _client;
        private NetworkStream _stream;
        private StreamReader _reader;
        private StreamWriter _writer;

        private readonly SemaphoreSlim _sendLock = new SemaphoreSlim(1, 1);

        public bool IsConnected => _client != null && _client.Connected;

        public async Task ConnectAsync(string robotIp, int port = 29999)
        {
            Dispose();

            _client = new TcpClient();
            _client.NoDelay = true;

            await _client.ConnectAsync(robotIp, port);

            _stream = _client.GetStream();

            _reader = new StreamReader(_stream, Encoding.ASCII);
            _writer = new StreamWriter(_stream, Encoding.ASCII)
            {
                AutoFlush = true,
                NewLine = "\n"
            };

            // Dashboard 连接成功后通常会返回一行欢迎信息：
            // Connected: Universal Robots Dashboard Server
            await _reader.ReadLineAsync();
        }

        public async Task<string> SendCommandAsync(string command)
        {
            if (_client == null || !_client.Connected)
                throw new InvalidOperationException("Dashboard 未连接。");

            await _sendLock.WaitAsync();

            try
            {
                await _writer.WriteLineAsync(command);
                string response = await _reader.ReadLineAsync();
                return response ?? "";
            }
            finally
            {
                _sendLock.Release();
            }
        }

        public async Task<UrDashboardSnapshot> ReadSnapshotAsync()
        {
            UrDashboardSnapshot s = new UrDashboardSnapshot();

            // 这些命令都是低频查询，不建议几十Hz刷新。
            s.OperationalModeRaw = await SafeCommandAsync("get operational mode");
            s.RobotModeRaw = await SafeCommandAsync("robotmode");
            s.SafetyStatusRaw = await SafeCommandAsync("safetystatus");
            s.RunningRaw = await SafeCommandAsync("running");
            s.ProgramStateRaw = await SafeCommandAsync("programState");
            s.LoadedProgramRaw = await SafeCommandAsync("get loaded program");
            s.RemoteControlRaw = await SafeCommandAsync("is in remote control");
            s.SerialNumberRaw = await SafeCommandAsync("get serial number");
            s.RobotModelRaw = await SafeCommandAsync("get robot model");

            // 不同 PolyScope 版本可能对大小写敏感性不同，这里做一个兼容。
            s.PolyscopeVersionRaw = await SafeCommandAsync("PolyscopeVersion");
            if (string.IsNullOrWhiteSpace(s.PolyscopeVersionRaw) ||
                s.PolyscopeVersionRaw.IndexOf("unknown", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                s.PolyscopeVersionRaw = await SafeCommandAsync("version");
            }

            s.LocalTime = DateTime.Now;
            return s;
        }

        private async Task<string> SafeCommandAsync(string command)
        {
            try
            {
                return await SendCommandAsync(command);
            }
            catch (Exception ex)
            {
                return "[ERROR] " + command + " : " + ex.Message;
            }
        }

        public async Task PowerOnAsync()
        {
            await SendCommandAsync("power on");
        }

        public async Task BrakeReleaseAsync()
        {
            await SendCommandAsync("brake release");
        }

        public async Task ClosePopupAsync()
        {
            await SendCommandAsync("close popup");
        }

        public async Task UnlockProtectiveStopAsync()
        {
            await SendCommandAsync("unlock protective stop");
        }

        public async Task StopProgramAsync()
        {
            await SendCommandAsync("stop");
        }

        public void Dispose()
        {
            try { _writer?.Dispose(); } catch { }
            try { _reader?.Dispose(); } catch { }
            try { _stream?.Dispose(); } catch { }
            try { _client?.Close(); } catch { }

            _writer = null;
            _reader = null;
            _stream = null;
            _client = null;
        }
    }
}
