using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace WpfRobot.inquiry
{
    public class UrRtdeState
    {
        public DateTime LocalTime { get; set; } = DateTime.Now;

        public Dictionary<string, object> Values { get; } = new Dictionary<string, object>();

        public double Timestamp => GetDouble("timestamp") ?? 0.0;

        public double[] ActualQ => GetDoubleArray("actual_q");
        public double[] ActualQd => GetDoubleArray("actual_qd");
        public double[] ActualCurrent => GetDoubleArray("actual_current");
        public double[] ActualCurrentAsTorque => GetDoubleArray("actual_current_as_torque");

        public double[] TargetQ => GetDoubleArray("target_q");
        public double[] TargetQd => GetDoubleArray("target_qd");
        public double[] TargetQdd => GetDoubleArray("target_qdd");
        public double[] TargetMoment => GetDoubleArray("target_moment");

        public double[] ActualTcpPose => GetDoubleArray("actual_TCP_pose");
        public double[] ActualTcpSpeed => GetDoubleArray("actual_TCP_speed");
        public double[] ActualTcpForce => GetDoubleArray("actual_TCP_force");
        public double[] ActualTcpAcceleration => GetDoubleArray("actual_TCP_acceleration");

        public double[] TargetTcpPose => GetDoubleArray("target_TCP_pose");
        public double[] TargetTcpSpeed => GetDoubleArray("target_TCP_speed");
        public double[] TargetTcpAcceleration => GetDoubleArray("target_TCP_acceleration");

        public double[] TcpOffset => GetDoubleArray("tcp_offset");
        public double[] JointTemperatures => GetDoubleArray("joint_temperatures");
        public double[] ActualJointVoltage => GetDoubleArray("actual_joint_voltage");

        public double[] ActualToolAccelerometer => GetDoubleArray("actual_tool_accelerometer");

        public double? SpeedScaling => GetDouble("speed_scaling");
        public double? TargetSpeedFraction => GetDouble("target_speed_fraction");
        public double? ActualMomentum => GetDouble("actual_momentum");

        public double? ActualMainVoltage => GetDouble("actual_main_voltage");
        public double? ActualRobotVoltage => GetDouble("actual_robot_voltage");
        public double? ActualRobotCurrent => GetDouble("actual_robot_current");

        public double? StandardAnalogInput0 => GetDouble("standard_analog_input0");
        public double? StandardAnalogInput1 => GetDouble("standard_analog_input1");
        public double? StandardAnalogOutput0 => GetDouble("standard_analog_output0");
        public double? StandardAnalogOutput1 => GetDouble("standard_analog_output1");
        public double? IoCurrent => GetDouble("io_current");

        public double? ToolAnalogInput0 => GetDouble("tool_analog_input0");
        public double? ToolAnalogInput1 => GetDouble("tool_analog_input1");
        public double? ToolOutputCurrent => GetDouble("tool_output_current");
        public double? ToolTemperature => GetDouble("tool_temperature");
        public double? TcpForceScalar => GetDouble("tcp_force_scalar");

        public int? RobotMode => GetInt32("robot_mode");
        public int? SafetyStatus => GetInt32("safety_status");
        public uint? RuntimeState => GetUInt32("runtime_state");
        public uint? RobotStatusBits => GetUInt32("robot_status_bits");
        public uint? SafetyStatusBits => GetUInt32("safety_status_bits");

        public ulong? ActualDigitalInputBits => GetUInt64("actual_digital_input_bits");
        public ulong? ActualDigitalOutputBits => GetUInt64("actual_digital_output_bits");
        public ulong? ActualConfigurableDigitalInputBits => GetUInt64("actual_configurable_digital_input_bits");
        public ulong? ActualConfigurableDigitalOutputBits => GetUInt64("actual_configurable_digital_output_bits");

        public uint? AnalogIoTypes => GetUInt32("analog_io_types");
        public uint? ToolMode => GetUInt32("tool_mode");
        public uint? ToolAnalogInputTypes => GetUInt32("tool_analog_input_types");

        public int? ToolOutputVoltage => GetInt32("tool_output_voltage");

        public double TcpLinearSpeedMmPerSec
        {
            get
            {
                double[] v = ActualTcpSpeed;
                if (v == null || v.Length < 3)
                    return 0.0;

                return Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) * 1000.0;
            }
        }

        public double TcpLinearAccelerationMmPerSec2
        {
            get
            {
                double[] a = ActualTcpAcceleration;
                if (a == null || a.Length < 3)
                    return 0.0;

                return Math.Sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) * 1000.0;
            }
        }

        public bool IsPowerOn => GetRobotStatusBit(0);
        public bool IsProgramRunning => GetRobotStatusBit(1);
        public bool IsTeachButtonPressed => GetRobotStatusBit(2);
        public bool IsPowerButtonPressed => GetRobotStatusBit(3);

        public bool IsSafetyNormal => GetSafetyStatusBit(0);
        public bool IsReducedMode => GetSafetyStatusBit(1);
        public bool IsProtectiveStopped => GetSafetyStatusBit(2);
        public bool IsRecoveryMode => GetSafetyStatusBit(3);
        public bool IsSafeguardStopped => GetSafetyStatusBit(4);
        public bool IsSystemEmergencyStopped => GetSafetyStatusBit(5);
        public bool IsRobotEmergencyStopped => GetSafetyStatusBit(6);
        public bool IsEmergencyStopped => GetSafetyStatusBit(7);
        public bool IsSafetyViolation => GetSafetyStatusBit(8);
        public bool IsSafetyFault => GetSafetyStatusBit(9);
        public bool IsStoppedDueToSafety => GetSafetyStatusBit(10);

        public bool GetRobotStatusBit(int bit)
        {
            uint? bits = RobotStatusBits;
            if (!bits.HasValue) return false;
            return ((bits.Value >> bit) & 1u) != 0;
        }

        public bool GetSafetyStatusBit(int bit)
        {
            uint? bits = SafetyStatusBits;
            if (!bits.HasValue) return false;
            return ((bits.Value >> bit) & 1u) != 0;
        }

        public bool GetDigitalInputBit(int bit)
        {
            ulong? bits = ActualDigitalInputBits;
            if (!bits.HasValue) return false;
            return ((bits.Value >> bit) & 1UL) != 0;
        }

        public bool GetDigitalOutputBit(int bit)
        {
            ulong? bits = ActualDigitalOutputBits;
            if (!bits.HasValue) return false;
            return ((bits.Value >> bit) & 1UL) != 0;
        }

        public double? GetDouble(string name)
        {
            if (!Values.TryGetValue(name, out object value))
                return null;

            if (value is double d)
                return d;

            return null;
        }

        public double[] GetDoubleArray(string name)
        {
            if (!Values.TryGetValue(name, out object value))
                return null;

            return value as double[];
        }

        public int? GetInt32(string name)
        {
            if (!Values.TryGetValue(name, out object value))
                return null;

            if (value is int i)
                return i;

            return null;
        }

        public uint? GetUInt32(string name)
        {
            if (!Values.TryGetValue(name, out object value))
                return null;

            if (value is uint u)
                return u;

            return null;
        }

        public ulong? GetUInt64(string name)
        {
            if (!Values.TryGetValue(name, out object value))
                return null;

            if (value is ulong u)
                return u;

            return null;
        }
    }

    public static class UrRtdeText
    {
        public static string RobotModeToText(int? mode)
        {
            if (!mode.HasValue) return "未知";

            switch (mode.Value)
            {
                case 0: return "DISCONNECTED";
                case 1: return "CONFIRM_SAFETY";
                case 2: return "BOOTING";
                case 3: return "POWER_OFF";
                case 4: return "POWER_ON";
                case 5: return "IDLE";
                case 6: return "BACKDRIVE";
                case 7: return "RUNNING";
                case 8: return "UPDATING";
                case 9: return "POWERING_OFF";
                case 10: return "ARM_BOOTING";
                case 11: return "ARM_UPDATING";
                default: return "UNKNOWN(" + mode.Value + ")";
            }
        }

        public static string SafetyStatusToText(int? status)
        {
            if (!status.HasValue) return "未知";

            switch (status.Value)
            {
                case 1: return "NORMAL";
                case 2: return "REDUCED";
                case 3: return "PROTECTIVE_STOP";
                case 4: return "RECOVERY";
                case 5: return "SAFEGUARD_STOP";
                case 6: return "SYSTEM_EMERGENCY_STOP";
                case 7: return "ROBOT_EMERGENCY_STOP";
                case 8: return "VIOLATION";
                case 9: return "FAULT";
                case 12: return "AUTOMATIC_MODE_SAFEGUARD_STOP";
                case 13: return "SYSTEM_THREE_POSITION_ENABLING_STOP";
                case 14: return "TP_THREE_POSITION_ENABLING_STOP";
                case 15: return "IMMI_EMERGENCY_STOP";
                case 16: return "IMMI_SAFEGUARD_STOP";
                case 17: return "PROFISAFE_WAITING_FOR_PARAMETERS";
                case 18: return "PROFISAFE_AUTOMATIC_MODE_SAFEGUARD_STOP";
                case 19: return "PROFISAFE_SAFEGUARD_STOP";
                case 20: return "PROFISAFE_EMERGENCY_STOP";
                case 22: return "SAFETY_API_SAFEGUARD_STOP";
                default: return "UNKNOWN(" + status.Value + ")";
            }
        }

        public static string RuntimeStateToText(uint? state)
        {
            if (!state.HasValue) return "未知";

            switch (state.Value)
            {
                case 0: return "STOPPING";
                case 1: return "STOPPED";
                case 2: return "RUNNING";
                case 3: return "PAUSING";
                case 4: return "PAUSED";
                case 5: return "RESUMING";
                case 6: return "RETRACTING";
                default: return "UNKNOWN(" + state.Value + ")";
            }
        }
    }

    public sealed class UrRtdeClient : IDisposable
    {
        private const byte RTDE_REQUEST_PROTOCOL_VERSION = 86;
        private const byte RTDE_GET_URCONTROL_VERSION = 118;
        private const byte RTDE_TEXT_MESSAGE = 77;
        private const byte RTDE_DATA_PACKAGE = 85;
        private const byte RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79;
        private const byte RTDE_CONTROL_PACKAGE_START = 83;
        private const byte RTDE_CONTROL_PACKAGE_PAUSE = 80;

        private TcpClient _client;
        private NetworkStream _stream;

        private byte _outputRecipeId;
        private List<string> _activeVariables = new List<string>();
        private List<string> _activeTypes = new List<string>();
        private List<string> _unsupportedVariables = new List<string>();

        public event Action<UrRtdeState> StateReceived;
        public event Action<string> LogMessage;

        public IReadOnlyList<string> ActiveVariables => _activeVariables;
        public IReadOnlyList<string> ActiveTypes => _activeTypes;
        public IReadOnlyList<string> UnsupportedVariables => _unsupportedVariables;

        public bool IsConnected => _client != null && _client.Connected;

        /// <summary>
        /// 尽量完整的 RTDE 输出变量。
        /// 某些字段只在新版本 PolyScope 支持，脚本会自动剔除 NOT_FOUND 字段。
        /// </summary>
        public static readonly string[] DefaultOutputVariables =
        {
            // 时间
            "timestamp",

            // 目标关节状态
            "target_q",
            "target_qd",
            "target_qdd",
            "target_current",
            "target_moment",

            // 实际关节状态
            "actual_q",
            "actual_qd",
            "actual_current",
            "actual_current_as_torque",
            "joint_control_output",
            "joint_temperatures",
            "actual_joint_voltage",

            // TCP 状态
            "actual_TCP_pose",
            "actual_TCP_speed",
            "actual_TCP_force",
            "actual_TCP_acceleration",
            "target_TCP_pose",
            "target_TCP_speed",
            "target_TCP_acceleration",
            "tcp_offset",

            // IO
            "actual_digital_input_bits",
            "actual_configurable_digital_input_bits",
            "actual_digital_output_bits",
            "actual_configurable_digital_output_bits",
            "analog_io_types",
            "standard_analog_input0",
            "standard_analog_input1",
            "standard_analog_output0",
            "standard_analog_output1",
            "io_current",

            // 机器人实时状态
            "actual_execution_time",
            "target_execution_time",
            "robot_mode",
            "joint_mode",
            "safety_status",
            "runtime_state",
            "robot_status_bits",
            "safety_status_bits",

            // 工具 / 力 / 动量
            "actual_tool_accelerometer",
            "speed_scaling",
            "target_speed_fraction",
            "actual_momentum",
            "actual_main_voltage",
            "actual_robot_voltage",
            "actual_robot_current",

            // 工具端状态
            "tool_mode",
            "tool_analog_input_types",
            "tool_analog_input0",
            "tool_analog_input1",
            "tool_output_voltage",
            "tool_output_current",
            "tool_temperature",
            "tool_output_mode",
            "tool_digital_output0_mode",
            "tool_digital_output1_mode",

            // 力控 / 载荷 / 碰撞相关，新版本可能支持
            "tcp_force_scalar",
            "joint_position_deviation_ratio",
            "collision_detection_ratio",
            "ft_raw_wrench",
            "wrench_calc_from_currents",
            "payload",
            "payload_cog",
            "payload_inertia",
            "script_control_line",
            "time_scale_source"
        };

        public async Task ConnectAndStartAsync(
        string robotIp,
        int port = 30004,
        double frequency = 125.0,
        CancellationToken cancellationToken = default(CancellationToken))
        {
            Dispose();

            _client = new TcpClient();
            _client.NoDelay = true;

            await _client.ConnectAsync(robotIp, port);

            _stream = _client.GetStream();

            bool protocolOk = await RequestProtocolVersionAsync(2, cancellationToken);
            if (!protocolOk)
                throw new InvalidOperationException("RTDE protocol v2 请求失败。");

            await ReadUrControlVersionAsync(cancellationToken);

            await SetupOutputsWithFallbackAsync(DefaultOutputVariables, frequency, cancellationToken);

            bool started = await StartSynchronizationAsync(cancellationToken);
            if (!started)
                throw new InvalidOperationException("RTDE start synchronization 失败。");

            Log("[RTDE] 已启动，实际订阅字段数量 = " + _activeVariables.Count);

            await ReceiveLoopAsync(cancellationToken);
        }

        public async Task PauseAsync(CancellationToken cancellationToken = default(CancellationToken))
        {
            if (_stream == null)
                return;

            await SendPackageAsync(RTDE_CONTROL_PACKAGE_PAUSE, new byte[0], cancellationToken);

            RtdePacket packet = await ReceiveUntilCommandAsync(RTDE_CONTROL_PACKAGE_PAUSE, cancellationToken);
            if (packet.Payload.Length > 0 && packet.Payload[0] == 1)
                Log("[RTDE] 已暂停。");
        }

        private async Task<bool> RequestProtocolVersionAsync(ushort version, CancellationToken token)
        {
            byte[] payload = BigEndian.GetBytes(version);

            await SendPackageAsync(RTDE_REQUEST_PROTOCOL_VERSION, payload, token);

            RtdePacket packet = await ReceiveUntilCommandAsync(RTDE_REQUEST_PROTOCOL_VERSION, token);

            return packet.Payload.Length >= 1 && packet.Payload[0] == 1;
        }

        private async Task ReadUrControlVersionAsync(CancellationToken token)
        {
            try
            {
                await SendPackageAsync(RTDE_GET_URCONTROL_VERSION, new byte[0], token);

                RtdePacket packet = await ReceiveUntilCommandAsync(RTDE_GET_URCONTROL_VERSION, token);

                if (packet.Payload.Length >= 16)
                {
                    int offset = 0;
                    uint major = BigEndian.ReadUInt32(packet.Payload, ref offset);
                    uint minor = BigEndian.ReadUInt32(packet.Payload, ref offset);
                    uint bugfix = BigEndian.ReadUInt32(packet.Payload, ref offset);
                    uint build = BigEndian.ReadUInt32(packet.Payload, ref offset);

                    Log($"[RTDE] URControl Version: {major}.{minor}.{bugfix}.{build}");
                }
            }
            catch (Exception ex)
            {
                Log("[RTDE] 获取 URControl 版本失败：" + ex.Message);
            }
        }

        private async Task SetupOutputsWithFallbackAsync(
            string[] requestedVariables,
            double frequency,
            CancellationToken token)
        {
            SetupResult result = await SetupOutputsAsync(requestedVariables, frequency, token);

            if (!result.HasNotFound && result.RecipeId != 0)
            {
                _outputRecipeId = result.RecipeId;
                _activeVariables = result.Variables.ToList();
                _activeTypes = result.Types.ToList();
                _unsupportedVariables.Clear();
                return;
            }

            List<string> supportedVariables = new List<string>();
            List<string> unsupportedVariables = new List<string>();

            for (int i = 0; i < result.Variables.Count; i++)
            {
                string type = i < result.Types.Count ? result.Types[i] : "NOT_FOUND";

                if (string.Equals(type, "NOT_FOUND", StringComparison.OrdinalIgnoreCase))
                    unsupportedVariables.Add(result.Variables[i]);
                else
                    supportedVariables.Add(result.Variables[i]);
            }

            _unsupportedVariables = unsupportedVariables;

            if (unsupportedVariables.Count > 0)
            {
                Log("[RTDE] 当前机器人不支持以下字段，已自动移除：");
                foreach (string v in unsupportedVariables)
                    Log("  - " + v);
            }

            if (supportedVariables.Count == 0)
                throw new InvalidOperationException("没有任何 RTDE 输出字段可用。");

            SetupResult second = await SetupOutputsAsync(supportedVariables.ToArray(), frequency, token);

            if (second.HasNotFound || second.RecipeId == 0)
                throw new InvalidOperationException("RTDE 输出字段二次配置仍失败。");

            _outputRecipeId = second.RecipeId;
            _activeVariables = second.Variables.ToList();
            _activeTypes = second.Types.ToList();
        }

        private async Task<SetupResult> SetupOutputsAsync(
            string[] variables,
            double frequency,
            CancellationToken token)
        {
            List<byte> payload = new List<byte>();

            payload.AddRange(BigEndian.GetBytes(frequency));

            string names = string.Join(",", variables);
            payload.AddRange(Encoding.ASCII.GetBytes(names));

            await SendPackageAsync(
                RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
                payload.ToArray(),
                token);

            RtdePacket response = await ReceiveUntilCommandAsync(
                RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
                token);

            if (response.Payload.Length < 2)
                throw new InvalidDataException("RTDE setup outputs response 过短。");

            byte recipeId = response.Payload[0];

            string typeString = Encoding.ASCII.GetString(
                response.Payload,
                1,
                response.Payload.Length - 1).Trim('\0', '\r', '\n', ' ');

            string[] types = typeString.Split(new[] { ',' }, StringSplitOptions.None);

            SetupResult result = new SetupResult
            {
                RecipeId = recipeId,
                Variables = variables.ToList(),
                Types = types.ToList()
            };

            return result;
        }

        private async Task<bool> StartSynchronizationAsync(CancellationToken token)
        {
            await SendPackageAsync(RTDE_CONTROL_PACKAGE_START, new byte[0], token);

            RtdePacket response = await ReceiveUntilCommandAsync(RTDE_CONTROL_PACKAGE_START, token);

            return response.Payload.Length >= 1 && response.Payload[0] == 1;
        }

        private async Task ReceiveLoopAsync(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                RtdePacket packet = await ReceivePackageAsync(token);

                if (packet.Command == RTDE_DATA_PACKAGE)
                {
                    UrRtdeState state = ParseDataPackage(packet.Payload);
                    StateReceived?.Invoke(state);
                }
                else if (packet.Command == RTDE_TEXT_MESSAGE)
                {
                    string msg = Encoding.ASCII.GetString(packet.Payload);
                    Log("[RTDE TEXT] " + msg);
                }
            }
        }

        private UrRtdeState ParseDataPackage(byte[] payload)
        {
            UrRtdeState state = new UrRtdeState();

            if (payload == null || payload.Length < 1)
                return state;

            int offset = 0;

            byte recipeId = payload[offset++];

            if (recipeId != _outputRecipeId)
            {
                // 多 recipe 时才需要严格判断；这里只做提示，不中断。
                Log($"[RTDE] 收到 recipe id={recipeId}, 当前输出 recipe id={_outputRecipeId}");
            }

            for (int i = 0; i < _activeVariables.Count; i++)
            {
                string name = _activeVariables[i];
                string type = _activeTypes[i];

                object value = ReadValueByType(payload, ref offset, type);
                state.Values[name] = value;
            }

            state.LocalTime = DateTime.Now;
            return state;
        }

        private object ReadValueByType(byte[] buffer, ref int offset, string type)
        {
            switch (type)
            {
                case "BOOL":
                    return BigEndian.ReadBool(buffer, ref offset);

                case "UINT8":
                    return BigEndian.ReadUInt8(buffer, ref offset);

                case "INT32":
                    return BigEndian.ReadInt32(buffer, ref offset);

                case "UINT32":
                    return BigEndian.ReadUInt32(buffer, ref offset);

                case "UINT64":
                    return BigEndian.ReadUInt64(buffer, ref offset);

                case "DOUBLE":
                    return BigEndian.ReadDouble(buffer, ref offset);

                case "VECTOR3D":
                    return BigEndian.ReadDoubleArray(buffer, ref offset, 3);

                case "VECTOR6D":
                    return BigEndian.ReadDoubleArray(buffer, ref offset, 6);

                case "VECTOR6INT32":
                    return BigEndian.ReadInt32Array(buffer, ref offset, 6);

                case "VECTOR6UINT32":
                    return BigEndian.ReadUInt32Array(buffer, ref offset, 6);

                default:
                    throw new NotSupportedException("暂不支持 RTDE 数据类型：" + type);
            }
        }

        private async Task SendPackageAsync(byte command, byte[] payload, CancellationToken token)
        {
            if (_stream == null)
                throw new InvalidOperationException("RTDE 未连接。");

            int length = 3 + (payload?.Length ?? 0);

            byte[] packet = new byte[length];

            byte[] lengthBytes = BigEndian.GetBytes((ushort)length);

            packet[0] = lengthBytes[0];
            packet[1] = lengthBytes[1];
            packet[2] = command;

            if (payload != null && payload.Length > 0)
                Buffer.BlockCopy(payload, 0, packet, 3, payload.Length);

            await _stream.WriteAsync(packet, 0, packet.Length, token);
        }

        private async Task<RtdePacket> ReceiveUntilCommandAsync(byte expectedCommand, CancellationToken token)
        {
            while (true)
            {
                RtdePacket packet = await ReceivePackageAsync(token);

                if (packet.Command == expectedCommand)
                    return packet;

                if (packet.Command == RTDE_TEXT_MESSAGE)
                {
                    string msg = Encoding.ASCII.GetString(packet.Payload);
                    Log("[RTDE TEXT] " + msg);
                }
            }
        }

        private async Task<RtdePacket> ReceivePackageAsync(CancellationToken token)
        {
            byte[] headerLength = new byte[2];
            await ReadExactAsync(_stream, headerLength, 0, 2, token);

            ushort packageSize = BigEndian.ToUInt16(headerLength, 0);

            if (packageSize < 3)
                throw new InvalidDataException("RTDE package size 无效：" + packageSize);

            byte[] rest = new byte[packageSize - 2];
            await ReadExactAsync(_stream, rest, 0, rest.Length, token);

            byte command = rest[0];

            byte[] payload = new byte[rest.Length - 1];
            if (payload.Length > 0)
                Buffer.BlockCopy(rest, 1, payload, 0, payload.Length);

            return new RtdePacket
            {
                Command = command,
                Payload = payload
            };
        }

        private static async Task ReadExactAsync(
            Stream stream,
            byte[] buffer,
            int offset,
            int count,
            CancellationToken token)
        {
            while (count > 0)
            {
                int n = await stream.ReadAsync(buffer, offset, count, token);

                if (n == 0)
                    throw new IOException("RTDE socket 已断开。");

                offset += n;
                count -= n;
            }
        }

        private void Log(string text)
        {
            LogMessage?.Invoke(text);
        }

        public void Dispose()
        {
            try { _stream?.Dispose(); } catch { }
            try { _client?.Close(); } catch { }

            _stream = null;
            _client = null;
        }

        private class RtdePacket
        {
            public byte Command { get; set; }
            public byte[] Payload { get; set; }
        }

        private class SetupResult
        {
            public byte RecipeId { get; set; }
            public List<string> Variables { get; set; } = new List<string>();
            public List<string> Types { get; set; } = new List<string>();

            public bool HasNotFound
            {
                get
                {
                    return Types.Any(t => string.Equals(t, "NOT_FOUND", StringComparison.OrdinalIgnoreCase));
                }
            }
        }

        private static class BigEndian
        {
            public static byte[] GetBytes(ushort value)
            {
                byte[] b = BitConverter.GetBytes(value);
                if (BitConverter.IsLittleEndian) Array.Reverse(b);
                return b;
            }

            public static byte[] GetBytes(double value)
            {
                byte[] b = BitConverter.GetBytes(value);
                if (BitConverter.IsLittleEndian) Array.Reverse(b);
                return b;
            }

            public static ushort ToUInt16(byte[] buffer, int offset)
            {
                byte[] b = new byte[2];
                Buffer.BlockCopy(buffer, offset, b, 0, 2);
                if (BitConverter.IsLittleEndian) Array.Reverse(b);
                return BitConverter.ToUInt16(b, 0);
            }

            public static bool ReadBool(byte[] buffer, ref int offset)
            {
                byte v = ReadUInt8(buffer, ref offset);
                return v != 0;
            }

            public static byte ReadUInt8(byte[] buffer, ref int offset)
            {
                byte v = buffer[offset];
                offset += 1;
                return v;
            }

            public static int ReadInt32(byte[] buffer, ref int offset)
            {
                byte[] b = new byte[4];
                Buffer.BlockCopy(buffer, offset, b, 0, 4);
                if (BitConverter.IsLittleEndian) Array.Reverse(b);
                offset += 4;
                return BitConverter.ToInt32(b, 0);
            }

            public static uint ReadUInt32(byte[] buffer, ref int offset)
            {
                byte[] b = new byte[4];
                Buffer.BlockCopy(buffer, offset, b, 0, 4);
                if (BitConverter.IsLittleEndian) Array.Reverse(b);
                offset += 4;
                return BitConverter.ToUInt32(b, 0);
            }

            public static ulong ReadUInt64(byte[] buffer, ref int offset)
            {
                byte[] b = new byte[8];
                Buffer.BlockCopy(buffer, offset, b, 0, 8);
                if (BitConverter.IsLittleEndian) Array.Reverse(b);
                offset += 8;
                return BitConverter.ToUInt64(b, 0);
            }

            public static double ReadDouble(byte[] buffer, ref int offset)
            {
                byte[] b = new byte[8];
                Buffer.BlockCopy(buffer, offset, b, 0, 8);
                if (BitConverter.IsLittleEndian) Array.Reverse(b);
                offset += 8;
                return BitConverter.ToDouble(b, 0);
            }

            public static double[] ReadDoubleArray(byte[] buffer, ref int offset, int count)
            {
                double[] arr = new double[count];

                for (int i = 0; i < count; i++)
                    arr[i] = ReadDouble(buffer, ref offset);

                return arr;
            }

            public static int[] ReadInt32Array(byte[] buffer, ref int offset, int count)
            {
                int[] arr = new int[count];

                for (int i = 0; i < count; i++)
                    arr[i] = ReadInt32(buffer, ref offset);

                return arr;
            }

            public static uint[] ReadUInt32Array(byte[] buffer, ref int offset, int count)
            {
                uint[] arr = new uint[count];

                for (int i = 0; i < count; i++)
                    arr[i] = ReadUInt32(buffer, ref offset);

                return arr;
            }
        }
    }
}


