using System;
using System.Globalization;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;
using WpfRobot.inquiry;

namespace WpfRobot.command
{
    /// <summary>
    /// 统一运动控制调度器：
    /// 1. UI / Omega / AI 的运动命令统一进入这里；
    /// 2. 永远先更新 VTK 仿真；
    /// 3. 真机连接时，J1-J6 发给 UR；
    /// 4. 发送失败不回滚 VTK；
    /// 5. UR movej 下发后，通过 RTDE actual_q 等待到位；
    /// 6. StopAsync() 可以中止当前等待任务，并向 UR 发送 stopj。
    /// </summary>
    public class RobotMotionCoordinator
    {
        private readonly object _sendLock = new object();

        private readonly Dispatcher _uiDispatcher;


        /// <summary>
        /// 当前正在执行的运动任务取消源。
        /// StopAsync 会 Cancel 它，避免轨迹/等待到位继续执行。
        /// </summary>
        private CancellationTokenSource _activeMotionCts;

        private volatile bool _isMotionRunning = false;

        public event Action<string> LogMessage;

        /// <summary>
        /// 当前允许控制机器人的控制源。
        /// UI / Omega / AI 不能同时抢控制。
        /// Stop 命令不受 ActiveSource 限制。
        /// </summary>
        public MotionSourceType ActiveSource { get; set; } = MotionSourceType.UI;

        public bool IsMotionRunning
        {
            get { return _isMotionRunning; }
        }

        public RobotMotionCoordinator(Dispatcher uiDispatcher)
        {
            _uiDispatcher = uiDispatcher;
        }


        ///<summary>
        ///解决停止慢的问题
        ///<summary>
        private const double StopJAcceleration = 3.0;

        /// <summary>
        /// 统一执行运动命令。
        /// UI / Omega / AI 都调用这个函数。
        /// </summary>
        public async Task ExecuteAsync(RobotMotionCommand command, CancellationToken token = default)
        {
            if (command == null)
                return;

            if (command.Source != ActiveSource &&
                command.CommandType != MotionCommandType.Stop)
            {
                Log($"[MOTION] 当前控制权属于 {ActiveSource}，忽略 {command.Source} 指令。");
                return;
            }

            if (command.CommandType == MotionCommandType.Stop)
            {
                await StopAsync();
                return;
            }

            if (!command.IsValidJointTarget())
            {
                Log("[MOTION ERROR] 运动命令无效：JointDeg6 为空或长度不足 6。");
                return;
            }

            if (_isMotionRunning)
            {
                Log("[MOTION WARN] 当前已有运动正在执行，忽略新的运动指令。请先停止或等待到位。");
                return;
            }

            _activeMotionCts = CancellationTokenSource.CreateLinkedTokenSource(token);
            CancellationToken motionToken = _activeMotionCts.Token;

            _isMotionRunning = true;

            try
            {
                // 真机下发优先启动，不要被 VTK 刷新阻塞
                Task realTask = TrySendToRealRobotAsync(command, motionToken);

                // VTK/UI 低优先级刷新
                Task simTask = UpdateSimulationAsync(command);

                await Task.WhenAll(realTask, simTask);
            }
            catch (OperationCanceledException)
            {
                Log("[MOTION] 当前运动任务已取消。");
            }
            catch (Exception ex)
            {
                Log("[MOTION ERROR] 运动任务异常：" + ex.Message);
            }
            finally
            {
                _isMotionRunning = false;

                try
                {
                    _activeMotionCts?.Dispose();
                }
                catch
                {
                }

                _activeMotionCts = null;
            }
        }

        private async Task UpdateSimulationAsync(RobotMotionCommand command)
        {
            await _uiDispatcher.InvokeAsync(
                new Action(() =>
                {
                    // 只保存目标角，不强制切换“仿/实”模式
                    global_variable.SetGlobalJointDeg(command.JointDeg6);

                    // 当前是“仿”：VTK 用 UI 目标角
                    // 当前是“实”：VTK 用 RTDE actual_q
                    simulation_mode.RefreshSimulationByCurrentMode();
                }),
                DispatcherPriority.Background
            );

            if (simulation_mode._simulationDriveMode ==
                simulation_mode.SimulationDriveMode.TargetCommand)
            {
                Log($"[SIM] 当前为【仿】模式，{command.Source} 目标关节角已驱动 VTK。");
            }
            else
            {
                Log($"[SIM] 当前为【实】模式，VTK 仍由 RTDE actual_q 驱动，{command.Source} 目标角只用于下发命令。");
            }
        }

        private async Task TrySendToRealRobotAsync(RobotMotionCommand command, CancellationToken token)
        {
            bool urConnected = IsUrMotionSocketReady();
            if (!urConnected)
            {
                Log($"[REAL] 真机未连接，仅执行 VTK 仿真。来源={command.Source}");
                return;
            }

            Task urTask = Task.CompletedTask;

            if (urConnected)
            {
                urTask = SendUrMoveJAndWaitAsync(command, token);
            }
            else
            {
                Log("[REAL] UR 未连接，跳过 J1-J6 下发。");
            }


            try
            {
                await urTask;
                Log($"[REAL] 真机运动流程结束。来源={command.Source}");
            }
            catch (OperationCanceledException)
            {
                Log("[REAL] 真机运动流程已被取消。");
            }
            catch (Exception ex)
            {
                Log("[REAL ERROR] 真机运动命令执行失败：" + ex.Message);

                // 注意：失败不回滚 VTK。
                // 如果 UR socket 出错，只关闭 UR socket。
                try
                {
                    global_socket.CloseSocket();
                }
                catch
                {
                }
            }
        }

        private bool IsUrMotionSocketReady()
        {
            if (!global_socket.socketsuccess)
                return false;

            if (global_socket._socket == null)
                return false;

            if (global_socket._isClosingSocket)
                return false;

            if (!global_socket._socket.Connected)
                return false;

            return true;
        }

        /// <summary>
        /// 发送 UR movej，然后通过 RTDE actual_q 等待到位。
        /// </summary>
        private async Task SendUrMoveJAndWaitAsync(RobotMotionCommand command, CancellationToken token)
        {
            double[] targetDeg6 = command.GetUrJointDeg6();

            if (targetDeg6 == null || targetDeg6.Length < 6)
            {
                Log("[REAL ERROR] 目标关节角无效，无法等待到位。");
                return;
            }

            double[] targetRad6 = targetDeg6
                .Select(deg => deg * Math.PI / 180.0)
                .ToArray();

            LogMotionStateSnapshot("BEFORE_SEND", targetRad6);

            await SendUrMoveJAsync(command, token);

            LogMotionStateSnapshot("AFTER_SEND_0MS", targetRad6);

            await Task.Delay(100, token);
            LogMotionStateSnapshot("AFTER_SEND_100MS", targetRad6);

            await Task.Delay(500, token);
            LogMotionStateSnapshot("AFTER_SEND_600MS", targetRad6);

            if (!global_socket.HasFreshRtdeState(1500))
            {
                Log("[REAL WARN] movej 已发送，但 RTDE 状态未就绪，无法确认是否到位。");
                return;
            }

            double toleranceDeg = RobotParameterRuntime.ToleranceDeg;
            int timeoutMs = RobotParameterRuntime.TimeoutMsPerPoint;
            int pollMs = RobotParameterRuntime.PollIntervalMs;
            int stableCount = RobotParameterRuntime.StableCount;

            if (toleranceDeg <= 0.0)
                toleranceDeg = 0.5;

            if (timeoutMs <= 0)
                timeoutMs = 20000;

            if (pollMs <= 0)
                pollMs = 20;

            if (stableCount <= 0)
                stableCount = 3;

            double toleranceRad = toleranceDeg * Math.PI / 180.0;

            Log(
                $"[REAL] 开始等待 UR 到位：tol={toleranceDeg:F2}°, " +
                $"timeout={timeoutMs}ms, poll={pollMs}ms, stable={stableCount}"
            );

            bool reached = await WaitUntilUrReachedAsync(
                targetRad6,
                toleranceRad,
                timeoutMs,
                pollMs,
                stableCount,
                token
            );

            if (reached)
            {
                Log("[REAL] UR 已到达目标关节角。");
            }
            else
            {
                Log("[REAL WARN] UR 在超时时间内未到达目标关节角。");

                UrRtdeState state = global_socket.GetLatestRtdeState();

                if (state != null &&
                    state.ActualQ != null &&
                    state.ActualQ.Length >= 6)
                {
                    double maxErrDeg =
                        MaxAbsJointErrorRad(state.ActualQ, targetRad6) * 180.0 / Math.PI;

                    Log($"[REAL WARN] 当前最大关节误差约 {maxErrDeg:F2}°。");
                }

                LogMotionStateSnapshot("TIMEOUT", targetRad6);
            }
        }

        /// <summary>
        /// 下发 UR movej。
        /// RobotMotionCommand.JointDeg6 是角度制；
        /// 这里会把 J1-J6 转为弧度再发送给 UR。
        /// </summary>
        private async Task SendUrMoveJAsync( RobotMotionCommand command, CancellationToken token)
        {
            double[] qDeg6 = command.GetUrJointDeg6();

            if (qDeg6 == null || qDeg6.Length < 6)
                throw new InvalidOperationException("UR 6轴关节角无效。");

            double[] qRad6 = qDeg6
                .Select(deg => deg * Math.PI / 180.0)
                .ToArray();

            string qText = "[" + string.Join(", ", qRad6.Select(FormatUrDouble)) + "]";

            double a = command.MoveJAcceleration > 0.0 ? command.MoveJAcceleration : 0.5;
            double v = command.MoveJVelocity > 0.0 ? command.MoveJVelocity : 0.3;
            double t = command.MoveJTime >= 0.0 ? command.MoveJTime : 0.0;
            double r = command.MoveJBlendRadius >= 0.0 ? command.MoveJBlendRadius : 0.0;

            string script;

            // t=0 时不写 t 参数，避免时间参数覆盖 a/v 导致运动行为不符合预期
            if (t > 1e-6)
            {
                script =
                    $"movej({qText}, " +
                    $"a={FormatUrDouble(a)}, " +
                    $"v={FormatUrDouble(v)}, " +
                    $"t={FormatUrDouble(t)}, " +
                    $"r={FormatUrDouble(r)})\n";
            }
            else
            {
                script =
                    $"movej({qText}, " +
                    $"a={FormatUrDouble(a)}, " +
                    $"v={FormatUrDouble(v)}, " +
                    $"r={FormatUrDouble(r)})\n";
            }

            Log(
                "[UR DEBUG] qDeg6 = " +
                string.Join(", ", qDeg6.Select(x => x.ToString("F2", CultureInfo.InvariantCulture)))
            );

            Log(
                "[UR DEBUG] qRad6 = " +
                string.Join(", ", qRad6.Select(x => x.ToString("F6", CultureInfo.InvariantCulture)))
            );

            Log(
                $"[UR DEBUG] a={a:F3}, v={v:F3}, t={t:F3}, r={r:F3}"
            );

            byte[] data = Encoding.ASCII.GetBytes(script);

            await Task.Run(() =>
            {
                token.ThrowIfCancellationRequested();

                lock (_sendLock)
                {
                    if (!IsUrMotionSocketReady())
                        throw new InvalidOperationException("UR 运动 Socket 未连接。");

                    int sent = 0;

                    while (sent < data.Length)
                    {
                        int n = global_socket._socket.Send(
                            data,
                            sent,
                            data.Length - sent,
                            SocketFlags.None
                        );

                        if (n <= 0)
                            throw new SocketException();

                        sent += n;
                    }
                }
            }, token);

            Log("[UR CMD] " + script.Trim());
        }

        /// <summary>
        /// 等待 UR 实际关节角 actual_q 接近目标关节角。
        /// </summary>
        private async Task<bool> WaitUntilUrReachedAsync(double[] targetRad6, double toleranceRad,
        int timeoutMs, int pollIntervalMs, int stableCount, CancellationToken token)
        {
            DateTime start = DateTime.Now;
            int stableCounter = 0;
            int loopCounter = 0;

            while ((DateTime.Now - start).TotalMilliseconds < timeoutMs)
            {
                token.ThrowIfCancellationRequested();

                UrRtdeState state = global_socket.GetLatestRtdeState();

                if (state == null ||
                    state.ActualQ == null ||
                    state.ActualQ.Length < 6)
                {
                    await Task.Delay(pollIntervalMs, token);
                    continue;
                }

                double maxErrorRad = MaxAbsJointErrorRad(
                    state.ActualQ,
                    targetRad6
                );

                // 不要每 20ms 都刷日志，否则日志太多。
                // 大约每 500ms 打印一次最大误差。
                if (loopCounter % Math.Max(1, 500 / pollIntervalMs) == 0)
                {
                    double maxErrorDeg = maxErrorRad * 180.0 / Math.PI;

                    double actualQdMax = MaxAbsArray(state.ActualQd);
                    double targetQdMax = MaxAbsArray(state.TargetQd);

                    string runtime = UrRtdeText.RuntimeStateToText(state.RuntimeState);
                    string speedScaling = state.SpeedScaling.HasValue ? state.SpeedScaling.Value.ToString("F3") : "null";
                    string targetSpeedFraction = state.TargetSpeedFraction.HasValue ? state.TargetSpeedFraction.Value.ToString("F3") : "null";

                    double targetQErrDeg = -1.0;
                    if (state.TargetQ != null && state.TargetQ.Length >= 6)
                    {
                        targetQErrDeg = MaxAbsJointErrorRad(state.TargetQ, targetRad6) * 180.0 / Math.PI;
                    }

                    Log(
                        $"[REAL] 等待到位中，" +
                        $"actual_err={maxErrorDeg:F2}°, " +
                        $"target_q_err={targetQErrDeg:F2}°, " +
                        $"actual_qd_max={actualQdMax:F5}, " +
                        $"target_qd_max={targetQdMax:F5}, " +
                        $"speed_scaling={speedScaling}, " +
                        $"target_speed_fraction={targetSpeedFraction}, " +
                        $"runtime={runtime}"
                    );
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

                await Task.Delay(pollIntervalMs, token);
            }

            return false;
        }

        private double MaxAbsJointErrorRad(double[] actualRad6, double[] targetRad6)
        {
            if (actualRad6 == null || targetRad6 == null)
                return double.MaxValue;

            if (actualRad6.Length < 6 || targetRad6.Length < 6)
                return double.MaxValue;

            double maxError = 0.0;

            for (int i = 0; i < 6; i++)
            {
                double err = Math.Abs(NormalizeAngleRad(targetRad6[i] - actualRad6[i]));

                if (err > maxError)
                    maxError = err;
            }

            return maxError;
        }

        private double NormalizeAngleRad(double angle)
        {
            while (angle > Math.PI)
                angle -= 2.0 * Math.PI;

            while (angle < -Math.PI)
                angle += 2.0 * Math.PI;

            return angle;
        }

        /// <summary>
        /// 中止当前运动。
        /// 1. 取消当前等待到位任务；
        /// 2. UR 发送 stopj；
        
        /// </summary>
        public async Task StopAsync()
        {
            // 先取消等待到位任务，不要先做复杂 UI 日志
            try
            {
                _activeMotionCts?.Cancel();
            }
            catch
            {
            }

            Task urStop = Task.CompletedTask;

            if (IsUrMotionSocketReady())
            {
                urStop = SendRawUrScriptAsync(
                    $"stopj({FormatUrDouble(StopJAcceleration)})\n"
                );
            }


            try
            {
                await urStop;
                Log($"[MOTION] 停止命令已执行：stopj({StopJAcceleration:F1})");
            }
            catch (Exception ex)
            {
                Log("[MOTION ERROR] 停止命令执行失败：" + ex.Message);
            }
        }
        //public async Task StopAsync()
        //{
        //    Log("[MOTION] 收到停止命令。");

        //    try
        //    {
        //        _activeMotionCts?.Cancel();
        //    }
        //    catch
        //    {
        //    }

        //    Task urStop = Task.CompletedTask;
        //    Task seventhStop = Task.CompletedTask;

        //    if (IsUrMotionSocketReady())
        //    {
        //        urStop = SendRawUrScriptAsync("stopj(2.0)\n");
        //    }
        //    else
        //    {
        //        Log("[MOTION] UR Socket 未连接，跳过 UR stopj。");
        //    }

        //    if (_seventhAxisController != null &&
        //        _seventhAxisController.IsConnected)
        //    {
        //        seventhStop = _seventhAxisController.StopAsync();
        //    }
        //    else
        //    {
        //        Log("[MOTION] 第七轴未连接，跳过第七轴停止。");
        //    }

        //    try
        //    {
        //        await urStop;
        //        Log("[MOTION] 停止命令已执行。");
        //    }
        //    catch (Exception ex)
        //    {
        //        Log("[MOTION ERROR] 停止命令执行失败：" + ex.Message);
        //    }
        //}

        private async Task SendRawUrScriptAsync(string script)
        {
            if (string.IsNullOrWhiteSpace(script))
                return;

            if (!script.EndsWith("\n"))
                script += "\n";

            byte[] data = Encoding.ASCII.GetBytes(script);

            await Task.Run(() =>
            {
                lock (_sendLock)
                {
                    if (!IsUrMotionSocketReady())
                        return;

                    int sent = 0;

                    while (sent < data.Length)
                    {
                        int n = global_socket._socket.Send(
                            data,
                            sent,
                            data.Length - sent,
                            SocketFlags.None
                        );

                        if (n <= 0)
                            throw new SocketException();

                        sent += n;
                    }
                }
            });
        }

        private string FormatUrDouble(double value)
        {
            return value.ToString("G17", CultureInfo.InvariantCulture);
        }

        private void Log(string msg)
        {
            LogMessage?.Invoke(msg);
        }
        private void LogMotionStateSnapshot(string tag, double[] commandTargetRad6 = null)
        {
            UrRtdeState s = global_socket.GetLatestRtdeState();
            UrDashboardSnapshot d = global_socket.GetLatestDashboardSnapshot();

            if (s == null)
            {
                Log($"[DIAG {tag}] RTDE=null");
                return;
            }

            string robotMode = UrRtdeText.RobotModeToText(s.RobotMode);
            string safety = UrRtdeText.SafetyStatusToText(s.SafetyStatus);
            string runtime = UrRtdeText.RuntimeStateToText(s.RuntimeState);

            double actualErrDeg = -1.0;
            double targetErrDeg = -1.0;

            if (commandTargetRad6 != null && s.ActualQ != null && s.ActualQ.Length >= 6)
            {
                actualErrDeg = MaxAbsJointErrorRad(s.ActualQ, commandTargetRad6) * 180.0 / Math.PI;
            }

            if (commandTargetRad6 != null && s.TargetQ != null && s.TargetQ.Length >= 6)
            {
                targetErrDeg = MaxAbsJointErrorRad(s.TargetQ, commandTargetRad6) * 180.0 / Math.PI;
            }

            double actualQdMax = MaxAbsArray(s.ActualQd);
            double targetQdMax = MaxAbsArray(s.TargetQd);

            string speedScaling = s.SpeedScaling.HasValue ? s.SpeedScaling.Value.ToString("F3") : "null";
            string targetSpeedFraction = s.TargetSpeedFraction.HasValue ? s.TargetSpeedFraction.Value.ToString("F3") : "null";

            string dashRemote = d != null ? d.IsRemoteControl.ToString() : "null";
            string dashRunning = d != null ? d.IsProgramRunning.ToString() : "null";
            string dashProgram = d != null ? d.ProgramState : "null";

            Log(
                $"[DIAG {tag}] " +
                $"robot={robotMode}, safety={safety}, runtime={runtime}, " +
                $"speed_scaling={speedScaling}, target_speed_fraction={targetSpeedFraction}, " +
                $"actual_err={actualErrDeg:F2}°, target_q_err={targetErrDeg:F2}°, " +
                $"actual_qd_max={actualQdMax:F5}, target_qd_max={targetQdMax:F5}, " +
                $"remote={dashRemote}, dashRunning={dashRunning}, dashProgram={dashProgram}"
            );
        }

        private double MaxAbsArray(double[] values)
        {
            if (values == null || values.Length == 0)
                return 0.0;

            double max = 0.0;

            for (int i = 0; i < values.Length; i++)
            {
                double a = Math.Abs(values[i]);
                if (a > max)
                    max = a;
            }

            return max;
        }
    }
}
