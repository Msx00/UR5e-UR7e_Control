using System;
using System.Threading;
using NimServoSDK_DLL;

namespace WpfRobot.joint7
{
    public interface IMotor7Driver
    {
        bool IsConnected { get; }

        int NodeId { get; }

        void Connect();

        void Disconnect();

        double ReadEncoderAngleDeg();

        Motor7State ReadState();

        void MoveRelativeEncoderDeg(double deltaDeg);

        void MoveAbsoluteEncoderDeg(double targetDeg);

        void FastStop();

        void ClearEncoderHardwareZero();
    }

    public class Motor7State
    {
        public bool IsConnected { get; set; }

        public bool ServoReady { get; set; }

        public bool TargetReached { get; set; }

        public bool HasAlarm { get; set; }

        public ushort StatusWord { get; set; }

        public uint AlarmCode { get; set; }

        public double CurrentPositionDeg { get; set; }

        public double CurrentVelocityDegPerSec { get; set; }

        public int CurrentTorqueRaw { get; set; }

        public int WorkModeDisplay { get; set; }

        public int RetStatus { get; set; }

        public int RetAlarm { get; set; }

        public int RetPos { get; set; }

        public int RetVelocity { get; set; }

        public int RetTorque { get; set; }

        public int RetWorkMode { get; set; }

        public string Message { get; set; } = "";

        public override string ToString()
        {
            return
                $"Motor State: {(IsConnected ? "Connected" : "Disconnected")}\n" +
                $"Servo: {(ServoReady ? "Ready" : "--")}\n" +
                $"TargetReached: {(TargetReached ? "Yes" : "No")}\n" +
                $"Alarm: {(HasAlarm ? $"0x{AlarmCode:X8}" : "None")}\n" +
                $"StatusWord: 0x{StatusWord:X4}\n" +
                $"WorkModeDisplay: {WorkModeDisplay}\n" +
                $"Position: {CurrentPositionDeg:F3} deg\n" +
                $"Velocity: {CurrentVelocityDegPerSec:F3} deg/s\n" +
                $"TorqueRaw: {CurrentTorqueRaw}\n" +
                $"Ret: status={RetStatus}, pos={RetPos}, alarm={RetAlarm}, vel={RetVelocity}, torque={RetTorque}, mode={RetWorkMode}\n" +
                $"Message: {Message}";
        }
    }

    public class NimMotor7Driver : IMotor7Driver
    {
        private uint _hMaster = 0;
        private int _nodeId = 0;
        private bool _isConnected = false;
        private bool _masterCreated = false;

        private readonly object _sdkLock = new object();

        private readonly int _commType;
        private readonly string _commParam;
        private readonly double _unitFactor;
        private readonly double _profileVelocity;
        private readonly double _profileAccel;
        private readonly double _profileDecel;

        private const int TargetReachedMask = 0x400;

        public bool IsConnected => _isConnected;

        public int NodeId => _nodeId;

        private const double DegPerSdkUnit = 360;

        /// <summary>
        /// 方向系数。
        /// 如果发现 + 点动和 - 点动方向反了，把 1.0 改成 -1.0。
        /// </summary>
        private const double DirectionSign = 1.0;

        private static double DegToSdkUnit(double deg)
        {
            return DirectionSign * deg / DegPerSdkUnit;
        }

        private static double SdkUnitToDeg(double sdkUnit)
        {
            return DirectionSign * sdkUnit * DegPerSdkUnit;
        }

        /// <summary>
        /// commType:
        /// 0 = CANopen
        /// 1 = EtherCAT
        /// 2 = Modbus
        ///
        /// commParam:
        /// CANopen: "1001" / "1002" / "1004"
        /// EtherCAT: 网卡名称
        /// Modbus: 串口号，例如 "COM3"
        ///
        /// unitFactor:
        /// 用户单位换算系数。
        /// 如果 SDK 中 1.0 表示 1 deg，则这里应设置为 编码器脉冲数 / deg。
        /// </summary>
        public NimMotor7Driver(
            int commType,
            string commParam,
            double unitFactor = 10000.0,
            double profileVelocity = 3.0,
            double profileAccel = 5.0,
            double profileDecel = 5.0)
        {
            _commType = commType;
            _commParam = commParam;
            _unitFactor = unitFactor;
            _profileVelocity = profileVelocity;
            _profileAccel = profileAccel;
            _profileDecel = profileDecel;
        }

        public void Connect()
        {
            lock (_sdkLock)
            {
                if (_isConnected)
                    return;

                uint hMaster = 0;
                int nodeId = 0;
                bool masterStarted = false;

                try
                {
                    Environment.CurrentDirectory = AppDomain.CurrentDomain.BaseDirectory;

                    NimServoSDK.Nim_setLogFlags(1);

                    int retLog = NimServoSDK.Nim_getLogFlags();
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_getLogFlags ret={retLog}");

                    int nCommType = _commType;

                    int nRe = NimServoSDK.Nim_init("");
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_init ret={nRe}");
                    CheckRet(nRe, "Nim_init");

                    nRe = NimServoSDK.Nim_create_master(nCommType, ref hMaster);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_create_master ret={nRe}, hMaster={hMaster}");
                    CheckRet(nRe, "Nim_create_master");


                    if (nCommType < 0 || nCommType > 2)
                        throw new Exception($"不支持的通信方式 commType={nCommType}");

                    string connStr = BuildConnectionString(nCommType, _commParam);

                    System.Diagnostics.Debug.WriteLine($"[NIM] BaseDirectory={AppDomain.CurrentDomain.BaseDirectory}");
                    System.Diagnostics.Debug.WriteLine($"[NIM] CurrentDirectory={Environment.CurrentDirectory}");
                    System.Diagnostics.Debug.WriteLine($"[NIM] commType={nCommType}");
                    System.Diagnostics.Debug.WriteLine($"[NIM] commParam={_commParam}");
                    System.Diagnostics.Debug.WriteLine($"[NIM] connStr={connStr}");

                    nRe = NimServoSDK.Nim_master_run(hMaster, connStr);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_master_run ret={nRe}");
                    CheckRet(nRe, "Nim_master_run");

                    masterStarted = true;

                    nRe = NimServoSDK.Nim_master_changeToPreOP(hMaster);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_master_changeToPreOP ret={nRe}");
                    CheckRet(nRe, "Nim_master_changeToPreOP");

                    Thread.Sleep(50);

                    nRe = NimServoSDK.Nim_scan_nodes(hMaster, 1, 10);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_scan_nodes ret={nRe}");
                    CheckRet(nRe, "Nim_scan_nodes");

                    for (int i = 0; i < 10; i++)
                    {
                        int online = NimServoSDK.Nim_is_online(hMaster, i);
                        System.Diagnostics.Debug.WriteLine($"[NIM] Nim_is_online node={i}, online={online}");

                        if (online != 0)
                        {
                            nodeId = i;
                            System.Diagnostics.Debug.WriteLine($"[NIM] motor {i} is online");
                            break;
                        }
                    }

                    if (nodeId == 0)
                        throw new Exception("There is no motor online");

                    double fUnitFactor = _unitFactor;
                    System.Diagnostics.Debug.WriteLine($"[NIM] UnitFactor={fUnitFactor}");

                    string dbName = GetDbName(nCommType);
                    string dbPath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, dbName);

                    System.Diagnostics.Debug.WriteLine($"[NIM] DbName={dbName}");
                    System.Diagnostics.Debug.WriteLine($"[NIM] DbPath={dbPath}");
                    System.Diagnostics.Debug.WriteLine($"[NIM] DbExists={System.IO.File.Exists(dbPath)}");

                    if (!System.IO.File.Exists(dbPath))
                        throw new Exception($"找不到参数数据库文件：{dbPath}");

                    nRe = NimServoSDK.Nim_load_params(hMaster, nodeId, dbName);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_load_params ret={nRe}");
                    CheckRet(nRe, "Nim_load_params");

                    nRe = NimServoSDK.Nim_read_PDOConfig(hMaster, nodeId);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_read_PDOConfig ret={nRe}, NodeId={nodeId}");
                    CheckRet(nRe, "Nim_read_PDOConfig");

                    nRe = NimServoSDK.Nim_master_changeToOP(hMaster);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_master_changeToOP ret={nRe}");
                    CheckRet(nRe, "Nim_master_changeToOP");

                    Thread.Sleep(50);

                    nRe = NimServoSDK.Nim_set_unitsFactor(hMaster, nodeId, fUnitFactor);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_set_unitsFactor ret={nRe}, value={fUnitFactor}");
                    CheckRet(nRe, "Nim_set_unitsFactor");

                    nRe = NimServoSDK.Nim_clearError(hMaster, nodeId, 1);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_clearError ret={nRe}");
                    CheckRet(nRe, "Nim_clearError");

                    nRe = NimServoSDK.Nim_power_off(hMaster, nodeId, 1);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_power_off ret={nRe}");
                    CheckRet(nRe, "Nim_power_off");

                    Thread.Sleep(50);

                    nRe = NimServoSDK.Nim_set_workMode(hMaster, nodeId, 1, 1);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_set_workMode ret={nRe}");
                    CheckRet(nRe, "Nim_set_workMode");

                    Thread.Sleep(50);

                    double sdkVelocity = DegToSdkUnit(_profileVelocity);
                    double sdkAccel = DegToSdkUnit(_profileAccel);
                    double sdkDecel = DegToSdkUnit(_profileDecel);

                    nRe = NimServoSDK.Nim_set_profileVelocity(hMaster, nodeId, Math.Abs(sdkVelocity));
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_set_profileVelocity ret={nRe}, deg/s={_profileVelocity}, sdk={Math.Abs(sdkVelocity)}");
                    CheckRet(nRe, "Nim_set_profileVelocity");

                    nRe = NimServoSDK.Nim_set_profileAccel(hMaster, nodeId, Math.Abs(sdkAccel));
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_set_profileAccel ret={nRe}, deg/s^2={_profileAccel}, sdk={Math.Abs(sdkAccel)}");
                    CheckRet(nRe, "Nim_set_profileAccel");

                    nRe = NimServoSDK.Nim_set_profileDecel(hMaster, nodeId, Math.Abs(sdkDecel));
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_set_profileDecel ret={nRe}, deg/s^2={_profileDecel}, sdk={Math.Abs(sdkDecel)}");
                    CheckRet(nRe, "Nim_set_profileDecel");

                    nRe = NimServoSDK.Nim_power_on(hMaster, nodeId, 1);
                    System.Diagnostics.Debug.WriteLine($"[NIM] Nim_power_on ret={nRe}");
                    CheckRet(nRe, "Nim_power_on");

                    Thread.Sleep(200);

                    ushort statusWord = 0;
                    double currentPos = 0.0;

                    int retStatus = NimServoSDK.Nim_get_statusWord(hMaster, nodeId, ref statusWord, 1);
                    int retPos = NimServoSDK.Nim_get_currentPosition(hMaster, nodeId, ref currentPos, 1);

                    System.Diagnostics.Debug.WriteLine(
                        $"[NIM] After power_on: retStatus={retStatus}, statusWord={statusWord}, hex=0x{statusWord:X4}, retPos={retPos}, currentPos={currentPos}");

                    CheckRet(retStatus, "Nim_get_statusWord");
                    CheckRet(retPos, "Nim_get_currentPosition");

                    // 关键：所有步骤成功后，最后统一提交到成员变量
                    _hMaster = hMaster;
                    _nodeId = nodeId;
                    _isConnected = true;
                    _masterCreated = true;

                    System.Diagnostics.Debug.WriteLine(
                        $"[NIM] Motor7 connected successfully. _hMaster={_hMaster}, _nodeId={_nodeId}, _isConnected={_isConnected}");
                }
                catch
                {
                    try
                    {
                        if (hMaster != 0 && nodeId > 0)
                        {
                            try { NimServoSDK.Nim_fastStop(hMaster, nodeId, 1); } catch { }
                            try { NimServoSDK.Nim_power_off(hMaster, nodeId, 1); Thread.Sleep(50); } catch { }
                        }

                        if (hMaster != 0)
                        {
                            try { NimServoSDK.Nim_master_changeToPreOP(hMaster); Thread.Sleep(50); } catch { }

                            if (masterStarted)
                            {
                                try { NimServoSDK.Nim_master_stop(hMaster); } catch { }
                            }

                            try { NimServoSDK.Nim_destroy_master(hMaster); } catch { }
                        }

                        try { NimServoSDK.Nim_clean(); } catch { }
                    }
                    finally
                    {
                        _hMaster = 0;
                        _nodeId = 0;
                        _isConnected = false;
                    }

                    throw;
                }
            }
        }
        public void Disconnect()
        {
            lock (_sdkLock)
            {
                SafeCloseCore();
            }
        }

        public double ReadEncoderAngleDeg()
        {
            lock (_sdkLock)
            {
                CheckConnectedNoLock();
                return ReadEncoderAngleDegNoLock();
            }
        }

        public Motor7State ReadState()
        {
            lock (_sdkLock)
            {
                if (!_isConnected || !_masterCreated || _nodeId <= 0)
                {
                    return new Motor7State
                    {
                        IsConnected = false,
                        ServoReady = false,
                        HasAlarm = false,
                        Message = $"Disconnected. _isConnected={_isConnected}, _masterCreated={_masterCreated}, _nodeId={_nodeId}"
                    };
                }

                ushort statusWord = 0;
                uint alarmCode = 0;
                double sdkPos = 0.0;
                double sdkVelocity = 0.0;
                int torqueRaw = 0;
                int workModeDisplay = 0;

                int retStatus = NimServoSDK.Nim_get_statusWord(_hMaster, _nodeId, ref statusWord, 1);
                int retAlarm = NimServoSDK.Nim_get_newestAlarm(_hMaster, _nodeId, ref alarmCode, 1);
                int retPos = NimServoSDK.Nim_get_currentPosition(_hMaster, _nodeId, ref sdkPos, 1);
                int retVelocity = NimServoSDK.Nim_get_currentVelocity(_hMaster, _nodeId, ref sdkVelocity, 1);
                int retTorque = NimServoSDK.Nim_get_currentTorque(_hMaster, _nodeId, ref torqueRaw, 1);
                int retWorkMode = NimServoSDK.Nim_get_workModeDisplay(_hMaster, _nodeId, ref workModeDisplay, 1);

                bool hasAlarm = retAlarm == 0 && alarmCode != 0;
                bool targetReached = retStatus == 0 && (statusWord & TargetReachedMask) != 0;

                return new Motor7State
                {
                    IsConnected = true,

                    // 这里只代表成功读到了状态字，不等价于“已使能”
                    ServoReady = retStatus == 0,

                    TargetReached = targetReached,
                    HasAlarm = hasAlarm,

                    StatusWord = statusWord,
                    AlarmCode = alarmCode,

                    CurrentPositionDeg = retPos == 0 ? SdkUnitToDeg(sdkPos) : 0.0,
                    CurrentVelocityDegPerSec = retVelocity == 0 ? SdkUnitToDeg(sdkVelocity) : 0.0,
                    CurrentTorqueRaw = retTorque == 0 ? torqueRaw : 0,
                    WorkModeDisplay = retWorkMode == 0 ? workModeDisplay : 0,

                    RetStatus = retStatus,
                    RetAlarm = retAlarm,
                    RetPos = retPos,
                    RetVelocity = retVelocity,
                    RetTorque = retTorque,
                    RetWorkMode = retWorkMode,

                    Message =
                        $"NodeId={_nodeId}, " +
                        $"retStatus={retStatus}, retPos={retPos}, retAlarm={retAlarm}, " +
                        $"retVelocity={retVelocity}, retTorque={retTorque}, retWorkMode={retWorkMode}"
                };
            }
        }

        public void MoveRelativeEncoderDeg(double deltaDeg)
        {
            lock (_sdkLock)
            {
                CheckConnectedNoLock();

                if (Math.Abs(deltaDeg) < 1e-9)
                    return;

                double currentDeg = ReadEncoderAngleDegNoLock();
                double sdkDelta = DegToSdkUnit(deltaDeg);

                System.Diagnostics.Debug.WriteLine(
                    $"[NIM MOVE] Relative start. currentDeg={currentDeg:F3}, deltaDeg={deltaDeg:F3}, sdkDelta={sdkDelta:F6}");

                int ret = NimServoSDK.Nim_moveRelative(_hMaster, _nodeId, sdkDelta, 0, 1);//电机转动
                System.Diagnostics.Debug.WriteLine($"[NIM MOVE] Nim_moveRelative ret={ret}");
                CheckRet(ret, "Nim_moveRelative");

                Thread.Sleep(50);

                int timeoutMs = CalcMoveTimeoutMs(Math.Abs(deltaDeg));
                WaitUntilTargetReachedNoLock(timeoutMs);

                double finalDeg = ReadEncoderAngleDegNoLock();

                System.Diagnostics.Debug.WriteLine(
                    $"[NIM MOVE] Relative done. finalDeg={finalDeg:F3}");
            }
        }

        public void MoveRelativeEncoderDeg_old(double deltaDeg)
        {
            lock (_sdkLock)
            {
                CheckConnectedNoLock();

                if (Math.Abs(deltaDeg) < 1e-9)
                    return;

                double currentPos = ReadEncoderAngleDegNoLock();

                System.Diagnostics.Debug.WriteLine(
                    $"[NIM MOVE] Relative start. current={currentPos:F3}, delta={deltaDeg:F3}");

                int ret = NimServoSDK.Nim_moveRelative(_hMaster, _nodeId, deltaDeg, 0, 1);
                System.Diagnostics.Debug.WriteLine($"[NIM MOVE] Nim_moveRelative ret={ret}");
                CheckRet(ret, "Nim_moveRelative");

                Thread.Sleep(50);

                int timeoutMs = CalcMoveTimeoutMs(Math.Abs(deltaDeg));
                WaitUntilTargetReachedNoLock(timeoutMs);

                double finalPos = ReadEncoderAngleDegNoLock();

                System.Diagnostics.Debug.WriteLine(
                    $"[NIM MOVE] Relative done. final={finalPos:F3}");
            }
        }

        public void MoveAbsoluteEncoderDeg(double targetDeg)
        {
            lock (_sdkLock)
            {
                CheckConnectedNoLock();

                double currentDeg = ReadEncoderAngleDegNoLock();
                double distanceDeg = targetDeg - currentDeg;

                if (Math.Abs(distanceDeg) < 1e-6)
                    return;

                double sdkTarget = DegToSdkUnit(targetDeg);

                System.Diagnostics.Debug.WriteLine(
                    $"[NIM MOVE] Absolute start. currentDeg={currentDeg:F3}, targetDeg={targetDeg:F3}, distanceDeg={distanceDeg:F3}, sdkTarget={sdkTarget:F6}");

                int ret = NimServoSDK.Nim_moveAbsolute(_hMaster, _nodeId, sdkTarget, 0, 1);
                System.Diagnostics.Debug.WriteLine($"[NIM MOVE] Nim_moveAbsolute ret={ret}");
                CheckRet(ret, "Nim_moveAbsolute");

                Thread.Sleep(50);

                int timeoutMs = CalcMoveTimeoutMs(Math.Abs(distanceDeg));
                WaitUntilTargetReachedNoLock(timeoutMs);

                double finalDeg = ReadEncoderAngleDegNoLock();

                System.Diagnostics.Debug.WriteLine(
                    $"[NIM MOVE] Absolute done. finalDeg={finalDeg:F3}");
            }
        }

        public void MoveAbsoluteEncoderDeg_old(double targetDeg)
        {
            lock (_sdkLock)
            {
                CheckConnectedNoLock();

                double currentPos = ReadEncoderAngleDegNoLock();
                double distance = targetDeg - currentPos;

                if (Math.Abs(distance) < 1e-6)
                    return;

                System.Diagnostics.Debug.WriteLine(
                    $"[NIM MOVE] Absolute start. current={currentPos:F3}, target={targetDeg:F3}, distance={distance:F3}");

                int ret = NimServoSDK.Nim_moveAbsolute(_hMaster, _nodeId, targetDeg, 0, 1);
                System.Diagnostics.Debug.WriteLine($"[NIM MOVE] Nim_moveAbsolute ret={ret}");
                CheckRet(ret, "Nim_moveAbsolute");

                Thread.Sleep(50);

                int timeoutMs = CalcMoveTimeoutMs(Math.Abs(distance));
                WaitUntilTargetReachedNoLock(timeoutMs);

                double finalPos = ReadEncoderAngleDegNoLock();

                System.Diagnostics.Debug.WriteLine(
                    $"[NIM MOVE] Absolute done. final={finalPos:F3}");
            }
        }

        public void FastStop()
        {
            lock (_sdkLock)
            {
                if (_masterCreated && _nodeId > 0)
                {
                    NimServoSDK.Nim_fastStop(_hMaster, _nodeId, 1);
                }
            }
        }

        private double ReadEncoderAngleDegNoLock()
        {
            double sdkPos = 0.0;

            CheckRet(
                NimServoSDK.Nim_get_currentPosition(_hMaster, _nodeId, ref sdkPos, 1),
                "Nim_get_currentPosition");

            return SdkUnitToDeg(sdkPos);
        }


        private void WaitUntilTargetReachedNoLock(int timeoutMs)
        {
            DateTime start = DateTime.Now;

            ushort statusWord = 0;
            double pos = 0.0;
            int continuousReadFailCount = 0;

            while (true)
            {
                Thread.Sleep(50);

                int retPos = NimServoSDK.Nim_get_currentPosition(_hMaster, _nodeId, ref pos, 1);
                int retStatus = NimServoSDK.Nim_get_statusWord(_hMaster, _nodeId, ref statusWord, 1);

                if (retPos == 0 && retStatus == 0)
                {
                    continuousReadFailCount = 0;

                    double posDeg = SdkUnitToDeg(pos);

                    System.Diagnostics.Debug.WriteLine(
                        $"[NIM MOVE] statusWord=0x{statusWord:X4}, currentDeg={posDeg:F3}, sdkPos={pos:F6}");

                    if ((statusWord & TargetReachedMask) != 0)
                        return;
                }
                else
                {
                    continuousReadFailCount++;

                    System.Diagnostics.Debug.WriteLine(
                        $"[NIM WARN] 等待到位读取失败：retPos={retPos}, retStatus={retStatus}, failCount={continuousReadFailCount}");

                    if (continuousReadFailCount >= 10)
                    {
                        throw new Exception(
                            $"等待第七电机到位时连续读取失败。retPos={retPos}, retStatus={retStatus}");
                    }
                }

                if ((DateTime.Now - start).TotalMilliseconds > timeoutMs)
                {
                    throw new TimeoutException(
                        $"第七电机运动超时。CurrentPos={pos:F3}, StatusWord=0x{statusWord:X4}, Timeout={timeoutMs}ms");
                }
            }
        }

        private void WaitUntilTargetReachedNoLock_old(int timeoutMs)
        {
            DateTime start = DateTime.Now;

            ushort statusWord = 0;
            double pos = 0.0;
            int continuousReadFailCount = 0;

            while (true)
            {
                Thread.Sleep(50);

                int retPos = NimServoSDK.Nim_get_currentPosition(_hMaster, _nodeId, ref pos, 1);
                int retStatus = NimServoSDK.Nim_get_statusWord(_hMaster, _nodeId, ref statusWord, 1);

                if (retPos == 0 && retStatus == 0)
                {
                    continuousReadFailCount = 0;

                    System.Diagnostics.Debug.WriteLine(
                        $"[NIM MOVE] statusWord=0x{statusWord:X4}, currentPos={pos:F3}");

                    if ((statusWord & TargetReachedMask) != 0)
                        return;
                }
                else
                {
                    continuousReadFailCount++;

                    System.Diagnostics.Debug.WriteLine(
                        $"[NIM WARN] 等待到位读取失败：retPos={retPos}, retStatus={retStatus}, failCount={continuousReadFailCount}");

                    if (continuousReadFailCount >= 10)
                    {
                        throw new Exception(
                            $"等待第七电机到位时连续读取失败。retPos={retPos}, retStatus={retStatus}");
                    }
                }

                if ((DateTime.Now - start).TotalMilliseconds > timeoutMs)
                {
                    throw new TimeoutException(
                        $"第七电机运动超时。CurrentPos={pos:F3}, StatusWord=0x{statusWord:X4}, Timeout={timeoutMs}ms");
                }
            }
        }

        private int CalcMoveTimeoutMs(double distanceDeg)
        {
            double velocity = Math.Abs(_profileVelocity);

            if (velocity < 1e-6)
                velocity = 3.0;

            double seconds = distanceDeg / velocity;

            int timeoutMs = (int)(seconds * 1000.0 + 8000.0);

            if (timeoutMs < 10000)
                timeoutMs = 10000;

            if (timeoutMs > 120000)
                timeoutMs = 120000;

            return timeoutMs;
        }

        private int ScanFirstOnlineNode(int from, int to)
        {
            CheckRet(NimServoSDK.Nim_scan_nodes(_hMaster, from, to), "Nim_scan_nodes");

            for (int node = from; node <= to; node++)
            {
                if (NimServoSDK.Nim_is_online(_hMaster, node) != 0)
                    return node;
            }

            return 0;
        }

        private static string BuildConnectionString(int commType, string commParam)
        {
            switch (commType)
            {
                case 0:
                    return BuildCanOpenConnectionString(commParam);

                case 1:
                    return
                        "{\"NetworkAdapter\": \"" + commParam + "\", " +
                        "\"OverlappingPDO\": true, \"PDOIntervalMS\": 5}";

                case 2:
                    return
                        "{\"SerialPort\": \"" + commParam + "\", \"Baudrate\": 115200," +
                        " \"Parity\": \"N\", \"DataBits\": 8, \"StopBits\": 1," +
                        " \"PDOIntervalMS\": 20, \"SyncIntervalMS\": 0}";

                default:
                    throw new ArgumentException("不支持的通信方式 commType=" + commType);
            }
        }

        private static string BuildCanOpenConnectionString(string devTypeText)
        {
            int devType = int.Parse(devTypeText);

            switch (devType)
            {
                case 1001:
                    return
                        "{\"DevType\": \"1001\", \"DevIndex\": 0, \"Baudrate\": 8,"
                        + "\"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";

                case 1002:
                    return
                        "{\"DevType\": \"1002\", \"DevSubType\": 3, \"DevIndex\": 0, \"ChannelIndex\": 0,"
                        + " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";

                case 1003:
                    return
                        "{\"DevType\": \"%s\", \"DevIndex\": 0, \"Baudrate\": 8,"
                        + " \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";

                case 1004:
                    return
                        "{\"DevType\": \"1004\", \"IP\": \"192.168.0.96\", \"Port\": 40001,"
                        + " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";

                case 1005:
                    return
                        "{\"DevType\": \"%s\", \"DeviceName\": \"can0\","
                        + " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";

                default:
                    return
                        "{\"DevType\": \"1001\", \"DevIndex\": 0, \"Baudrate\": 8,"
                        + "\"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";
            }
        }

        private static string GetDbName(int commType)
        {
            switch (commType)
            {
                case 0:
                    return "CANopen.db";
                case 1:
                    return "EtherCAT.db";
                case 2:
                    return "Modbus.db";
                default:
                    throw new ArgumentException("不支持的通信方式 commType=" + commType);
            }
        }

        private void CheckConnectedNoLock()
        {
            if (!_isConnected || !_masterCreated || _nodeId <= 0)
                throw new InvalidOperationException("第七电机尚未连接。");
        }

        private static void CheckRet(int ret, string apiName)
        {
            System.Diagnostics.Debug.WriteLine($"[NIM RET] {apiName} ret={ret}");

            if (ret != 0)
                throw new Exception($"{apiName} 调用失败，ret={ret}");
        }

        private void SafeCloseCore()
        {
            try
            {
                if (_hMaster != 0 && _nodeId > 0)
                {
                    try
                    {
                        NimServoSDK.Nim_fastStop(_hMaster, _nodeId, 1);
                    }
                    catch { }

                    try
                    {
                        NimServoSDK.Nim_power_off(_hMaster, _nodeId, 1);
                        Thread.Sleep(50);
                    }
                    catch { }
                }

                if (_masterCreated)
                {
                    try
                    {
                        NimServoSDK.Nim_master_changeToPreOP(_hMaster);
                        Thread.Sleep(50);
                    }
                    catch { }

                    try
                    {
                        NimServoSDK.Nim_master_stop(_hMaster);
                    }
                    catch { }

                    try
                    {
                        NimServoSDK.Nim_destroy_master(_hMaster);
                    }
                    catch { }
                }
            }
            finally
            {
                try
                {
                    NimServoSDK.Nim_clean();
                }
                catch { }

                _hMaster = 0;
                _nodeId = 0;
                _isConnected = false;
            }
        }

       
        private void SetParamUIntNoLock(string paramNo, uint value, string apiName)
        {
            int ret = NimServoSDK.Nim_set_param_value(
                _hMaster,
                _nodeId,
                paramNo,
                value,
                1);

            System.Diagnostics.Debug.WriteLine(
                $"[NIM PARAM] {apiName}, param={paramNo}, value={value}, ret={ret}");

            CheckRet(ret, apiName);
        }

        public void ClearEncoderHardwareZero()
        {
            lock (_sdkLock)
            {
                CheckConnectedNoLock();

                // 清零前建议先停稳、脱机，避免坐标突变后伺服追目标
                try
                {
                    NimServoSDK.Nim_fastStop(_hMaster, _nodeId, 1);
                    Thread.Sleep(100);
                }
                catch
                {
                }

                CheckRet(
                    NimServoSDK.Nim_power_off(_hMaster, _nodeId, 1),
                    "Nim_power_off");

                Thread.Sleep(100);

                // 配置虚拟端子 VDI1 功能为“设置零点”
                SetParamUIntNoLock("H2017-01", 33, "H2017-01 设置零点功能");

                // 按 NiMotion 文档：H2017-02 = 0
                SetParamUIntNoLock("H2017-02", 0, "H2017-02 设置有效电平");

                // 保存参数。保存后以后再次清零只需要触发 VDI1。
                CheckRet(
                    NimServoSDK.Nim_save_AllParams(_hMaster, _nodeId, 5000),
                    "Nim_save_AllParams");

                Thread.Sleep(300);

                // 触发虚拟端子 VDI1：先写 1，再写 0
                CheckRet(
                    NimServoSDK.Nim_set_VDIs(_hMaster, _nodeId, 1),
                    "Nim_set_VDIs 1");

                Thread.Sleep(100);

                CheckRet(
                    NimServoSDK.Nim_set_VDIs(_hMaster, _nodeId, 0),
                    "Nim_set_VDIs 0");

                Thread.Sleep(300);

                // 重新使能
                CheckRet(
                    NimServoSDK.Nim_power_on(_hMaster, _nodeId, 1),
                    "Nim_power_on");
            }
        }
    }
}