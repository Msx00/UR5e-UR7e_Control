using System;

namespace WpfRobot
{
    /// <summary>
    /// 机器人运行时参数中心。
    /// 
    /// 设计原则：
    /// 1. Properties.Settings.Default 只在这里读写；
    /// 2. MainWindow / robParameter / RobotMotionCoordinator 都只使用 RobotParameterRuntime；
    /// 3. 程序启动时调用 LoadFromSettings()；
    /// 4. 参数窗口点击保存时调用 SaveToSettings()；
    /// 5. 单次运动发送时从 RobotParameterRuntime 读取当前参数。
    /// </summary>
    public static class RobotParameterRuntime
    {
        // =========================================================
        // 1. MoveJ 参数
        // =========================================================
        public static double MoveJVelocity = 0.30;
        public static double MoveJAcceleration = 0.50;
        public static double MoveJTime = 0.00;
        public static double MoveJBlendRadius = 0.00;

        // =========================================================
        // 2. MoveL 参数
        // 目前如果你还没有放入 Settings，可以先保留默认值
        // =========================================================
        public static double MoveLVelocity = 0.25;
        public static double MoveLAcceleration = 1.20;

        // =========================================================
        // 3. SpeedJ / ServoJ 参数
        // =========================================================
        public static double SpeedJDuration = 0.50;
        public static double ServoJLookaheadTime = 0.10;
        public static double ServoJGain = 300.0;

        // =========================================================
        // 4. 等待到位参数
        // =========================================================
        public static double ToleranceDeg = 0.50;
        public static int TimeoutMsPerPoint = 20000;
        public static int PollIntervalMs = 20;
        public static int StableCount = 3;

        // =========================================================
        // 5. RCM 参数
        // =========================================================
        public static double RcmX = -366.7;
        public static double RcmY = -741.9;
        public static double RcmZ = 313.9;
        public static bool RcmEnabled = true;

        // =========================================================
        // 6. Payload 参数
        // =========================================================
        public static double PayloadMass = 2.0;
        public static double PayloadCogX = 0.0;
        public static double PayloadCogY = 0.0;
        public static double PayloadCogZ = 50.0;

        /// <summary>
        /// 程序启动时，从 Properties.Settings.Default 加载保存的参数。
        /// </summary>
        public static void LoadFromSettings()
        {
            // MoveJ 参数从 Settings 读取
            MoveJVelocity = SafePositive(
                Properties.Settings.Default.movej_speed,
                fallback: 0.30,
                min: 0.01,
                max: 1.5
            );

            MoveJAcceleration = SafePositive(
                Properties.Settings.Default.movej_acc,
                fallback: 0.50,
                min: 0.01,
                max: 2.00
            );

            MoveJTime = SafeNonNegative(
                Properties.Settings.Default.movej_time,
                fallback: 0.00,
                min: 0.00,
                max: 120.00
            );

            MoveJBlendRadius = SafeNonNegative(
                Properties.Settings.Default.movej_blend,
                fallback: 0.00,
                min: 0.00,
                max: 0.10
            );

            MoveLVelocity = SafeNonNegative(
                Properties.Settings.Default.moveL_speed,
                fallback: 0.00,
                min: 0.01,
                max: 1.5
            );

            MoveLAcceleration = SafeNonNegative(
                Properties.Settings.Default.moveL_acc,
                fallback: 0.00,
                min: 0.01,
                max: 2.0
            );

            SpeedJDuration = SafeNonNegative(
                Properties.Settings.Default.speedj_t,
                fallback: 0.00,
                min: 0.01,
                max: 1.5
            );

            ServoJLookaheadTime = SafeNonNegative(
                Properties.Settings.Default.servoj_lookahead,
                fallback: 0.00,
                min: 0.01,
                max: 1.0
            );

            ServoJGain = SafeNonNegative(
                Properties.Settings.Default.servoj_gain,
                fallback: 200,
                min: 1,
                max: 1000
            );

            ToleranceDeg = SafeNonNegative(
                Properties.Settings.Default.toleranceDeg,
                fallback: 0.1,
                min: 0.001,
                max: 0.5
            );

            TimeoutMsPerPoint = (int)SafeNonNegative(
                Properties.Settings.Default.timeoutMsPerPoint,
                fallback: 10000,
                min: 1000,
                max: 60000
            );

            PollIntervalMs = (int)SafeNonNegative(
                Properties.Settings.Default.pollintervalMs,
                fallback: 20,
                min: 10,
                max: 1000
            );

            StableCount = (int)SafeNonNegative(
                Properties.Settings.Default.stableCount,
                fallback: 3,
                min: 1,
                max: 10
            );
        }

        /// <summary>
        /// 点击“保存参数”时，把当前运行时参数写入 Properties.Settings.Default。
        /// </summary>
        public static void SaveToSettings()
        {
            Properties.Settings.Default.movej_speed = MoveJVelocity;
            Properties.Settings.Default.movej_acc = MoveJAcceleration;
            Properties.Settings.Default.movej_time = MoveJTime;
            Properties.Settings.Default.movej_blend = MoveJBlendRadius;

            Properties.Settings.Default.moveL_speed = MoveLVelocity;
            Properties.Settings.Default.moveL_acc = MoveLAcceleration;

            // 如果你已经在 Settings.settings 里新增了这两个字段
            Properties.Settings.Default.speedj_t = SpeedJDuration;
            Properties.Settings.Default.servoj_lookahead = ServoJLookaheadTime;

            Properties.Settings.Default.servoj_gain = ServoJGain;

            Properties.Settings.Default.toleranceDeg = ToleranceDeg;
            Properties.Settings.Default.timeoutMsPerPoint = TimeoutMsPerPoint;
            Properties.Settings.Default.pollintervalMs = PollIntervalMs;
            Properties.Settings.Default.stableCount = StableCount;

            Properties.Settings.Default.Save();
        }

        /// <summary>
        /// 恢复软件内置默认参数。
        /// 注意：这里只恢复运行时参数，不自动写入 Settings。
        /// 如果希望恢复后也保存，需要再调用 SaveToSettings()。
        /// </summary>
        public static void ResetFactoryDefault()
        {
            MoveJVelocity = 0.30;
            MoveJAcceleration = 0.50;
            MoveJTime = 0.00;
            MoveJBlendRadius = 0.00;

            MoveLVelocity = 0.25;
            MoveLAcceleration = 1.20;

            SpeedJDuration = 0.50;
            ServoJLookaheadTime = 0.10;
            ServoJGain = 300.0;

            ToleranceDeg = 0.50;
            TimeoutMsPerPoint = 20000;
            PollIntervalMs = 20;
            StableCount = 3;

            RcmX = 500.0;
            RcmY = 0.0;
            RcmZ = 250.0;
            RcmEnabled = true;

            PayloadMass = 2.0;
            PayloadCogX = 0.0;
            PayloadCogY = 0.0;
            PayloadCogZ = 50.0;
        }

        /// <summary>
        /// 用于日志输出，方便确认当前真正使用的 MoveJ 参数。
        /// </summary>
        public static string GetMoveJText()
        {
            return
                $"v={MoveJVelocity:F3}, " +
                $"a={MoveJAcceleration:F3}, " +
                $"t={MoveJTime:F3}, " +
                $"r={MoveJBlendRadius:F3}";
        }

        private static double SafePositive(
            double value,
            double fallback,
            double min,
            double max)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
                return fallback;

            if (value <= 0.0)
                return fallback;

            return Clamp(value, min, max);
        }

        private static double SafeNonNegative(
            double value,
            double fallback,
            double min,
            double max)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
                return fallback;

            if (value < 0.0)
                return fallback;

            return Clamp(value, min, max);
        }

        private static double Clamp(double value, double min, double max)
        {
            if (value < min)
                return min;

            if (value > max)
                return max;

            return value;
        }
    }
}