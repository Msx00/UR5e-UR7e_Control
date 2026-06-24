using System;

namespace WpfRobot.command
{
    public enum MotionSourceType
    {
        UI,
        Omega,
        AI
    }

    public enum MotionCommandType
    {
        JointTarget,
        JointTrajectory,
        Stop
    }

    /// <summary>
    /// 统一运动命令。
    /// 当前支持 6 轴 UR 关节目标。
    /// </summary>
    public class RobotMotionCommand
    {
        public MotionSourceType Source { get; set; }

        public MotionCommandType CommandType { get; set; } = MotionCommandType.JointTarget;

        /// <summary>
        /// 6轴目标角，单位 degree。
        /// [J1,J2,J3,J4,J5,J6]
        /// </summary>
        public double[] JointDeg6 { get; set; }

        public double MoveJVelocity { get; set; } = 0.30;
        public double MoveJAcceleration { get; set; } = 0.50;
        public double MoveJTime { get; set; } = 0.00;
        public double MoveJBlendRadius { get; set; } = 0.00;

        public string Description { get; set; } = "";

        public DateTime Time { get; set; } = DateTime.Now;

        public bool IsValidJointTarget()
        {
            return JointDeg6 != null && JointDeg6.Length >= 6;
        }

        public double[] GetUrJointDeg6()
        {
            if (JointDeg6 == null || JointDeg6.Length < 6)
                return null;

            return new[]
            {
                JointDeg6[0],
                JointDeg6[1],
                JointDeg6[2],
                JointDeg6[3],
                JointDeg6[4],
                JointDeg6[5]
            };
        }
    }
}