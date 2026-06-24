using System;
using System.Collections.Generic;
using WpfRobot.simulation;
using static WpfRobot.simulation.RobotVisualization;

namespace WpfRobot.collision
{
    //当前UI更新的碰撞检测没有设定SDK，如果后续需要，再整理进来吧，在SimulationRealTime.cs;
    //如果要设置，需要也修改MainWindow.xaml.cs里面的调用，以统一接口
    public class collisionSDK
    {
        /// <summary>
        /// 静默的碰撞检测SDK，提供接口供外部调用，内部调用simulationRealTime进行碰撞检测计算
        /// 360度角度制输入
        /// </summary>
        private readonly SimulationRealTime simulationRealTime;

        public collisionSDK(SimulationRealTime simulationRealTime)
        {
            if (simulationRealTime == null)
                throw new ArgumentNullException(nameof(simulationRealTime));

            this.simulationRealTime = simulationRealTime;
        }

        public CollisionReport CheckCollisionForJointAngles(double[] jointsDeg)
        {
            return simulationRealTime.CheckCollisionForJointAngles(jointsDeg);
        }

        public TrajectoryCollisionResult CheckTrajectoryCollision(
            IList<double[]> jointPath,
            int interpolationCountPerSegment = 5)
        {
            return simulationRealTime.CheckTrajectoryCollision(
                jointPath,
                interpolationCountPerSegment
            );
        }
    }
}