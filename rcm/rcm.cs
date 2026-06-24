using System;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using WpfRobot.kinematics;

namespace WpfRobot.rcm
{
    /// <summary>
    /// RCM 远心运动求解模块。
    ///
    /// 核心思想：
    /// 1. 工具轴线始终穿过固定 RCM 点；
    /// 2. RCM 模式下，不直接自由控制 TCP XYZ，而是控制：
    ///    - pitch：绕当前工具局部 X 轴摆动
    ///    - yaw：绕当前工具局部 Y 轴摆动
    ///    - insertion：沿工具轴线进退
    ///    - roll：绕工具自身轴线旋转
    /// 3. 根据 RCM 几何关系构造目标 T0Tcp；
    /// 4. 调用现有七轴 IK 求目标关节角；
    /// 5. 用正运动学回代验证 RCM 点到工具轴线的误差。
    /// </summary>
    public class rcm
    {
        /// <summary>
        /// RCM 求解结果。
        /// </summary>
        public class RcmSolveResult
        {
            public bool Success { get; set; }
            public string Message { get; set; } = "";

            /// <summary>
            /// 目标七轴关节角，单位 degree。
            /// 可以直接填入 RobotMotionCommand.JointDeg7。
            /// </summary>
            public double[] TargetJointDeg7 { get; set; }

            /// <summary>
            /// 构造出来的目标 TCP 位姿。
            /// </summary>
            public Matrix<double> TargetTcpTransform { get; set; }

            /// <summary>
            /// 回代后的 RCM 误差，单位 mm。
            /// </summary>
            public double RcmErrorMm { get; set; }

            /// <summary>
            /// 当前/目标插入深度，单位 mm。
            /// </summary>
            public double InsertionMm { get; set; }

            /// <summary>
            /// 目标工具轴线方向，base 坐标系下单位向量。
            /// </summary>
            public double[] TargetAxis { get; set; }
        }

        private class AxisFrame
        {
            public Vector<double> X;
            public Vector<double> Y;
            public Vector<double> Z;
        }

        /// <summary>
        /// RCM 单步运动求解。
        ///
        /// 输入：
        /// currentJointDeg7：当前七轴关节角，单位 degree
        /// rcmX/Y/Z：RCM 点在 base 坐标系下的位置，单位 mm
        /// deltaPitchDeg：绕当前工具局部 X 轴摆动，单位 degree
        /// deltaYawDeg：绕当前工具局部 Y 轴摆动，单位 degree
        /// deltaInsertionMm：沿工具轴线进退，单位 mm
        /// deltaRollDeg：绕工具自身轴线自转，单位 degree
        ///
        /// 输出：
        /// RcmSolveResult.TargetJointDeg7，单位 degree。
        /// </summary>
        public static RcmSolveResult SolveStep(
            double[] currentJointDeg7,
            double rcmX,
            double rcmY,
            double rcmZ,
            double deltaPitchDeg,
            double deltaYawDeg,
            double deltaInsertionMm,
            double deltaRollDeg,
            double q7MinDeg = -60.0,
            double q7MaxDeg = 60.0,
            double q7StepDeg = 1.0,
            double ikPosTolMm = 1.0,
            double ikRotTolDeg = 1.0,
            double maxRcmErrorMm = 1.0,
            double minInsertionMm = 5.0,
            double maxInsertionMm = 600.0)
        {
            try
            {
                if (currentJointDeg7 == null || currentJointDeg7.Length != 7)
                {
                    return Fail("currentJointDeg7 必须是 7 个关节角，单位 degree。");
                }

                Vector<double> rcmPoint = Vec(rcmX, rcmY, rcmZ);

                // 1. 当前正运动学
                double[] currentJointRad7 = DegArrayToRadArray(currentJointDeg7);
                var fk = Forward.ForwardKinematicsMatrix7_All(currentJointRad7);

                Matrix<double> T07 = fk.T07;
                Matrix<double> T0Tcp = fk.T0Tcp;

                Vector<double> pJ7 = GetPosition(T07);
                Vector<double> pTcp = GetPosition(T0Tcp);

                // 2. 当前工具轴线方向
                // 默认认为器械轴线由 J7 原点指向 TCP 点。
                // 如果你的器械轴线不是 J7->TCP，需要改这里。
                Vector<double> currentAxis = SafeNormalize(
                    pTcp - pJ7,
                    "当前 J7 到 TCP 的距离过小，无法定义工具轴线。"
                );

                // 3. 当前插入深度
                // pTcp ≈ p_rcm + insertion * axis
                double currentInsertion = (pTcp - rcmPoint).DotProduct(currentAxis);
                double targetInsertion = Clamp(
                    currentInsertion + deltaInsertionMm,
                    minInsertionMm,
                    maxInsertionMm
                );

                // 4. 构造当前工具局部坐标系
                // 优先使用当前 TCP 的 X 轴作为参考，避免 roll 跳变。
                Vector<double> currentTcpX = GetColumnVector(T0Tcp, 0);
                AxisFrame currentFrame = BuildFrameFromAxisAndReferenceX(
                    currentAxis,
                    currentTcpX,
                    0.0
                );

                // 5. 根据 pitch/yaw 更新工具轴线
                AxisFrame targetNoRollFrame = ApplyPitchYaw(
                    currentFrame,
                    DegToRad(deltaPitchDeg),
                    DegToRad(deltaYawDeg)
                );

                // 6. 在新工具轴线基础上叠加 roll
                AxisFrame targetFrame = BuildFrameFromAxisAndReferenceX(
                    targetNoRollFrame.Z,
                    targetNoRollFrame.X,
                    DegToRad(deltaRollDeg)
                );

                // 7. RCM 几何约束：TCP 位于 RCM 点沿工具轴线方向的 insertion 位置
                Vector<double> targetTcpPos =
                    rcmPoint + targetFrame.Z * targetInsertion;

                Matrix<double> T0TcpTarget = BuildTransform(
                    targetFrame.X,
                    targetFrame.Y,
                    targetFrame.Z,
                    targetTcpPos
                );

                // 8. 调用你已有的七轴 TCP 逆解
                double[] targetJointDeg7 = Inverse.SolveBestIK7ForTcp(
                    T0TcpTarget,
                    currentJointDeg7,
                    q7MinDeg,
                    q7MaxDeg,
                    q7StepDeg,
                    ikPosTolMm,
                    ikRotTolDeg
                );

                if (targetJointDeg7 == null || targetJointDeg7.Length != 7)
                {
                    return Fail("RCM 目标位姿七轴 IK 失败，当前方向/插入深度可能不可达。");
                }

                // 9. 正运动学回代验证 RCM 误差
                double[] targetJointRad7 = DegArrayToRadArray(targetJointDeg7);
                var fkCheck = Forward.ForwardKinematicsMatrix7_All(targetJointRad7);

                double rcmError = ComputeRcmErrorMm(
                    fkCheck.T07,
                    fkCheck.T0Tcp,
                    new[] { rcmX, rcmY, rcmZ }
                );

                if (rcmError > maxRcmErrorMm)
                {
                    return new RcmSolveResult
                    {
                        Success = false,
                        Message =
                            $"RCM IK 有解，但回代误差过大：{rcmError:F3} mm > {maxRcmErrorMm:F3} mm。",
                        TargetJointDeg7 = targetJointDeg7,
                        TargetTcpTransform = T0TcpTarget,
                        RcmErrorMm = rcmError,
                        InsertionMm = targetInsertion,
                        TargetAxis = ToArray(targetFrame.Z)
                    };
                }

                return new RcmSolveResult
                {
                    Success = true,
                    Message = "RCM 求解成功。",
                    TargetJointDeg7 = targetJointDeg7,
                    TargetTcpTransform = T0TcpTarget,
                    RcmErrorMm = rcmError,
                    InsertionMm = targetInsertion,
                    TargetAxis = ToArray(targetFrame.Z)
                };
            }
            catch (Exception ex)
            {
                return Fail("RCM 求解异常：" + ex.Message);
            }
        }

        /// <summary>
        /// 根据指定工具轴线方向和插入深度直接求解 RCM 目标。
        /// 适合 AI / 路径规划模块直接给目标方向。
        /// </summary>
        public static RcmSolveResult SolveByTargetAxis(
            double[] currentJointDeg7,
            double rcmX,
            double rcmY,
            double rcmZ,
            double[] targetAxis,
            double insertionMm,
            double rollDeg = 0.0,
            double q7MinDeg = -60.0,
            double q7MaxDeg = 60.0,
            double q7StepDeg = 1.0,
            double ikPosTolMm = 1.0,
            double ikRotTolDeg = 1.0,
            double maxRcmErrorMm = 1.0)
        {
            try
            {
                if (currentJointDeg7 == null || currentJointDeg7.Length != 7)
                    return Fail("currentJointDeg7 必须是 7 个关节角，单位 degree。");

                if (targetAxis == null || targetAxis.Length != 3)
                    return Fail("targetAxis 必须是长度为 3 的方向向量。");

                Vector<double> rcmPoint = Vec(rcmX, rcmY, rcmZ);
                Vector<double> axis = SafeNormalize(
                    Vec(targetAxis[0], targetAxis[1], targetAxis[2]),
                    "targetAxis 长度过小。"
                );

                double[] currentRad7 = DegArrayToRadArray(currentJointDeg7);
                var fk = Forward.ForwardKinematicsMatrix7_All(currentRad7);

                Vector<double> currentTcpX = GetColumnVector(fk.T0Tcp, 0);

                AxisFrame frame = BuildFrameFromAxisAndReferenceX(
                    axis,
                    currentTcpX,
                    DegToRad(rollDeg)
                );

                Vector<double> targetTcpPos = rcmPoint + axis * insertionMm;

                Matrix<double> T0TcpTarget = BuildTransform(
                    frame.X,
                    frame.Y,
                    frame.Z,
                    targetTcpPos
                );

                double[] targetJointDeg7 = Inverse.SolveBestIK7ForTcp(
                    T0TcpTarget,
                    currentJointDeg7,
                    q7MinDeg,
                    q7MaxDeg,
                    q7StepDeg,
                    ikPosTolMm,
                    ikRotTolDeg
                );

                if (targetJointDeg7 == null || targetJointDeg7.Length != 7)
                {
                    return Fail("指定工具轴线方向下七轴 IK 失败。");
                }

                var fkCheck = Forward.ForwardKinematicsMatrix7_All(
                    DegArrayToRadArray(targetJointDeg7)
                );

                double rcmError = ComputeRcmErrorMm(
                    fkCheck.T07,
                    fkCheck.T0Tcp,
                    new[] { rcmX, rcmY, rcmZ }
                );

                bool ok = rcmError <= maxRcmErrorMm;

                return new RcmSolveResult
                {
                    Success = ok,
                    Message = ok
                        ? "RCM 指定方向求解成功。"
                        : $"RCM 指定方向 IK 有解，但回代误差过大：{rcmError:F3} mm。",
                    TargetJointDeg7 = targetJointDeg7,
                    TargetTcpTransform = T0TcpTarget,
                    RcmErrorMm = rcmError,
                    InsertionMm = insertionMm,
                    TargetAxis = ToArray(axis)
                };
            }
            catch (Exception ex)
            {
                return Fail("RCM 指定方向求解异常：" + ex.Message);
            }
        }

        /// <summary>
        /// 计算 RCM 点到当前工具轴线的垂直距离。
        /// 工具轴线默认由 T07 原点指向 T0Tcp 原点。
        /// </summary>
        public static double ComputeRcmErrorMm(
            Matrix<double> T07,
            Matrix<double> T0Tcp,
            double[] rcmPoint)
        {
            if (T07 == null || T0Tcp == null)
                return double.MaxValue;

            if (rcmPoint == null || rcmPoint.Length != 3)
                return double.MaxValue;

            Vector<double> pJ7 = GetPosition(T07);
            Vector<double> pTcp = GetPosition(T0Tcp);
            Vector<double> c = Vec(rcmPoint[0], rcmPoint[1], rcmPoint[2]);

            Vector<double> u = pTcp - pJ7;
            double norm = u.L2Norm();

            if (norm < 1e-9)
                return double.MaxValue;

            u = u / norm;

            // RCM 点到直线 pJ7 + lambda * u 的垂直距离
            Vector<double> v = c - pJ7;
            Vector<double> parallel = u * v.DotProduct(u);
            Vector<double> perpendicular = v - parallel;

            return perpendicular.L2Norm();
        }

        /// <summary>
        /// 获取当前工具轴线方向。
        /// </summary>
        public static double[] GetCurrentToolAxis(double[] jointDeg7)
        {
            if (jointDeg7 == null || jointDeg7.Length != 7)
                return null;

            var fk = Forward.ForwardKinematicsMatrix7_All(
                DegArrayToRadArray(jointDeg7)
            );

            Vector<double> pJ7 = GetPosition(fk.T07);
            Vector<double> pTcp = GetPosition(fk.T0Tcp);

            Vector<double> axis = SafeNormalize(
                pTcp - pJ7,
                "当前 J7 到 TCP 的距离过小，无法定义工具轴线。"
            );

            return ToArray(axis);
        }

        /// <summary>
        /// 获取当前插入深度。
        /// insertion = dot(TCP - RCM, toolAxis)
        /// </summary>
        public static double GetCurrentInsertionMm( double[] jointDeg7, double rcmX, double rcmY, double rcmZ)
        {
            if (jointDeg7 == null || jointDeg7.Length != 7)
                return 0.0;

            var fk = Forward.ForwardKinematicsMatrix7_All(
                DegArrayToRadArray(jointDeg7)
            );

            Vector<double> pJ7 = GetPosition(fk.T07);
            Vector<double> pTcp = GetPosition(fk.T0Tcp);
            Vector<double> c = Vec(rcmX, rcmY, rcmZ);

            Vector<double> axis = SafeNormalize(
                pTcp - pJ7,
                "当前 J7 到 TCP 的距离过小，无法定义工具轴线。"
            );

            return (pTcp - c).DotProduct(axis);
        }

        // =====================================================
        // 内部几何函数
        // =====================================================

        private static AxisFrame ApplyPitchYaw(
            AxisFrame currentFrame,
            double pitchRad,
            double yawRad)
        {
            // pitch：绕当前局部 X 轴
            Vector<double> x1 = currentFrame.X;
            Vector<double> y1 = RotateAroundAxis(currentFrame.Y, currentFrame.X, pitchRad);
            Vector<double> z1 = RotateAroundAxis(currentFrame.Z, currentFrame.X, pitchRad);

            // yaw：绕 pitch 后的局部 Y 轴
            Vector<double> x2 = RotateAroundAxis(x1, y1, yawRad);
            Vector<double> z2 = RotateAroundAxis(z1, y1, yawRad);

            return BuildFrameFromAxisAndReferenceX(z2, x2, 0.0);
        }

        private static AxisFrame BuildFrameFromAxisAndReferenceX(
            Vector<double> zAxis,
            Vector<double> xReference,
            double rollRad)
        {
            Vector<double> z = SafeNormalize(zAxis, "zAxis 长度过小。");

            Vector<double> x = xReference - z * xReference.DotProduct(z);

            if (x.L2Norm() < 1e-9)
            {
                Vector<double> baseX = Vec(1.0, 0.0, 0.0);
                Vector<double> baseY = Vec(0.0, 1.0, 0.0);

                x = baseX - z * baseX.DotProduct(z);

                if (x.L2Norm() < 1e-9)
                    x = baseY - z * baseY.DotProduct(z);
            }

            x = SafeNormalize(x, "无法构造工具 X 轴。");

            // y = z × x，可保证 x × y = z
            Vector<double> y = SafeNormalize(
                Cross(z, x),
                "无法构造工具 Y 轴。"
            );

            if (Math.Abs(rollRad) > 1e-12)
            {
                double c = Math.Cos(rollRad);
                double s = Math.Sin(rollRad);

                Vector<double> xRoll = x * c + y * s;
                Vector<double> yRoll = x * (-s) + y * c;

                x = SafeNormalize(xRoll, "roll 后 X 轴异常。");
                y = SafeNormalize(yRoll, "roll 后 Y 轴异常。");
            }

            return new AxisFrame
            {
                X = x,
                Y = y,
                Z = z
            };
        }

        private static Vector<double> RotateAroundAxis(
            Vector<double> v,
            Vector<double> axis,
            double angleRad)
        {
            Vector<double> k = SafeNormalize(axis, "旋转轴长度过小。");

            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);

            // Rodrigues 公式
            return v * c
                   + Cross(k, v) * s
                   + k * (k.DotProduct(v) * (1.0 - c));
        }

        private static Matrix<double> BuildTransform(
            Vector<double> x,
            Vector<double> y,
            Vector<double> z,
            Vector<double> p)
        {
            return DenseMatrix.OfArray(new double[,]
            {
                { x[0], y[0], z[0], p[0] },
                { x[1], y[1], z[1], p[1] },
                { x[2], y[2], z[2], p[2] },
                { 0.0,  0.0,  0.0,  1.0  }
            });
        }

        private static Vector<double> GetPosition(Matrix<double> T)
        {
            return Vec(T[0, 3], T[1, 3], T[2, 3]);
        }

        private static Vector<double> GetColumnVector(Matrix<double> T, int col)
        {
            return Vec(T[0, col], T[1, col], T[2, col]);
        }

        private static Vector<double> Cross(
            Vector<double> a,
            Vector<double> b)
        {
            return Vector<double>.Build.DenseOfArray(new double[]
            {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            });
        }

        private static Vector<double> SafeNormalize(
            Vector<double> v,
            string errorMessage)
        {
            double n = v.L2Norm();

            if (n < 1e-9)
                throw new InvalidOperationException(errorMessage);

            return v / n;
        }

        private static Vector<double> Vec(double x, double y, double z)
        {
            return Vector<double>.Build.DenseOfArray(new double[]
            {
                x, y, z
            });
        }

        private static double[] ToArray(Vector<double> v)
        {
            return new[]
            {
                v[0],
                v[1],
                v[2]
            };
        }

        private static double[] DegArrayToRadArray(double[] qDeg)
        {
            if (qDeg == null)
                return null;

            double[] qRad = new double[qDeg.Length];

            for (int i = 0; i < qDeg.Length; i++)
                qRad[i] = DegToRad(qDeg[i]);

            return qRad;
        }
        /// <summary>
        /// 围绕当前工具轴线生成圆锥形 RCM 演示运动。
        ///
        /// centerAxis：圆锥中心轴线，base坐标系下单位向量。
        /// phaseDeg：圆周相位角，0~360 度。
        /// coneAngleDeg：圆锥半角，例如 5°、10°、15°。
        /// insertionMm：TCP 到 RCM 点沿工具轴线的距离。
        ///
        /// 输出：满足 RCM 约束的目标七轴关节角。
        /// </summary>
        public static RcmSolveResult SolveConeAroundRcm(
            double[] currentJointDeg7,
            double rcmX,
            double rcmY,
            double rcmZ,
            double[] centerAxis,
            double phaseDeg,
            double coneAngleDeg,
            double insertionMm,
            double rollDeg = 0.0,
            double q7MinDeg = -60.0,
            double q7MaxDeg = 60.0,
            double q7StepDeg = 1.0,
            double ikPosTolMm = 1.0,
            double ikRotTolDeg = 1.0,
            double maxRcmErrorMm = 1.0)
        {
            try
            {
                if (currentJointDeg7 == null || currentJointDeg7.Length != 7)
                    return Fail("currentJointDeg7 必须是 7 个关节角，单位 degree。");

                if (centerAxis == null || centerAxis.Length != 3)
                    return Fail("centerAxis 必须是长度为 3 的方向向量。");

                Vector<double> w = SafeNormalize(
                    Vec(centerAxis[0], centerAxis[1], centerAxis[2]),
                    "centerAxis 长度过小。"
                );

                // 构造与中心轴 w 垂直的两个单位向量 u、v
                Vector<double> refVec = Vec(0.0, 0.0, 1.0);

                if (Math.Abs(w.DotProduct(refVec)) > 0.95)
                    refVec = Vec(1.0, 0.0, 0.0);

                Vector<double> u = refVec - w * refVec.DotProduct(w);
                u = SafeNormalize(u, "无法构造圆锥基向量 u。");

                Vector<double> v = Cross(w, u);
                v = SafeNormalize(v, "无法构造圆锥基向量 v。");

                double phaseRad = DegToRad(phaseDeg);
                double coneRad = DegToRad(coneAngleDeg);

                // 圆锥表面上的工具轴线方向
                Vector<double> targetAxis =
                    w * Math.Cos(coneRad)
                    + u * (Math.Sin(coneRad) * Math.Cos(phaseRad))
                    + v * (Math.Sin(coneRad) * Math.Sin(phaseRad));

                targetAxis = SafeNormalize(targetAxis, "targetAxis 长度过小。");

                return SolveByTargetAxis(
                    currentJointDeg7,
                    rcmX,
                    rcmY,
                    rcmZ,
                    ToArray(targetAxis),
                    insertionMm,
                    rollDeg,
                    q7MinDeg,
                    q7MaxDeg,
                    q7StepDeg,
                    ikPosTolMm,
                    ikRotTolDeg,
                    maxRcmErrorMm
                );
            }
            catch (Exception ex)
            {
                return Fail("RCM 圆锥演示求解异常：" + ex.Message);
            }
        }

        private static double DegToRad(double deg)
        {
            return deg * Math.PI / 180.0;
        }

        private static double Clamp(double v, double min, double max)
        {
            if (v < min)
                return min;

            if (v > max)
                return max;

            return v;
        }

        private static RcmSolveResult Fail(string msg)
        {
            return new RcmSolveResult
            {
                Success = false,
                Message = msg,
                TargetJointDeg7 = null,
                TargetTcpTransform = null,
                RcmErrorMm = double.MaxValue,
                InsertionMm = 0.0,
                TargetAxis = null
            };
        }
    }
}