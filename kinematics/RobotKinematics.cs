using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace WpfRobot.kinematics
{
    public static class VectorExtensions
    {
        public static MathNet.Numerics.LinearAlgebra.Vector<double> CrossProduct(this MathNet.Numerics.LinearAlgebra.Vector<double> a, MathNet.Numerics.LinearAlgebra.Vector<double> b)
        {
            if (a.Count != 3 || b.Count != 3)
                throw new ArgumentException("Cross product is only defined for 3D vectors.");

            return MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(new double[]
            {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            });
        }
    }

    public class Forward
    {
        public struct DHParam
        {
            public double a;      // 链节长度，单位 mm
            public double alpha;  // 链节扭转角，单位 rad
            public double d;      // 链节偏移，单位 mm
            public double theta;  // 关节角，单位 rad
        }

        // =====================================================
        // UR 本体 DH 参数，单位 mm
        // =====================================================
        public static double a2 = -425.0;
        public static double a3 = -392.2;
        public static double d1 = 162.5;
        public static double d4 = 133.3;
        public static double d5 = 99.7;
        public static double d6 = 99.6;
        public static double d7 = 174;


        /// <summary>
        /// 标准UR六轴末端 T06 到第七轴零位坐标系的固定安装变换。
        ///
        /// 必须和 GetUR7eDH_7() 保持一致：
        /// 第六轴在七轴模型里写成 d = d6 + d7, alpha = pi/2。
        ///
        /// 因此：
        /// T0J7_zero = T06_standard * TransZ(d7) * RotX(pi/2)
        /// </summary>
        public static Matrix<double> GetT6ToJoint7Zero()
        {
            return Trans(0.0, 0.0, d7) * RotX(Math.PI / 2.0);
        }
        /// <summary>
        /// 标准 T06 到第七轴零位坐标系的固定安装变换。
        /// </summary>
        public static Matrix<double> tcpToJoint7()
        {
            return GetT6ToJoint7Zero();
        }

        // =====================================================
        // 工具 TCP 相对于第七轴输出坐标系的固定变换
        //
        // 当前设置：
        // 工具尖端沿第七轴局部 z7 方向伸出 260 mm。
        //
        // 如果工具不是沿 z7 方向伸出，就修改 Tool_X / Tool_Y / Tool_Z。
        // 如果工具端点坐标系还有姿态偏移，就修改 Tool_RxDeg / Tool_RyDeg / Tool_RzDeg。
        // =====================================================
        //public static double Tool_X = 0.0;
        //public static double Tool_Y = 0.0;
        //public static double Tool_Z = 260.0;

        //public static double Tool_RxDeg = 0.0;
        //public static double Tool_RyDeg = 0.0;
        //public static double Tool_RzDeg = 0.0;

        /// <summary>
        /// 从 a_global_robotics 读取工具 TCP 固定变换。
        /// 工具参数统一由 a_global_robotics 管理。
        /// </summary>
        public static Matrix<double> GetTJoint7ToTool()
        {
            return FixedXYZRPYDegToTransform(
                global_variable.globalToolVector[0],
                global_variable.globalToolVector[1],
                global_variable.globalToolVector[2],
                global_variable.globalToolRxDeg,
                global_variable.globalToolRyDeg,
                global_variable.globalToolRzDeg
            );
        }

        /// <summary>
        /// 设置工具长度。
        /// 默认认为工具沿第七轴局部 z7 方向伸出。
        /// </summary>
        public static void SetToolLength(double toolLengthMm)
        {
            global_variable.SetGlobalTool(
                0.0,
                0.0,
                toolLengthMm,
                0.0,
                0.0,
                0.0
            );
        }
        /// <summary>
        /// 正运动学入口。
        ///
        /// joints 可以是 6 个或者 7 个关节角。
        /// 单位必须是弧度制。
        ///
        /// joints.Length == 6:
        ///     返回 UR 本体第六轴末端位姿。
        ///
        /// joints.Length == 7:
        ///     返回 UR 本体 + 第七轴 + 工具 TCP 的系统位姿。
        /// </summary>
        public static double[] ForwardKinematics(double[] joints)
        {
            Matrix<double> T = ForwardKinematicsMatrix(joints);
            return PoseTransform.TransformToPose(T);
        }

        /// <summary>
        /// 返回 4x4 齐次变换矩阵。
        ///
        /// joints 单位：弧度。
        ///
        /// joints.Length == 6:
        ///     返回 T06。
        ///
        /// joints.Length == 7:
        ///     返回 T0Tool。
        /// </summary>
        public static Matrix<double> ForwardKinematicsMatrix(double[] joints)
        {
            if (joints == null)
                throw new ArgumentNullException(nameof(joints));

            if (joints.Length == 6)
            {
                return ForwardKinematicsMatrix6(joints);
            }
            else if (joints.Length == 7)
            {
                return ForwardKinematicsMatrix7(joints);
            }
            else
            {
                throw new ArgumentException("ForwardKinematics 期望 6 或 7 个关节角，单位为弧度。");
            }
        }

        /// <summary>
        /// UR 本体 6 轴正运动学。
        ///
        /// 输入：
        /// joints6 = [q1, q2, q3, q4, q5, q6]
        ///
        /// 单位：
        /// joints6 为弧度。
        ///
        /// 输出：
        /// T06，表示第六关节末端坐标系在机器人 base 坐标系下的位姿。
        /// </summary>
        private static Matrix<double> ForwardKinematicsMatrix6(double[] joints6)
        {
            if (joints6 == null || joints6.Length != 6)
                throw new ArgumentException("ForwardKinematicsMatrix6 期望 6 个关节角，单位为弧度。");

            DHParam[] dh = GetUR7eDH(joints6);

            Matrix<double> T = DenseMatrix.CreateIdentity(4);

            foreach (DHParam p in dh)
            {
                T = T * DHToTransform(p);
            }

            return T;
        }
        public static (Matrix<double> T06, Matrix<double> T07, Matrix<double> T0Tcp) ForwardKinematicsMatrix6_All(double[] joints6)
        {
            Matrix<double> T06 = ForwardKinematicsMatrix6(joints6);
            Matrix<double> T0Tcp = T06 * GetTJoint7ToTool();

            // 纯六轴模式下保留 T07 字段兼容旧 UI，数值等同于 T06。
            return (T06, T06.Clone(), T0Tcp);
        }
        /// <summary>
        /// UR 本体 + 第七轴正运动学 DH 参数。
        /// 可以写成变量，这样可以灵活改变7关节的安装位置；同时若是参数都置为零的话相当于关闭了关节7
        /// </summary>
        /// <param name="joints"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public static DHParam[] GetUR7eDH_7(double[] joints)
        {
            if (joints == null || joints.Length != 7)
                throw new ArgumentException("GetUR7eDH 期望 7 个关节角，单位为弧度。");

            return new[]
            {
                new DHParam { a = 0,  alpha = Math.PI / 2,  d = d1,      theta = joints[0] },
                new DHParam { a = a2, alpha = 0,            d = 0,       theta = joints[1] },
                new DHParam { a = a3, alpha = 0,            d = 0,       theta = joints[2] },
                new DHParam { a = 0,  alpha = Math.PI / 2,  d = d4,      theta = joints[3] },
                new DHParam { a = 0,  alpha = -Math.PI / 2, d = d5,      theta = joints[4] },
                new DHParam { a = 0,  alpha = Math.PI / 2,  d = (d6+d7), theta = joints[5] },
                new DHParam { a = 0,  alpha = 0,            d = 0,       theta = joints[6] }
            };
        }

        /// <summary>
        /// UR 本体 + 第七轴 
        /// </summary>
        /// <param name="joints7"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        private static Matrix<double> ForwardKinematicsMatrix7(double[] joints7)
        {
            if (joints7 == null || joints7.Length != 7)
                throw new ArgumentException("ForwardKinematicsMatrix7 期望 7 个关节角，单位为弧度。");

            //// 1. 测试UR 本体 6 轴正运动学：base -> joint6
            //double[] joints6 = joints7.Take(6).ToArray();
            //double q7 = joints7[6];
            //Matrix<double> T06 = ForwardKinematicsMatrix6(joints6);

            // 2. 求解 7 轴
            DHParam[] dh = GetUR7eDH_7(joints7);

            Matrix<double> T = DenseMatrix.CreateIdentity(4);

            foreach (DHParam p in dh)
            {
                T = T * DHToTransform(p); //此处输出一个T06 一个T07
            }

            return T;
        }
        public static (Matrix<double> T06, Matrix<double> T07, Matrix<double> T0Tcp) ForwardKinematicsMatrix7_All(double[] joints7)
        {
            if (joints7 == null || joints7.Length != 7)
                throw new ArgumentException("ForwardKinematicsMatrix7_All 期望 7 个关节角，单位为弧度。");

            DHParam[] dh = GetUR7eDH_7(joints7);

            Matrix<double> T = DenseMatrix.CreateIdentity(4);

            Matrix<double> T06 = null;
            Matrix<double> T07 = null;

            for (int i = 0; i < dh.Length; i++)
            {
                T = T * DHToTransform(dh[i]);

                // 乘完第6个 DH 后，得到 T06
                if (i == 5)
                {
                    T06 = T.Clone();
                }

                // 乘完第7个 DH 后，得到 T07
                if (i == 6)
                {
                    T07 = T.Clone();
                }
            }

            Matrix<double> TJoint7ToTool = GetTJoint7ToTool();

            Matrix<double> T0Tcp = T07 * TJoint7ToTool;

            return (T06, T07, T0Tcp);
        }

        /// <summary>
        /// 返回用于可视化“关节转轴坐标系”的 T0J1 ~ T0J7。
        ///
        /// 这个函数用于显示每个关节轴上的局部坐标系：
        /// 1. 原点位于对应关节轴线上；
        /// 2. Z轴与该关节转轴重合；
        /// 3. X/Y轴会随着该关节角 qi 绕 Z 轴旋转。
        ///
        /// 注意：
        /// 这里仍然完全使用 GetUR7eDH_7()，包括第七轴。
        /// J6/J7 的显示偏置只用于 VTK 可视化，不参与真实正运动学。
        /// </summary>
        public static List<Matrix<double>> ForwardKinematicsJointAxisFrames(double[] qRad)
        {
            if (qRad == null)
                throw new ArgumentNullException(nameof(qRad));

            double[] q7 = new double[7];

            if (qRad.Length == 6)
            {
                for (int i = 0; i < 6; i++)
                    q7[i] = qRad[i];

                q7[6] = 0.0;
            }
            else if (qRad.Length == 7)
            {
                for (int i = 0; i < 7; i++)
                    q7[i] = qRad[i];
            }
            else
            {
                throw new ArgumentException("ForwardKinematicsJointAxisFrames 期望 6 或 7 个关节角，单位 rad。");
            }

            DHParam[] dh = GetUR7eDH_7(q7);

            List<Matrix<double>> frames = new List<Matrix<double>>();

            Matrix<double> Tprev = DenseMatrix.CreateIdentity(4);

            // 只用于显示坐标系 X/Y 零位方向，不改变 Tprev 的真实 DH 递推
            double[] axisSign = new double[7]
            {
                1, 1, 1, 1, 1, 1, 1
            };

            double[] axisOffsetDeg = new double[7]
            {
                0, 0, 0, 0, 0,
                0,   // J6 如果和模型差 90°，先试 90 或 -90
                0    // J7 如果和模型差 90°，先试 90 或 -90
            };

            for (int i = 0; i < dh.Length; i++)
            {
                DHParam p = dh[i];

                double thetaForDisplay =
                    axisSign[i] * p.theta + DegToRad(axisOffsetDeg[i]);

                /*
                 * 关节轴坐标系：
                 * Taxis_i = Tprev * RotZ(theta_i) * TransZ(d_i)
                 *
                 * 关键：
                 * 不乘 TransX(a_i) 和 RotX(alpha_i)，否则会变成连杆坐标系，
                 * 不是关节轴上的坐标系。
                 */
                Matrix<double> Taxis =
                    Tprev
                    * RotZ(thetaForDisplay)
                    * Trans(0.0, 0.0, p.d);

                frames.Add(Taxis.Clone());

                /*
                 * 真实 DH 递推仍然必须使用原始 p.theta。
                 * 不能用 thetaForDisplay，否则会把显示偏置污染到真实运动学链。
                 */
                Tprev = Tprev * DHToTransform(p);
            }

            return frames;
        }
        /// <summary>
        /// 返回用于可视化“关节转轴坐标系”的 T0J1 ~ T0J7。
        ///
        /// 这个不是普通的 T01~T07 连杆末端坐标系。
        /// 它的目的：
        /// 1. 坐标系原点位于对应关节轴线上；
        /// 2. Z轴与该关节转轴重合；
        /// 3. X/Y轴会随着该关节角 qi 绕 Z 轴旋转。
        ///
        /// 输入 qRad：6个或7个关节角，单位 rad。
        /// 输出：
        /// frames[0] = 第1关节转轴坐标系
        /// frames[1] = 第2关节转轴坐标系
        /// ...
        /// frames[6] = 第7关节转轴坐标系
        /// </summary>
        public static List<Matrix<double>> ForwardKinematicsJointAxisFrames_old(double[] qRad)
        {
            if (qRad == null)
                throw new ArgumentNullException(nameof(qRad));

            double[] q7 = new double[7];

            if (qRad.Length == 6)
            {
                for (int i = 0; i < 6; i++)
                    q7[i] = qRad[i];

                q7[6] = 0.0;
            }
            else if (qRad.Length == 7)
            {
                for (int i = 0; i < 7; i++)
                    q7[i] = qRad[i];
            }
            else
            {
                throw new ArgumentException("ForwardKinematicsJointAxisFrames 期望 6 或 7 个关节角，单位 rad。");
            }

            DHParam[] dh = GetUR7eDH_7(q7);

            List<Matrix<double>> frames = new List<Matrix<double>>();

            Matrix<double> Tprev = DenseMatrix.CreateIdentity(4);

            for (int i = 0; i < dh.Length; i++)
            {
                DHParam p = dh[i];

                /*
                 * 关键区别：
                 *
                 * 普通 DH 完整变换：
                 * A_i = RotZ(theta_i) * TransZ(d_i) * TransX(a_i) * RotX(alpha_i)
                 *
                 * 但你现在要画“关节转轴坐标系”，不应该乘 TransX(a_i) 和 RotX(alpha_i)，
                 * 否则坐标系就变成了连杆末端坐标系。
                 *
                 * 所以这里使用：
                 * Taxis_i = Tprev * RotZ(theta_i) * TransZ(d_i)
                 *
                 * 这样：
                 * Z轴仍然是当前关节轴；
                 * X/Y轴会随着 theta_i 绕 Z 轴转；
                 * 原点位于当前关节轴线上。
                 */
                Matrix<double> Taxis =
                    Tprev
                    * RotZ(p.theta)
                    * Trans(0.0, 0.0, p.d);

                frames.Add(Taxis.Clone());

                // 再更新 Tprev，用完整 DH 进入下一个关节
                Tprev = Tprev * DHToTransform(p);
            }

            return frames;
        }
        
        /// <summary>
        /// 工具 TCP 正运动学。
        /// 输出：
        /// T0Tool，表示工具 TCP 在机器人 base 坐标系下的位姿。
        /// </summary>
        public static Matrix<double> ForwardKinematicsMatrix7_TCP(double[] joints7)
        {
            if (joints7 == null || joints7.Length != 7)
                throw new ArgumentException("ForwardKinematicsMatrix7 期望 7 个关节角，单位为弧度。");

            // 1. UR 本体 7 轴正运动学：base -> joint7
            Matrix<double> T07 = ForwardKinematicsMatrix7(joints7);

            // 4. 第七轴输出坐标系 -> 工具 TCP
            Matrix<double> TJoint7ToTool = GetTJoint7ToTool();

            // 5. 完整系统正运动学
            // T07* TJoint7ToTool:
            //     工具 TCP 在 base 下的位姿。
            Matrix<double> T0Tool = T07 * TJoint7ToTool;

            return T0Tool;
        }

        /// <summary>
        /// 获取 UR 本体 6 轴 DH 参数。
        ///
        /// 注意：
        /// 这里仍然只接收 6 个关节角。
        /// 第七轴不要塞进 DH 表，而是在 ForwardKinematicsMatrix7 里额外乘固定变换和旋转变换。
        /// </summary>
        public static DHParam[] GetUR7eDH(double[] joints)
        {
            if (joints == null || joints.Length != 6)
                throw new ArgumentException("GetUR7eDH 期望 6 个关节角，单位为弧度。");

            return new[]
            {
                new DHParam { a = 0,  alpha = Math.PI / 2,  d = d1, theta = joints[0] },
                new DHParam { a = a2, alpha = 0,            d = 0,  theta = joints[1] },
                new DHParam { a = a3, alpha = 0,            d = 0,  theta = joints[2] },
                new DHParam { a = 0,  alpha = Math.PI / 2,  d = d4, theta = joints[3] },
                new DHParam { a = 0,  alpha = -Math.PI / 2, d = d5, theta = joints[4] },
                new DHParam { a = 0,  alpha = 0,            d = d6, theta = joints[5] }
            };
        }

        /// <summary>
        /// 根据标准 DH 参数计算单个关节的 4x4 齐次变换矩阵。
        /// 单位：弧度制
        /// </summary>
        public static Matrix<double> DHToTransform(DHParam p)
        {
            double ct = Math.Cos(p.theta);
            double st = Math.Sin(p.theta);
            double ca = Math.Cos(p.alpha);
            double sa = Math.Sin(p.alpha);

            return DenseMatrix.OfArray(new double[,]
            {
            { ct, -st * ca,  st * sa, p.a * ct },
            { st,  ct * ca, -ct * sa, p.a * st },
            { 0,        sa,       ca,      p.d  },
            { 0,         0,        0,       1   }
            });
        }

        // =====================================================
        // 普通刚体变换工具函数
        // =====================================================

        /// <summary>
        /// 根据 x, y, z, rx, ry, rz 生成固定刚体变换。
        ///
        /// 平移单位：mm。
        /// 旋转单位：degree。
        ///
        /// 矩阵形式：
        /// T = Trans(x,y,z) * RotX(rx) * RotY(ry) * RotZ(rz)
        /// </summary>
        public static Matrix<double> FixedXYZRPYDegToTransform(
            double x, double y, double z,
            double rxDeg, double ryDeg, double rzDeg)
        {
            double rx = DegToRad(rxDeg);
            double ry = DegToRad(ryDeg);
            double rz = DegToRad(rzDeg);

            return Trans(x, y, z) * RotX(rx) * RotY(ry) * RotZ(rz);
        }

        public static double DegToRad(double degree)
        {
            return degree * Math.PI / 180.0;
        }

        public static Matrix<double> Trans(double x, double y, double z)
        {
            return DenseMatrix.OfArray(new double[,]
            {
            { 1, 0, 0, x },
            { 0, 1, 0, y },
            { 0, 0, 1, z },
            { 0, 0, 0, 1 }
            });
        }

        public static Matrix<double> RotX(double angleRad)
        {
            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);

            return DenseMatrix.OfArray(new double[,]
            {
            { 1, 0,  0, 0 },
            { 0, c, -s, 0 },
            { 0, s,  c, 0 },
            { 0, 0,  0, 1 }
            });
        }

        public static Matrix<double> RotY(double angleRad)
        {
            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);

            return DenseMatrix.OfArray(new double[,]
            {
            {  c, 0, s, 0 },
            {  0, 1, 0, 0 },
            { -s, 0, c, 0 },
            {  0, 0, 0, 1 }
            });
        }

        public static Matrix<double> RotZ(double angleRad)
        {
            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);

            return DenseMatrix.OfArray(new double[,]
            {
            { c, -s, 0, 0 },
            { s,  c, 0, 0 },
            { 0,  0, 1, 0 },
            { 0,  0, 0, 1 }
            });
        }
    }

    public static class Inverse
    {
        const double ZERO_THRESH = 1e-8;
        const double PI = Math.PI;

        const double d1 = 162.5;
        const double a2 = -425;
        const double a3 = -392.2;
        const double d4 = 133.3;
        const double d5 = 99.7;
        const double d6 = 99.6;

        // =====================================================
        // 6 轴 / 7 轴最优解筛选
        // =====================================================

        /// <summary>
        /// 筛选最优六轴逆解。
        /// 输入：
        /// joints_list：候选解列表，每个解是 double[6]，单位 degree
        /// current_joint：当前关节角 double[6]，单位 degree
        /// 输出：
        /// 最接近当前关节角的六轴解，单位 degree
        /// </summary>
        public static double[] SelectBestIKSolution(List<double[]> joints_list, double[] current_joint)
        {
            if (joints_list == null || joints_list.Count == 0)
                return null;

            if (current_joint == null || current_joint.Length != 6)
                throw new ArgumentException("current_joint 必须是 6 个关节角，单位 degree。");

            double minDistance = double.MaxValue;
            double[] bestSolution = null;

            foreach (var sol in joints_list)
            {
                if (sol == null || sol.Length != 6)
                    continue;

                double distance = 0.0;

                for (int i = 0; i < 6; i++)
                {
                    double diff = WrapTo180(sol[i] - current_joint[i]);
                    distance += Math.Abs(diff);
                }

                if (distance < minDistance)
                {
                    minDistance = distance;
                    bestSolution = sol;
                }
            }

            if (bestSolution == null)
                return null;

            double[] adjustedSolution = new double[6];

            for (int i = 0; i < 6; i++)
            {
                adjustedSolution[i] = AdjustToClosestDegree(bestSolution[i], current_joint[i]);
            }

            return adjustedSolution;
        }

        /// <summary>
        /// TCP 六轴逆解主入口：给 UI 使用。
        /// </summary>
        public static double[] SolveBestIKForTcp(
            Matrix<double> T0TcpTarget,
            double[] currentJointDeg,
            double posTolMm = 1.0,
            double rotTolDeg = 1.0)
        {
            if (currentJointDeg == null || currentJointDeg.Length != 6)
                throw new ArgumentException("currentJointDeg 必须是 6 个关节角，单位 degree。");

            double q6DesRad = currentJointDeg[5] * Math.PI / 180.0;
            List<double[]> candidates = InverseKinematics(T0TcpTarget, "angle", q6DesRad);

            if (candidates == null || candidates.Count == 0)
                return null;

            return SelectBestIKSolution(candidates, currentJointDeg);
        }
        /// <summary>
        /// TCP 七轴逆解主入口：给 UI 使用。
        ///
        /// 输入：
        /// T0TcpTarget：目标 TCP 位姿
        /// currentJointDeg：当前七个关节角，单位 degree
        ///
        /// 输出：
        /// 最优七轴解，单位 degree。
        /// 如果无解，返回 null。
        /// </summary>
        public static double[] SolveBestIK7ForTcp(
            Matrix<double> T0TcpTarget,
            double[] currentJointDeg,
            double q7MinDeg = -60.0,
            double q7MaxDeg = 60.0,
            double q7StepDeg = 1.0,
            double posTolMm = 1.0,
            double rotTolDeg = 1.0)
        {
            if (currentJointDeg == null || currentJointDeg.Length != 7)
                throw new ArgumentException("currentJointDeg 必须是 7 个关节角，单位 degree。");

            double currentQ7Deg = currentJointDeg[6];
            double q6DesRad = currentJointDeg[5] * Math.PI / 180.0;

            // 1. 优先保持当前 q7 不变
            List<double[]> candidates = InverseKinematics7KnownQ7(
                T0TcpTarget,
                currentQ7Deg,
                "angle",
                q6DesRad,
                posTolMm,
                rotTolDeg,
                true
            );

            // 2. 如果当前 q7 无解，再采样 q7
            if (candidates == null || candidates.Count == 0)
            {
                candidates = InverseKinematics7SampleQ7(
                    T0TcpTarget,
                    q7MinDeg,
                    q7MaxDeg,
                    q7StepDeg,
                    "angle",
                    q6DesRad,
                    posTolMm,
                    rotTolDeg
                );
            }

            if (candidates == null || candidates.Count == 0)
                return null;

            // 3. 从所有候选解里选最接近当前关节角的解
            return SelectBestIKSolution7(candidates, currentJointDeg);
        }

        /// <summary>
        /// 筛选最优七轴逆解。
        /// 输入：
        /// jointsList：候选解列表，每个解是 double[7]，单位 degree
        /// currentJoint：当前关节角 double[7]，单位 degree
        /// 输出：
        /// 最接近当前关节角的七轴解，单位 degree
        /// </summary>
        public static double[] SelectBestIKSolution7(List<double[]> jointsList, double[] currentJoint)
        {
            if (jointsList == null || jointsList.Count == 0)
                return null;

            if (currentJoint == null || currentJoint.Length != 7)
                throw new ArgumentException("currentJoint 必须是 7 个关节角，单位 degree。");

            double minDistance = double.MaxValue;
            double[] bestSolution = null;

            foreach (double[] sol in jointsList)
            {
                if (sol == null || sol.Length != 7)
                    continue;

                double distance = 0.0;

                for (int i = 0; i < 7; i++)
                {
                    double diff = WrapTo180(sol[i] - currentJoint[i]);
                    distance += Math.Abs(diff);
                }

                if (distance < minDistance)
                {
                    minDistance = distance;
                    bestSolution = sol;
                }
            }

            if (bestSolution == null)
                return null;

            double[] adjustedSolution = new double[7];

            for (int i = 0; i < 7; i++)
            {
                adjustedSolution[i] = AdjustToClosestDegree(bestSolution[i], currentJoint[i]);
            }

            return adjustedSolution;
        }

        /// <summary>
        /// 将角度归一化到 [-180, 180]。
        /// 单位 degree。
        /// </summary>
        public static double WrapTo180(double angle)
        {
            while (angle > 180)
                angle -= 360;

            while (angle < -180)
                angle += 360;

            return angle;
        }

        /// <summary>
        /// 将 targetAngle 调整到与 referenceAngle 最接近的等效角。
        /// 单位 degree。
        /// </summary>
        public static double AdjustToClosestDegree(double targetAngle, double referenceAngle)
        {
            double adjusted = targetAngle;
            double minDiff = Math.Abs(adjusted - referenceAngle);

            for (int k = -2; k <= 2; k++)
            {
                double candidate = targetAngle + k * 360;
                double diff = Math.Abs(candidate - referenceAngle);

                if (diff < minDiff)
                {
                    minDiff = diff;
                    adjusted = candidate;
                }
            }

            return adjusted;
        }


        /// <summary>
        /// 七轴逆运动学：已知目标 TCP 位姿，并且已知第七轴角度 q7。
        ///
        /// 正运动学关系必须和 Forward.GetUR7eDH_7() 一致：
        ///
        /// T0Tcp = T06_standard
        ///         * T6ToJ7Zero
        ///         * RotZ(q7)
        ///         * TJoint7ToTool
        ///
        /// 其中：
        /// T6ToJ7Zero = TransZ(d7) * RotX(pi/2)
        ///
        /// 输入：
        /// T0TcpTarget：工具尖端 TCP 在机器人 base 坐标系下的目标位姿，4x4
        /// q7Deg：第七轴角度，单位 degree
        /// form："angle" 返回 degree；"rad" 返回 rad
        /// q6_des：腕部奇异情况下使用的 q6 期望值，单位 rad
        ///
        /// 输出：
        /// 多组七轴解 [q1, q2, q3, q4, q5, q6, q7]
        /// </summary>
        public static List<double[]> InverseKinematics7KnownQ7(
            Matrix<double> T0TcpTarget,
            double q7Deg,
            string form = "angle",
            double q6_des = 0.0,
            double posTolMm = 1.0,
            double rotTolDeg = 1.0,
            bool validateByForward = true)
        {
            if (T0TcpTarget == null ||
                T0TcpTarget.RowCount != 4 ||
                T0TcpTarget.ColumnCount != 4)
            {
                throw new ArgumentException("T0TcpTarget 必须是 4x4 齐次变换矩阵。");
            }

            string outputForm = form == "rad" ? "rad" : "angle";

            double q7Rad = q7Deg * Math.PI / 180.0;

            // 关键修改：
            // 这里不能再用原来的 TransZ(100)。
            // 必须使用和 GetUR7eDH_7() 一致的安装变换。
            Matrix<double> T6ToJ7Zero = Forward.GetT6ToJoint7Zero();

            // 第七轴自身旋转
            Matrix<double> TJoint7Rotation = Forward.RotZ(q7Rad);

            // 第七轴输出坐标系 -> 工具 TCP
            Matrix<double> TJoint7ToTool = Forward.GetTJoint7ToTool();

            // 从目标 TCP 反推标准六轴末端 T06
            Matrix<double> T06Target =
                T0TcpTarget
                * TJoint7ToTool.Inverse()
                * TJoint7Rotation.Inverse()
                * T6ToJ7Zero.Inverse();

            // 六轴解析 IK 仍然使用你原来正确的函数
            // 内部统一用 rad，最后再按需要转成 degree
            List<double[]> sol6RadList = InverseKinematics(T06Target, "rad", q6_des);

            List<double[]> sol7List = new List<double[]>();

            foreach (double[] sol6Rad in sol6RadList)
            {
                if (sol6Rad == null || sol6Rad.Length != 6)
                    continue;

                double[] sol7Rad = new double[7];

                for (int i = 0; i < 6; i++)
                {
                    sol7Rad[i] = sol6Rad[i];
                }

                sol7Rad[6] = q7Rad;

                // 用正运动学回代验证，防止矩阵链不一致时输出错误解
                if (validateByForward)
                {
                    Matrix<double> TCheck = Forward.ForwardKinematicsMatrix7_TCP(sol7Rad);

                    if (!IsPoseClose(TCheck, T0TcpTarget, posTolMm, rotTolDeg))
                        continue;
                }

                if (outputForm == "rad")
                {
                    sol7List.Add(sol7Rad);
                }
                else
                {
                    sol7List.Add(RadArrayToDegArray(sol7Rad));
                }
            }

            return sol7List;
        }

        /// <summary>
        /// 七轴逆运动学：q7 未知时，通过采样 q7 求多组候选解。
        ///
        /// 注意：
        /// 这个方法本质是利用七自由度冗余性。
        /// 一个 TCP 位姿一般对应多个 q7，因此需要从 q7 范围内采样。
        /// </summary>
        public static List<double[]> InverseKinematics7SampleQ7(
            Matrix<double> T0TcpTarget,
            double q7MinDeg = -60.0,
            double q7MaxDeg = 60.0,
            double q7StepDeg = 1.0,
            string form = "angle",
            double q6_des = 0.0,
            double posTolMm = 1.0,
            double rotTolDeg = 1.0)
        {
            if (T0TcpTarget == null ||
                T0TcpTarget.RowCount != 4 ||
                T0TcpTarget.ColumnCount != 4)
            {
                throw new ArgumentException("T0TcpTarget 必须是 4x4 齐次变换矩阵。");
            }

            if (q7StepDeg <= 0)
                throw new ArgumentException("q7StepDeg 必须大于 0。");

            if (q7MaxDeg < q7MinDeg)
                throw new ArgumentException("q7MaxDeg 不能小于 q7MinDeg。");

            List<double[]> allSolutions = new List<double[]>();

            for (double q7Deg = q7MinDeg;
                 q7Deg <= q7MaxDeg + 1e-9;
                 q7Deg += q7StepDeg)
            {
                List<double[]> sol7List = InverseKinematics7KnownQ7(
                    T0TcpTarget,
                    q7Deg,
                    form,
                    q6_des,
                    posTolMm,
                    rotTolDeg,
                    true
                );

                allSolutions.AddRange(sol7List);
            }

            return allSolutions;
        }
        private static double[] RadArrayToDegArray(double[] qRad)
        {
            if (qRad == null)
                return null;

            double[] qDeg = new double[qRad.Length];

            for (int i = 0; i < qRad.Length; i++)
            {
                qDeg[i] = qRad[i] * 180.0 / Math.PI;
            }

            return qDeg;
        }

        private static bool IsPoseClose(
            Matrix<double> TCheck,
            Matrix<double> TTarget,
            double posTolMm,
            double rotTolDeg)
        {
            double posErr = PositionErrorMm(TCheck, TTarget);
            double rotErrDeg = RotationErrorDeg(TCheck, TTarget);

            return posErr <= posTolMm && rotErrDeg <= rotTolDeg;
        }

        private static double PositionErrorMm(Matrix<double> A, Matrix<double> B)
        {
            double dx = A[0, 3] - B[0, 3];
            double dy = A[1, 3] - B[1, 3];
            double dz = A[2, 3] - B[2, 3];

            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        private static double RotationErrorDeg(Matrix<double> A, Matrix<double> B)
        {
            // R_err = R_A^T * R_B
            // trace(R_err) = sum_ij R_A(i,j) * R_B(i,j)
            double trace = 0.0;

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    trace += A[r, c] * B[r, c];
                }
            }

            double cosTheta = (trace - 1.0) / 2.0;
            cosTheta = Clamp(cosTheta, -1.0, 1.0);

            double angleRad = Math.Acos(cosTheta);

            return angleRad * 180.0 / Math.PI;
        }

        // =====================================================
        // 原始六轴解析逆运动学
        // =====================================================

        /// <summary>
        /// UR 六轴解析逆运动学。
        ///
        /// 输入：
        /// T：目标 T06 位姿，4x4
        /// form：
        ///     "angle" 返回 degree
        ///     "rad" 返回 rad
        /// q6_des：腕部奇异时使用的 q6 期望值，单位 rad
        ///
        /// 输出：
        /// 多组六轴候选解。
        /// </summary>
        public static List<double[]> InverseKinematics(Matrix<double> T, string form = "angle", double q6_des = 0.0)
        {
            if (T == null || T.RowCount != 4 || T.ColumnCount != 4)
                throw new ArgumentException("T 必须是 4x4 齐次变换矩阵。");

            List<double[]> solutions = new List<double[]>();

            double T00 = T[0, 0], T01 = T[0, 1], T02 = T[0, 2], T03 = T[0, 3];
            double T10 = T[1, 0], T11 = T[1, 1], T12 = T[1, 2], T13 = T[1, 3];
            double T20 = T[2, 0], T21 = T[2, 1], T22 = T[2, 2], T23 = T[2, 3];

            // -------------------------------------------------
            // 1. 求 q1
            // -------------------------------------------------
            double A = d6 * T12 - T13;
            double B = d6 * T02 - T03;
            double R = A * A + B * B;

            List<double> q1_list = new List<double>();

            if (Math.Abs(A) < ZERO_THRESH)
            {
                if (Math.Abs(B) < ZERO_THRESH)
                    return solutions;

                double div = -d4 / B;

                if (Math.Abs(div) > 1.0)
                    return solutions;

                double arcsin = Math.Asin(Clamp(div, -1.0, 1.0));

                q1_list.Add(NormalizeAngle(arcsin));
                q1_list.Add(NormalizeAngle(PI - arcsin));
            }
            else if (Math.Abs(B) < ZERO_THRESH)
            {
                double div = d4 / A;

                if (Math.Abs(div) > 1.0)
                    return solutions;

                double arccos = Math.Acos(Clamp(div, -1.0, 1.0));

                q1_list.Add(NormalizeAngle(arccos));
                q1_list.Add(NormalizeAngle(2 * PI - arccos));
            }
            else if (d4 * d4 > R)
            {
                return solutions;
            }
            else
            {
                double arccos = Math.Acos(Clamp(d4 / Math.Sqrt(R), -1.0, 1.0));
                double arctan = Math.Atan2(-B, A);

                double pos = arctan + arccos;
                double neg = arctan - arccos;

                q1_list.Add(NormalizeAngle(pos));
                q1_list.Add(NormalizeAngle(neg));
            }

            // -------------------------------------------------
            // 2. 遍历 q1，求 q5、q6、q2、q3、q4
            // -------------------------------------------------
            foreach (double q1 in q1_list)
            {
                double c1 = Math.Cos(q1);
                double s1 = Math.Sin(q1);

                // 求 q5
                double numer = T03 * s1 - T13 * c1 - d4;
                double div = numer / d6;

                if (Math.Abs(div) > 1.0)
                    continue;

                div = Clamp(div, -1.0, 1.0);

                double arccos_q5 = Math.Acos(div);

                List<double> q5_list = new List<double>
            {
                NormalizeAngle(arccos_q5),
                NormalizeAngle(2 * PI - arccos_q5)
            };

                foreach (double q5 in q5_list)
                {
                    double s5 = Math.Sin(q5);
                    double c5 = Math.Cos(q5);

                    // 求 q6
                    double q6;

                    if (Math.Abs(s5) < ZERO_THRESH)
                    {
                        q6 = NormalizeAngle(q6_des);
                    }
                    else
                    {
                        q6 = Math.Atan2(
                            -(T01 * s1 - T11 * c1) * Math.Sign(s5),
                             (T00 * s1 - T10 * c1) * Math.Sign(s5)
                        );

                        q6 = NormalizeAngle(q6);
                    }

                    double c6 = Math.Cos(q6);
                    double s6 = Math.Sin(q6);

                    // 求 q2、q3、q4
                    double x04x =
                        -s5 * (T02 * c1 + T12 * s1)
                        - c5 * (
                            s6 * (T01 * c1 + T11 * s1)
                            - c6 * (T00 * c1 + T10 * s1)
                        );

                    double x04y =
                        c5 * (T20 * c6 - T21 * s6)
                        - T22 * s5;

                    double p13x =
                        d5 * (
                            s6 * (T00 * c1 + T10 * s1)
                            + c6 * (T01 * c1 + T11 * s1)
                        )
                        - d6 * (T02 * c1 + T12 * s1)
                        + T03 * c1
                        + T13 * s1;

                    double p13y =
                        T23
                        - d1
                        - d6 * T22
                        + d5 * (T21 * c6 + T20 * s6);

                    double c3 =
                        (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3)
                        / (2 * a2 * a3);

                    if (Math.Abs(c3) > 1.0)
                        continue;

                    c3 = Clamp(c3, -1.0, 1.0);

                    double arccos_q3 = Math.Acos(c3);

                    List<double> q3_list = new List<double>
                {
                    NormalizeAngle(arccos_q3),
                    NormalizeAngle(2 * PI - arccos_q3)
                };

                    foreach (double q3 in q3_list)
                    {
                        double s3 = Math.Sin(q3);

                        double A_val = a2 + a3 * c3;
                        double B_val = a3 * s3;

                        double q2 = Math.Atan2(
                            A_val * p13y - B_val * p13x,
                            A_val * p13x + B_val * p13y
                        );

                        q2 = NormalizeAngle(q2);

                        double c23 = Math.Cos(q2 + q3);
                        double s23 = Math.Sin(q2 + q3);

                        double q4 = Math.Atan2(
                            c23 * x04y - s23 * x04x,
                            x04x * c23 + x04y * s23
                        );

                        q4 = NormalizeAngle(q4);

                        solutions.Add(new double[]
                        {
                        q1, q2, q3, q4, q5, q6
                        });
                    }
                }
            }

            if (form == "rad")
            {
                return solutions;
            }
            else
            {
                List<double[]> solutionsInDegrees = new List<double[]>();

                foreach (double[] radArray in solutions)
                {
                    double[] angleArray = new double[radArray.Length];

                    for (int i = 0; i < radArray.Length; i++)
                    {
                        angleArray[i] = radArray[i] * 180.0 / Math.PI;
                    }

                    solutionsInDegrees.Add(angleArray);
                }

                return solutionsInDegrees;
            }
        }

        // =====================================================
        // 工具函数
        // =====================================================

        private static double NormalizeAngle(double angle)
        {
            while (angle < 0)
                angle += 2 * PI;

            while (angle >= 2 * PI)
                angle -= 2 * PI;

            return angle;
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
