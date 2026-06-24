using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace WpfRobot.kinematics
{
    public class PoseTransform
    {
        /// <summary>
        ///  Pose(6 向量：[x,y,z, rx,ry,rz]) → 4x4变换矩阵
        /// </summary>
        /// <param name="pose"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public static Matrix<double> PoseToTransform(double[] pose)
        {
            if (pose.Length != 6) throw new ArgumentException();
            var t = DenseMatrix.CreateIdentity(4);
            t[0, 3] = pose[0];
            t[1, 3] = pose[1];
            t[2, 3] = pose[2];

            var R = PoseTransform.AxisAngleToRotation(pose[3], pose[4], pose[5]);
            //t.SetSubMatrix(0, 3, R);
            t.SetSubMatrix(0, 3, 0, 3, R);

            return t;
        }

        /// <summary>
        /// rx ry rz是旋转向量
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="rx"></param>
        /// <param name="ry"></param>
        /// <param name="rz"></param>
        /// <returns></returns>
        public static Matrix<double> PoseToTransform(double x, double y, double z, double rx, double ry, double rz)
        {
            double theta = Math.Sqrt(rx * rx + ry * ry + rz * rz);

            if (theta < 1e-8)
            {
                var T_id = DenseMatrix.CreateIdentity(4);
                T_id[0, 3] = x;
                T_id[1, 3] = y;
                T_id[2, 3] = z;
                return T_id;
            }

            double ux = rx / theta;
            double uy = ry / theta;
            double uz = rz / theta;
            double c = Math.Cos(theta);
            double s = Math.Sin(theta);
            double oneMinusC = 1 - c;

            double[,] T = new double[4, 4];

            T[0, 0] = c + ux * ux * oneMinusC;
            T[0, 1] = ux * uy * oneMinusC - uz * s;
            T[0, 2] = ux * uz * oneMinusC + uy * s;
            T[0, 3] = x;

            T[1, 0] = uy * ux * oneMinusC + uz * s;
            T[1, 1] = c + uy * uy * oneMinusC;
            T[1, 2] = uy * uz * oneMinusC - ux * s;
            T[1, 3] = y;

            T[2, 0] = uz * ux * oneMinusC - uy * s;
            T[2, 1] = uz * uy * oneMinusC + ux * s;
            T[2, 2] = c + uz * uz * oneMinusC;
            T[2, 3] = z;

            T[3, 0] = 0;
            T[3, 1] = 0;
            T[3, 2] = 0;
            T[3, 3] = 1;

            return DenseMatrix.OfArray(T);
        }

        /// <summary>
        /// 4x4变换矩阵 → 6位旋转向量格式Pose(x,y,z,rx,ry,rz)
        /// </summary>
        /// <param name="T"></param>
        /// <returns></returns>
        public static double[] TransformToPose(Matrix<double> T)
        {
            var pose = new double[6];
            pose[0] = T[0, 3];
            pose[1] = T[1, 3];
            pose[2] = T[2, 3];

            var R = T.SubMatrix(0, 3, 0, 3);
            var aa = PoseTransform.RotationToAxisAngle(R);
            pose[3] = aa[0];
            pose[4] = aa[1];
            pose[5] = aa[2];
            return pose;
        }

        /// <summary>
        /// 输入3x3旋转矩阵，输出旋转向量
        /// </summary>
        /// <param name="R"></param>
        /// <param name="rotationToAngle"></param>
        /// <returns></returns>
        public static double[] RotationToAxisAngle(Matrix<double> R, bool rotationToAngle = false)
        {
            double trace = R[0, 0] + R[1, 1] + R[2, 2];
            double cos_theta = Math.Max(-1.0, Math.Min(1.0, (trace - 1) / 2));
            double theta = Math.Acos(cos_theta);

            if (Math.Abs(theta) < 1e-8)
                return new double[] { 0, 0, 0 };

            double rx, ry, rz;

            if (Math.Abs(theta - Math.PI) < 1e-4)
            {
                // θ ≈ π 特殊处理
                //double halfInverse = 0;
                if (R[0, 0] >= R[1, 1] && R[0, 0] >= R[2, 2])
                {
                    rx = Math.Sqrt(Math.Max(0, (R[0, 0] + 1) / 2));
                    ry = R[0, 1] / (2 * rx);
                    rz = R[0, 2] / (2 * rx);
                }
                else if (R[1, 1] >= R[2, 2])
                {
                    ry = Math.Sqrt(Math.Max(0, (R[1, 1] + 1) / 2));
                    rx = R[0, 1] / (2 * ry);
                    rz = R[1, 2] / (2 * ry);
                }
                else
                {
                    rz = Math.Sqrt(Math.Max(0, (R[2, 2] + 1) / 2));
                    rx = R[0, 2] / (2 * rz);
                    ry = R[1, 2] / (2 * rz);
                }

                rx *= theta;
                ry *= theta;
                rz *= theta;
            }
            else
            {
                double sin_theta = Math.Sin(theta);
                double multiplier = theta / (2 * sin_theta);

                rx = (R[2, 1] - R[1, 2]) * multiplier;
                ry = (R[0, 2] - R[2, 0]) * multiplier;
                rz = (R[1, 0] - R[0, 1]) * multiplier;
            }

            if (rotationToAngle)
            {
                rx *= 180.0 / Math.PI;
                ry *= 180.0 / Math.PI;
                rz *= 180.0 / Math.PI;
            }

            return new[] { rx, ry, rz };
        }

        /// <summary>
        /// 向量3:[rx, ry, rz]Axis‑Angle → 3x3 RotationMatrix
        /// </summary>
        /// <param name="rx"></param>
        /// <param name="ry"></param>
        /// <param name="rz"></param>
        /// <returns></returns>
        public static Matrix<double> AxisAngleToRotation(double rx, double ry, double rz)
        {
            var theta = Math.Sqrt(rx * rx + ry * ry + rz * rz);
            if (theta < 1e-8) return DenseMatrix.CreateIdentity(3);

            var ux = rx / theta;
            var uy = ry / theta;
            var uz = rz / theta;
            var c = Math.Cos(theta);
            var s = Math.Sin(theta);

            return DenseMatrix.OfArray(new double[,]
            {
                { c + ux*ux*(1-c),    ux*uy*(1-c) - uz*s, ux*uz*(1-c) + uy*s },
                { uy*ux*(1-c) + uz*s, c + uy*uy*(1-c),    uy*uz*(1-c) - ux*s },
                { uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s, c + uz*uz*(1-c)    }
            });
        }


        /// <summary>
        /// 旋转向量转换为欧拉角，ZYX 顺序（Yaw-Pitch-Roll）,输出为弧度制
        /// </summary>
        /// <param name="axisAngle"></param>
        /// <returns></returns>
        public static double[] AxisAngleToEuler(double[] axis_Angle)
        {
            var axisAngle = Vector<double>.Build.DenseOfArray(axis_Angle);
            double theta = axisAngle.L2Norm();
            if (theta < 1e-8)
                return new double[] { 0, 0, 0 };

            var k = axisAngle / theta;
            var K = DenseMatrix.OfArray(new double[,]
            {
                { 0, -k[2], k[1] },
                { k[2], 0, -k[0] },
                { -k[1], k[0], 0 }
            });

            var R = DenseMatrix.CreateIdentity(3) + Math.Sin(theta) * K + (1 - Math.Cos(theta)) * (K * K);

            double roll = Math.Atan2(R[2, 1], R[2, 2]);
            double pitch = Math.Atan2(-R[2, 0], Math.Sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2]));
            double yaw = Math.Atan2(R[1, 0], R[0, 0]);

            return new double[] { roll, pitch, yaw };
        }

        /// <summary>
        /// 欧拉角 (roll, pitch, yaw, ZYX) 转旋转向量 (axis-angle)
        /// </summary>
        public static double[] EulerToAxisAngle(double roll, double pitch, double yaw)
        {
            // 构造 ZYX 顺序的旋转矩阵
            double cr = Math.Cos(roll), sr = Math.Sin(roll);
            double cp = Math.Cos(pitch), sp = Math.Sin(pitch);
            double cy = Math.Cos(yaw), sy = Math.Sin(yaw);

            var R = DenseMatrix.OfArray(new double[,]
            {
                { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
                { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
                { -sp,     cp * sr,                cp * cr }
            });

            double trace = R.Trace();
            double theta = Math.Acos(Math.Min(1.0, Math.Max(-1.0, (trace - 1) / 2)));

            if (Math.Abs(theta) < 1e-8)
                return new double[] { 0, 0, 0 };

            double denom = 2 * Math.Sin(theta);

            double kx = (R[2, 1] - R[1, 2]) / denom;
            double ky = (R[0, 2] - R[2, 0]) / denom;
            double kz = (R[1, 0] - R[0, 1]) / denom;

            var axis = new double[] { kx * theta, ky * theta, kz * theta };
            return axis;
        }

        /// <summary>
        /// 末端关节轴指向矢量 → 4x4 Transform; currentJointsDeg(角度值); string type = "Transform"
        /// </summary>
        /// <param name="pE">末端参考点坐标（3×1 向量）</param>
        /// <param name="pT">空间目标点坐标（3×1 向量）</param>
        /// <param name="currentJointsDeg">当前关节角度（6 元素，度）</param>
        /// <returns></returns>
        public static Matrix<double> EndEffectorVectorToTransform(double[] pE, double[] pT, double[] currentJointsDeg)
        {
            var v = Vector<double>.Build.DenseOfArray(pT) - Vector<double>.Build.DenseOfArray(pE);
            var zDesired = v.Normalize(2);

            var poseCurr = Forward.ForwardKinematics(currentJointsDeg.Select(d => d * Math.PI / 180.0).ToArray());
            // poseCurr = [x, y, z, rx, ry, rz]，其中 rx,ry,rz 为轴角（rad）
            var Tcurr = PoseTransform.PoseToTransform(poseCurr);
            var Rcurr = Tcurr.SubMatrix(0, 3, 0, 3);
            var zCurr = Rcurr * Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 1 });

            var k = zCurr.CrossProduct(zDesired);
            if (k.L2Norm() < 1e-6)
            {
                // 已近似对齐，无需额外旋转
                k = Vector<double>.Build.DenseOfArray(new double[] { 1, 0, 0 });
            }
            else
            {
                k = k.Normalize(2);
            }
            var cosTheta = Math.Max(-1.0, Math.Min(1.0, zCurr.DotProduct(zDesired)));
            var theta = Math.Acos(cosTheta);

            var K = DenseMatrix.OfArray(new double[,]
            {
                { 0,       -k[2],   k[1] },
                { k[2],     0,     -k[0] },
                { -k[1],   k[0],    0    }
            });

            Matrix<double> Ralign = DenseMatrix.CreateIdentity(3)
                          + Math.Sin(theta) * K
                          + (1 - Math.Cos(theta)) * (K * K);

            // 5. 得到新的末端全局旋转
            Matrix<double> R_new = Ralign * Rcurr;

            Matrix<double> Ttarget = DenseMatrix.CreateIdentity(4);
            Ttarget.SetSubMatrix(0, 3, 0, 3, R_new);
            Ttarget[0, 3] = pE[0];
            Ttarget[1, 3] = pE[1];
            Ttarget[2, 3] = pE[2];

            return Ttarget;
        }


        /// <summary>
        /// 在知道末端工具上顶端点toolEnd、杆上任意 toolMiddlePoint 点两点(RCM), 以及杆长, 推算末端关节位置。
        /// 为什么说是杆上任意一点，因为RCM 点是可变的。
        /// toolMiddlePoint 在这里是 RCM 点
        /// </summary>
        /// <param name="currentJointsDeg">当前关节角度（6 元素，度）</param>
        /// <returns></returns>
        public static Matrix<double> AlignRobotToTool(double[] toolMiddlePoint, double[] toolEnd, double Length, double[] currentJointsDeg)
        {
            var v = Vector<double>.Build.DenseOfArray(toolEnd) - Vector<double>.Build.DenseOfArray(toolMiddlePoint);
            var zDesired = v.Normalize(2);

            var poseCurr = Forward.ForwardKinematics(currentJointsDeg.Select(d => d * Math.PI / 180.0).ToArray());
            // poseCurr = [x, y, z, rx, ry, rz]，其中 rx,ry,rz 为轴角（rad）
            var Tcurr = PoseTransform.PoseToTransform(poseCurr);
            var Rcurr = Tcurr.SubMatrix(0, 3, 0, 3);
            var zCurr = Rcurr * Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 1 });

            var k = zCurr.CrossProduct(zDesired);
            if (k.L2Norm() < 1e-6)
            {
                // 已近似对齐，无需额外旋转
                k = Vector<double>.Build.DenseOfArray(new double[] { 1, 0, 0 });
            }
            else
            {
                k = k.Normalize(2);
            }
            var cosTheta = Math.Max(-1.0, Math.Min(1.0, zCurr.DotProduct(zDesired)));
            var theta = Math.Acos(cosTheta);

            var K = DenseMatrix.OfArray(new double[,]
            {
                { 0,       -k[2],   k[1] },
                { k[2],     0,     -k[0] },
                { -k[1],   k[0],    0    }
            });

            Matrix<double> Ralign = DenseMatrix.CreateIdentity(3)
                          + Math.Sin(theta) * K
                          + (1 - Math.Cos(theta)) * (K * K);

            // 5. 得到新的末端全局旋转
            Matrix<double> R_new = Ralign * Rcurr;

            //6.计算机器人末端关节期望位置
            double[] RobiotPosition = ComputeEEPostionm(toolMiddlePoint, toolEnd, Length);

            Matrix<double> Ttarget = DenseMatrix.CreateIdentity(4);
            Ttarget.SetSubMatrix(0, 3, 0, 3, R_new);
            Ttarget[0, 3] = RobiotPosition[0];
            Ttarget[1, 3] = RobiotPosition[1];
            Ttarget[2, 3] = RobiotPosition[2];

            return Ttarget;
        }

        /// <summary>
        /// 在知道末端工具上顶端点A、杆上任意B点两点, 以及杆长, 推算末端关节位置。
        /// B在这里是 RCM 点
        /// </summary>
        /// <returns></returns>
        public static double[] ComputeEEPostionm(double[] B, double[] A, double Length)
        {
            double[] v = new double[3];  // 向量 AB
            for (int i = 0; i < 3; i++)
                v[i] = A[i] - B[i];

            // 计算向量长度（范数）
            double norm = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

            // 单位方向向量
            double[] unitV = new double[3];
            for (int i = 0; i < 3; i++)
                unitV[i] = v[i] / norm;

            // 计算 C = A - Length * unitV
            double[] EndResult = new double[3];
            for (int i = 0; i < 3; i++)
                EndResult[i] = A[i] - Length * unitV[i];

            return EndResult;
        }


        /// <summary>
        /// 末端关节轴指向矢量 → 1x6 向量Pose; currentJointsDeg(角度值), string type = "AxisAngle"
        /// </summary>
        /// <param name="pE">末端参考点坐标（3×1 向量）</param>
        /// <param name="pT">空间目标点坐标（3×1 向量）</param>
        /// <param name="currentJointsDeg">当前关节角度（6 元素，度）</param>
        /// <returns></returns>
        public static double[] EndEffectorVectorToAxisAngle(double[] pE, double[] pT, double[] currentJointsDeg)
        {
            // 1. 计算连线向量并归一化为期望 Z 轴方向
            var v = Vector<double>.Build.DenseOfArray(pT) - Vector<double>.Build.DenseOfArray(pE);
            var zDesired = v.Normalize(2);

            // 2. 取当前末端姿态并提取全局 Z 轴方向
            var poseCurr = Forward.ForwardKinematics(currentJointsDeg.Select(d => d * Math.PI / 180.0).ToArray());
            // poseCurr = [x, y, z, rx, ry, rz]，其中 rx,ry,rz 为轴角（rad）
            var Tcurr = PoseTransform.PoseToTransform(poseCurr);
            var Rcurr = Tcurr.SubMatrix(0, 3, 0, 3);
            // 当前全局 Z 轴方向：
            var zCurr = Rcurr * Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 1 });

            // 3. 计算旋转轴 k = zCurr × zDesired，及旋转角 theta
            var k = zCurr.CrossProduct(zDesired);
            if (k.L2Norm() < 1e-6)
            {
                // 已近似对齐，无需额外旋转
                k = Vector<double>.Build.DenseOfArray(new double[] { 1, 0, 0 });
            }
            else
            {
                k = k.Normalize(2);
            }
            var cosTheta = Math.Max(-1.0, Math.Min(1.0, zCurr.DotProduct(zDesired)));
            var theta = Math.Acos(cosTheta);

            // 4. Rodrigues 公式生成对齐旋转矩阵 Ralign
            var K = DenseMatrix.OfArray(new double[,]
            {
                { 0,       -k[2],   k[1] },
                { k[2],     0,     -k[0] },
                { -k[1],   k[0],    0    }
            });

            Matrix<double> Ralign = DenseMatrix.CreateIdentity(3)
                          + Math.Sin(theta) * K
                          + (1 - Math.Cos(theta)) * (K * K);

            // 5. 得到新的末端全局旋转
            Matrix<double> R_new = Ralign * Rcurr;

            // 旋转矩阵转换为旋转向量
            double[] RotationVector = PoseTransform.RotationToAxisAngle(R_new);

            double[] returnPose = new double[6] { pE[0], pE[1], pE[2], RotationVector[0], RotationVector[1], RotationVector[2] };

            return returnPose;
        }


        /// <summary>
        /// 末端关节轴指向矢量 → 1x3 旋转向量; currentJointsDeg(角度值), string type = "Euler"
        /// </summary>
        /// <param name="pE">末端参考点坐标（3×1 向量）</param>
        /// <param name="pT">空间目标点坐标（3×1 向量）</param>
        /// <param name="currentJointsDeg">当前关节角度（6 元素，度）</param>
        /// <returns></returns>
        public static double[] EndEffectorVectorToEuler(double[] pE, double[] pT, double[] currentJointsDeg)
        {
            var v = Vector<double>.Build.DenseOfArray(pT) - Vector<double>.Build.DenseOfArray(pE);
            var zDesired = v.Normalize(2);

            var poseCurr = Forward.ForwardKinematics(currentJointsDeg.Select(d => d * Math.PI / 180.0).ToArray());
            // poseCurr = [x, y, z, rx, ry, rz]，其中 rx,ry,rz 为轴角（rad）
            var Tcurr = PoseTransform.PoseToTransform(poseCurr);
            var Rcurr = Tcurr.SubMatrix(0, 3, 0, 3);
            var zCurr = Rcurr * Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 1 });

            var k = zCurr.CrossProduct(zDesired);
            if (k.L2Norm() < 1e-6)
            {
                // 已近似对齐，无需额外旋转
                k = Vector<double>.Build.DenseOfArray(new double[] { 1, 0, 0 });
            }
            else
            {
                k = k.Normalize(2);
            }
            var cosTheta = Math.Max(-1.0, Math.Min(1.0, zCurr.DotProduct(zDesired)));
            var theta = Math.Acos(cosTheta);

            var K = DenseMatrix.OfArray(new double[,]
            {
                { 0,       -k[2],   k[1] },
                { k[2],     0,     -k[0] },
                { -k[1],   k[0],    0    }
            });

            Matrix<double> Ralign = DenseMatrix.CreateIdentity(3)
                          + Math.Sin(theta) * K
                          + (1 - Math.Cos(theta)) * (K * K);

            Matrix<double> R_new = Ralign * Rcurr;

            double[] RotationVector = PoseTransform.RotationToAxisAngle(R_new);

            double[] AxisAngleToEuler = PoseTransform.AxisAngleToEuler(RotationVector);

            double[] returnPose = new double[3] { AxisAngleToEuler[0], AxisAngleToEuler[1], AxisAngleToEuler[2] };

            //double[] ax = EulerToAxisAngle(AxisAngleToEuler[0], AxisAngleToEuler[1], AxisAngleToEuler[2]);

            //double[] AEuler = PoseTransform.AxisAngleToEuler(ax);

            return returnPose;
        }
    }

}
