using System;
using System.Globalization;

namespace WpfRobot.kinematics
{
    /// <summary>
    /// 欧拉角 RPY 与 UR 旋转向量 Rotation Vector 的转换工具类
    /// 
    /// 注意：
    /// 1. UR 的 p[x,y,z,rx,ry,rz] 中 rx, ry, rz 是旋转向量，单位 rad
    /// 2. 这里的 RPY 使用 ZYX 顺序：
    ///    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    /// 3. UI 中通常显示角度 deg，UR 发送时通常需要 rad
    /// </summary>
    public static class euler2vector
    {
        private const double Deg2Rad = Math.PI / 180.0;
        private const double Rad2Deg = 180.0 / Math.PI;
        private const double Eps = 1e-12;

        /// <summary>
        /// 角度制 RPY 转 UR 旋转向量
        /// 输入：rxDeg, ryDeg, rzDeg 单位为 degree
        /// 输出：double[3] = { rx, ry, rz }，单位为 rad，可用于 UR 的 p[x,y,z,rx,ry,rz]
        /// </summary>
        public static double[] RpyDegToRotationVectorRad(double rxDeg, double ryDeg, double rzDeg)
        {
            return RpyRadToRotationVectorRad(
                rxDeg * Deg2Rad,
                ryDeg * Deg2Rad,
                rzDeg * Deg2Rad
            );
        }

        /// <summary>
        /// 弧度制 RPY 转 UR 旋转向量
        /// 输入：rxRad, ryRad, rzRad 单位为 rad
        /// 输出：double[3] = { rx, ry, rz }，单位为 rad
        /// </summary>
        public static double[] RpyRadToRotationVectorRad(double rxRad, double ryRad, double rzRad)
        {
            // RPY 对应：
            // roll  = Rx
            // pitch = Ry
            // yaw   = Rz
            //
            // 旋转顺序：
            // R = Rz * Ry * Rx

            double cy = Math.Cos(rzRad * 0.5);
            double sy = Math.Sin(rzRad * 0.5);

            double cp = Math.Cos(ryRad * 0.5);
            double sp = Math.Sin(ryRad * 0.5);

            double cr = Math.Cos(rxRad * 0.5);
            double sr = Math.Sin(rxRad * 0.5);

            // 四元数 q = [w, x, y, z]
            double w = cr * cp * cy + sr * sp * sy;
            double x = sr * cp * cy - cr * sp * sy;
            double y = cr * sp * cy + sr * cp * sy;
            double z = cr * cp * sy - sr * sp * cy;

            return QuaternionToRotationVectorRad(x, y, z, w);
        }

        /// <summary>
        /// UR 旋转向量转角度制 RPY
        /// 输入：rx, ry, rz 为 UR rotation vector，单位 rad
        /// 输出：double[3] = { rollDeg, pitchDeg, yawDeg }，单位 degree
        /// </summary>
        public static double[] RotationVectorRadToRpyDeg(double rx, double ry, double rz)
        {
            double[] rpyRad = RotationVectorRadToRpyRad(rx, ry, rz);

            return new double[]
            {
                rpyRad[0] * Rad2Deg,
                rpyRad[1] * Rad2Deg,
                rpyRad[2] * Rad2Deg
            };
        }

        /// <summary>
        /// UR 旋转向量转弧度制 RPY
        /// 输入：rx, ry, rz 为 UR rotation vector，单位 rad
        /// 输出：double[3] = { rollRad, pitchRad, yawRad }，单位 rad
        /// </summary>
        public static double[] RotationVectorRadToRpyRad(double rx, double ry, double rz)
        {
            double[,] R = RotationVectorRadToMatrix(rx, ry, rz);

            // R = Rz(yaw) * Ry(pitch) * Rx(roll)
            //
            // pitch = atan2(-R[2,0], sqrt(R[0,0]^2 + R[1,0]^2))
            // yaw   = atan2(R[1,0], R[0,0])
            // roll  = atan2(R[2,1], R[2,2])

            double pitch = Math.Atan2(-R[2, 0], Math.Sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]));

            double roll;
            double yaw;

            double cosPitch = Math.Cos(pitch);

            if (Math.Abs(cosPitch) > 1e-8)
            {
                roll = Math.Atan2(R[2, 1], R[2, 2]);
                yaw = Math.Atan2(R[1, 0], R[0, 0]);
            }
            else
            {
                // 接近万向节锁时，令 yaw = 0
                yaw = 0.0;
                roll = Math.Atan2(-R[0, 1], R[1, 1]);
            }

            return new double[] { roll, pitch, yaw };
        }

        /// <summary>
        /// UR 旋转向量转 3x3 旋转矩阵
        /// 输入：rx, ry, rz 单位 rad
        /// </summary>
        public static double[,] RotationVectorRadToMatrix(double rx, double ry, double rz)
        {
            double theta = Math.Sqrt(rx * rx + ry * ry + rz * rz);

            double[,] R = new double[3, 3];

            if (theta < Eps)
            {
                R[0, 0] = 1.0;
                R[1, 1] = 1.0;
                R[2, 2] = 1.0;
                return R;
            }

            double kx = rx / theta;
            double ky = ry / theta;
            double kz = rz / theta;

            double c = Math.Cos(theta);
            double s = Math.Sin(theta);
            double v = 1.0 - c;

            R[0, 0] = kx * kx * v + c;
            R[0, 1] = kx * ky * v - kz * s;
            R[0, 2] = kx * kz * v + ky * s;

            R[1, 0] = ky * kx * v + kz * s;
            R[1, 1] = ky * ky * v + c;
            R[1, 2] = ky * kz * v - kx * s;

            R[2, 0] = kz * kx * v - ky * s;
            R[2, 1] = kz * ky * v + kx * s;
            R[2, 2] = kz * kz * v + c;

            return R;
        }

        /// <summary>
        /// RPY 弧度制转 4x4 齐次矩阵
        /// R = Rz * Ry * Rx
        /// </summary>
        public static double[,] RpyRadTo4x4Matrix(
            double x,
            double y,
            double z,
            double rxRad,
            double ryRad,
            double rzRad)
        {
            double cz = Math.Cos(rzRad);
            double sz = Math.Sin(rzRad);

            double cy = Math.Cos(ryRad);
            double sy = Math.Sin(ryRad);

            double cx = Math.Cos(rxRad);
            double sx = Math.Sin(rxRad);

            return new double[4, 4]
            {
                {
                    cz * cy,
                    cz * sy * sx - sz * cx,
                    cz * sy * cx + sz * sx,
                    x
                },
                {
                    sz * cy,
                    sz * sy * sx + cz * cx,
                    sz * sy * cx - cz * sx,
                    y
                },
                {
                    -sy,
                    cy * sx,
                    cy * cx,
                    z
                },
                {
                    0.0,
                    0.0,
                    0.0,
                    1.0
                }
            };
        }

        /// <summary>
        /// RPY 角度制转 4x4 齐次矩阵
        /// </summary>
        public static double[,] RpyDegTo4x4Matrix(
            double x,
            double y,
            double z,
            double rxDeg,
            double ryDeg,
            double rzDeg)
        {
            return RpyRadTo4x4Matrix(
                x,
                y,
                z,
                rxDeg * Deg2Rad,
                ryDeg * Deg2Rad,
                rzDeg * Deg2Rad
            );
        }

        /// <summary>
        /// 生成 URScript 可用的 p[x,y,z,rx,ry,rz]
        /// 输入位置 x,y,z 和 RPY 角度制姿态
        /// 输出姿态自动转成 UR rotation vector
        /// 
        /// 注意：
        /// URScript 中 x,y,z 通常单位是 m。
        /// 如果你的程序内部是 mm，发送前需要除以 1000。
        /// </summary>
        public static string ToUrPoseStringFromRpyDeg(
            double x,
            double y,
            double z,
            double rxDeg,
            double ryDeg,
            double rzDeg)
        {
            double[] rv = RpyDegToRotationVectorRad(rxDeg, ryDeg, rzDeg);

            return string.Format(
                CultureInfo.InvariantCulture,
                "p[{0},{1},{2},{3},{4},{5}]",
                x, y, z,
                rv[0], rv[1], rv[2]
            );
        }

        /// <summary>
        /// 四元数转 UR 旋转向量
        /// 输入四元数 x,y,z,w
        /// 输出 rotation vector，单位 rad
        /// </summary>
        public static double[] QuaternionToRotationVectorRad(double x, double y, double z, double w)
        {
            // 归一化，避免数值误差
            double norm = Math.Sqrt(x * x + y * y + z * z + w * w);

            if (norm < Eps)
            {
                return new double[] { 0.0, 0.0, 0.0 };
            }

            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;

            // 避免 Acos 输入略微越界
            w = Clamp(w, -1.0, 1.0);

            double angle = 2.0 * Math.Acos(w);
            double s = Math.Sqrt(1.0 - w * w);

            if (s < Eps || Math.Abs(angle) < Eps)
            {
                return new double[] { 0.0, 0.0, 0.0 };
            }

            double axisX = x / s;
            double axisY = y / s;
            double axisZ = z / s;

            return new double[]
            {
                axisX * angle,
                axisY * angle,
                axisZ * angle
            };
        }

        /// <summary>
        /// UR 旋转向量转四元数
        /// 输出 x,y,z,w
        /// </summary>
        public static void RotationVectorRadToQuaternion(
            double rx,
            double ry,
            double rz,
            out double x,
            out double y,
            out double z,
            out double w)
        {
            double angle = Math.Sqrt(rx * rx + ry * ry + rz * rz);

            if (angle < Eps)
            {
                x = 0.0;
                y = 0.0;
                z = 0.0;
                w = 1.0;
                return;
            }

            double axisX = rx / angle;
            double axisY = ry / angle;
            double axisZ = rz / angle;

            double half = angle * 0.5;
            double sinHalf = Math.Sin(half);

            x = axisX * sinHalf;
            y = axisY * sinHalf;
            z = axisZ * sinHalf;
            w = Math.Cos(half);
        }

        private static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        /// <summary>
        /// 4x4 齐次矩阵转 UR 旋转向量
        /// 输入 T 为 4x4 齐次变换矩阵
        /// 输出 double[3] = { rx, ry, rz }，单位 rad
        /// 
        /// 注意：
        /// 只使用 T 左上角 3x3 旋转矩阵，不使用平移部分。
        /// </summary>
        public static double[] Matrix4x4ToRotationVectorRad(double[,] T)
        {
            if (T == null || T.GetLength(0) < 4 || T.GetLength(1) < 4)
                throw new ArgumentException("Matrix4x4ToRotationVectorRad 需要 4x4 矩阵。");

            double[,] R = new double[3, 3]
            {
                { T[0, 0], T[0, 1], T[0, 2] },
                { T[1, 0], T[1, 1], T[1, 2] },
                { T[2, 0], T[2, 1], T[2, 2] }
            };

            return RotationMatrixToRotationVectorRad(R);
        }

        /// <summary>
        /// 3x3 旋转矩阵转 UR 旋转向量
        /// 输出 double[3] = { rx, ry, rz }，单位 rad
        /// </summary>
        public static double[] RotationMatrixToRotationVectorRad(double[,] R)
        {
            if (R == null || R.GetLength(0) < 3 || R.GetLength(1) < 3)
                throw new ArgumentException("RotationMatrixToRotationVectorRad 需要 3x3 矩阵。");

            double axisX, axisY, axisZ;

            double trace = R[0, 0] + R[1, 1] + R[2, 2];

            double cosTheta = (trace - 1.0) * 0.5;
            cosTheta = Clamp(cosTheta, -1.0, 1.0);

            double theta = Math.Acos(cosTheta);

            // 接近 0 度旋转
            if (theta < 1e-12)
            {
                return new double[] { 0.0, 0.0, 0.0 };
            }

            // 接近 180 度旋转时，sin(theta) 接近 0，普通公式不稳定
            if (Math.Abs(Math.PI - theta) < 1e-6)
            {
                double xx = (R[0, 0] + 1.0) * 0.5;
                double yy = (R[1, 1] + 1.0) * 0.5;
                double zz = (R[2, 2] + 1.0) * 0.5;

                double xy = (R[0, 1] + R[1, 0]) * 0.25;
                double xz = (R[0, 2] + R[2, 0]) * 0.25;
                double yz = (R[1, 2] + R[2, 1]) * 0.25;


                if (xx >= yy && xx >= zz)
                {
                    axisX = Math.Sqrt(Math.Max(xx, 0.0));
                    axisY = Math.Abs(axisX) > Eps ? xy / axisX : 0.0;
                    axisZ = Math.Abs(axisX) > Eps ? xz / axisX : 0.0;
                }
                else if (yy >= xx && yy >= zz)
                {
                    axisY = Math.Sqrt(Math.Max(yy, 0.0));
                    axisX = Math.Abs(axisY) > Eps ? xy / axisY : 0.0;
                    axisZ = Math.Abs(axisY) > Eps ? yz / axisY : 0.0;
                }
                else
                {
                    axisZ = Math.Sqrt(Math.Max(zz, 0.0));
                    axisX = Math.Abs(axisZ) > Eps ? xz / axisZ : 0.0;
                    axisY = Math.Abs(axisZ) > Eps ? yz / axisZ : 0.0;
                }

                return new double[]
                {
                    axisX * theta,
                    axisY * theta,
                    axisZ * theta
                };
            }

            // 普通情况
            double sinTheta = Math.Sin(theta);

            axisX = (R[2, 1] - R[1, 2]) / (2.0 * sinTheta);
            axisY = (R[0, 2] - R[2, 0]) / (2.0 * sinTheta);
            axisZ = (R[1, 0] - R[0, 1]) / (2.0 * sinTheta);

            return new double[]
            {
                axisX * theta,
                axisY * theta,
                axisZ * theta
            };
        }

        /// <summary>
        /// 4x4 齐次矩阵转 UR 位姿数组
        /// 输出 double[6] = { x, y, z, rx, ry, rz }
        /// 
        /// x,y,z 直接取矩阵最后一列。
        /// rx,ry,rz 是 UR rotation vector，单位 rad。
        /// </summary>
        public static double[] Matrix4x4ToUrPoseArray(double[,] T)
        {
            if (T == null || T.GetLength(0) < 4 || T.GetLength(1) < 4)
                throw new ArgumentException("Matrix4x4ToUrPoseArray 需要 4x4 矩阵。");

            double[] rv = Matrix4x4ToRotationVectorRad(T);

            return new double[]
            {
                T[0, 3],
                T[1, 3],
                T[2, 3],
                rv[0],
                rv[1],
                rv[2]
            };
        }

        /// <summary>
        /// 4x4 齐次矩阵转 URScript 可用的 p[x,y,z,rx,ry,rz]
        /// </summary>
        public static string Matrix4x4ToUrPoseString(double[,] T)
        {
            double[] pose = Matrix4x4ToUrPoseArray(T);

            return string.Format(
                CultureInfo.InvariantCulture,
                "p[{0},{1},{2},{3},{4},{5}]",
                pose[0], pose[1], pose[2],
                pose[3], pose[4], pose[5]
            );
        }
    }
}