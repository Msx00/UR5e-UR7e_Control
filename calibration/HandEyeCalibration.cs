
using System;
using System.Collections.Generic;
//using System.Numerics;
using System.Linq;
using MathNet.Numerics; // 用于 Quaternion
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;

public static class HandEyeCalibration_NDI
{
    private static readonly MatrixBuilder<double> M = Matrix<double>.Build;
    private static readonly VectorBuilder<double> V = Vector<double>.Build;

    /// <summary>
    /// 求解手眼标定 (Eye-in-Hand)。
    /// 求解方程 X * B_rel = A_rel * X, 其中 X = T^{ndi}_{tool}
    /// </summary>
    /// <param name="robotPoses_A">N 组 T^{tool}_{base} 矩阵列表 (机械臂末端位姿)</param>
    /// <param name="trackerPoses_B">N 组 T^{ndi}_{world} 矩阵列表 (NDI 刚体位姿)</param>
    /// <returns>标定结果 X = T^{ndi}_{tool} 矩阵 (4x4)</returns>
    public static Matrix<double> SolveHandEye(List<Matrix<double>> robotPoses_A, List<Matrix<double>> trackerPoses_B)
    {
        if (robotPoses_A == null || trackerPoses_B == null || robotPoses_A.Count != trackerPoses_B.Count)
        {
            throw new ArgumentException("输入列表必须大小相同且不为空。");
        }
        if (robotPoses_A.Count < 3)
        {
            throw new ArgumentException("至少需要3组位姿数据来进行标定（以产生2组相对运动）。");
        }

        int numMotions = robotPoses_A.Count - 1;
        var R_A_list = new List<Matrix<double>>(numMotions);
        var t_A_list = new List<Vector<double>>(numMotions);
        var R_B_list = new List<Matrix<double>>(numMotions);
        var t_B_list = new List<Vector<double>>(numMotions);

        // 1. 计算所有相对运动
        for (int i = 0; i < numMotions; i++)
        {
            try
            {
                var A_i_inv = robotPoses_A[i].Inverse();
                var A_rel = A_i_inv * robotPoses_A[i + 1]; // A_rel = A_i^{-1} * A_{i+1}

                var B_i_inv = trackerPoses_B[i].Inverse();
                var B_rel = B_i_inv * trackerPoses_B[i + 1]; // B_rel = B_i^{-1} * B_{i+1}

                // 提取 R 和 t
                R_A_list.Add(A_rel.SubMatrix(0, 3, 0, 3));
                t_A_list.Add(A_rel.SubMatrix(0, 3, 3, 1).Column(0));
                R_B_list.Add(B_rel.SubMatrix(0, 3, 0, 3));
                t_B_list.Add(B_rel.SubMatrix(0, 3, 3, 1).Column(0));
            }
            catch (Exception ex)
            {
                Console.WriteLine($"警告: 计算第 {i} 组相对运动时出错 (可能是矩阵奇异): {ex.Message}");
            }
        }

        if (R_A_list.Count < 2)
        {
            throw new InvalidOperationException("没有足够的有效相对运动数据对（至少需要2对）。");
        }

        // 2. 求解旋转 R_X
        Matrix<double> R_X = SolveRotation(R_A_list, R_B_list);

        // 3. 求解平移 t_X
        Vector<double> t_X = SolveTranslation(R_A_list, t_A_list, R_B_list, t_B_list, R_X);

        // 4. 组合成 T^{ndi}_{tool} (4x4 矩阵)
        return BuildTransformationMatrix(R_X, t_X);
    }

    /// <summary>
    /// 步骤 1: 求解旋转 R_X
    /// 求解 R_A * R_X = R_X * R_B 的超定方程组
    /// 我们使用四元数方法将其转换为 M*q_X = 0，并使用 SVD 求解
    /// </summary>
    private static Matrix<double> SolveRotation(List<Matrix<double>> R_A, List<Matrix<double>> R_B)
    {
        int N = R_A.Count;
        // M_total 是 (4*N) x 4 矩阵
        var M_total = M.Dense(4 * N, 4);

        for (int i = 0; i < N; i++)
        {
            var q_A = RotationMatrixToQuaternion(R_A[i]);
            var q_B = RotationMatrixToQuaternion(R_B[i]);

            // M_i = Q_L(q_A) - Q_R(q_B)
            var M_i = BuildQuaternionMatrix(q_A, q_B);
            M_total.SetSubMatrix(4 * i, 0, M_i);
        }

        // 使用 SVD 求解 M_total * q_X = 0
        // 我们需要 V^T 矩阵
        var svd = M_total.Svd(true);

        // q_X 是 V^T 的最后一行（对应最小奇异值）
        var q_X_vec = svd.VT.Row(3);

        // MathNet.Numerics.Quaternion 构造函数是 (w, x, y, z)
        var q_X = new QuaternionSelf(q_X_vec[0], q_X_vec[1], q_X_vec[2], q_X_vec[3]);

        return q_X.Normalize().ToRotationMatrix();
    }

    /// <summary>
    /// 步骤 2: 求解平移 t_X
    /// 求解 (R_A - I) * t_X = R_X * t_B - t_A 的超定线性方程组
    /// 形式为 C * t_X = d
    /// </summary>
    private static Vector<double> SolveTranslation(
        List<Matrix<double>> R_A_list, List<Vector<double>> t_A_list,
        List<Matrix<double>> R_B_list, List<Vector<double>> t_B_list,
        Matrix<double> R_X)
    {
        int N = R_A_list.Count;
        var I = M.DenseIdentity(3, 3);

        // C_total 是 (3*N) x 3 矩阵
        var C_total = M.Dense(3 * N, 3);
        // d_total 是 (3*N) x 1 向量
        var d_total = V.Dense(3 * N);

        for (int i = 0; i < N; i++)
        {
            var C_i = R_A_list[i] - I;
            var d_i = (R_X * t_B_list[i]) - t_A_list[i];

            C_total.SetSubMatrix(3 * i, 0, C_i);
            d_total.SetSubVector(3 * i, 3, d_i);
        }

        // 使用最小二乘法求解 C_total * t_X = d_total
        // MathNet.Numerics 的 Solve 函数自动处理超定系统
        var t_X = C_total.Solve(d_total);
        return t_X;
    }


    /// <summary>
    /// 构造 M_i = Q_L(q_A) - Q_R(q_B)
    /// </summary>
    private static Matrix<double> BuildQuaternionMatrix(QuaternionSelf q_A, QuaternionSelf q_B)
    {
        double wA = q_A.W, xA = q_A.X, yA = q_A.Y, zA = q_A.Z;
        double wB = q_B.W, xB = q_B.X, yB = q_B.Y, zB = q_B.Z;

        var M_i = M.Dense(4, 4);

        // [Q_L(q_A) - Q_R(q_B)]
        M_i[0, 0] = wA - wB; M_i[0, 1] = -xA + xB; M_i[0, 2] = -yA + yB; M_i[0, 3] = -zA + zB;
        M_i[1, 0] = xA - xB; M_i[1, 1] = wA - wB; M_i[1, 2] = -zA - zB; M_i[1, 3] = yA + yB;
        M_i[2, 0] = yA - yB; M_i[2, 1] = zA + zB; M_i[2, 2] = wA - wB; M_i[2, 3] = -xA - xB;
        M_i[3, 0] = zA - zB; M_i[3, 1] = -yA - yB; M_i[3, 2] = xA + xB; M_i[3, 3] = wA - wB;

        return M_i;
    }

    /// <summary>
    /// 将 3x3 旋转矩阵转换为四元数 (W, X, Y, Z)
    /// MathNet.Numerics.Quaternion 没有 R -> Q 的直接构造函数
    /// </summary>
    private static QuaternionSelf RotationMatrixToQuaternion(Matrix<double> R)
    {
        double w, x, y, z;
        double tr = R.Trace(); // 矩阵的迹 (R[0,0] + R[1,1] + R[2,2])

        if (tr > 0)
        {
            double S = Math.Sqrt(tr + 1.0) * 2;
            w = 0.25 * S;
            x = (R[2, 1] - R[1, 2]) / S;
            y = (R[0, 2] - R[2, 0]) / S;
            z = (R[1, 0] - R[0, 1]) / S;
        }
        else if ((R[0, 0] > R[1, 1]) && (R[0, 0] > R[2, 2]))
        {
            double S = Math.Sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2;
            w = (R[2, 1] - R[1, 2]) / S;
            x = 0.25 * S;
            y = (R[0, 1] + R[1, 0]) / S;
            z = (R[0, 2] + R[2, 0]) / S;
        }
        else if (R[1, 1] > R[2, 2])
        {
            double S = Math.Sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2;
            w = (R[0, 2] - R[2, 0]) / S;
            x = (R[0, 1] + R[1, 0]) / S;
            y = 0.25 * S;
            z = (R[1, 2] + R[2, 1]) / S;
        }
        else
        {
            double S = Math.Sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2;
            w = (R[1, 0] - R[0, 1]) / S;
            x = (R[0, 2] + R[2, 0]) / S;
            y = (R[1, 2] + R[2, 1]) / S;
            z = 0.25 * S;
        }

        return new QuaternionSelf(w, x, y, z).Normalize();
    }

    /// <summary>
    /// 将 R (3x3) 和 t (3x1) 组合成一个 4x4 刚性变换矩阵
    /// </summary>
    private static Matrix<double> BuildTransformationMatrix(Matrix<double> R, Vector<double> t)
    {
        var T = M.DenseIdentity(4, 4);
        T.SetSubMatrix(0, 0, R);
        T.SetSubMatrix(0, 3, t.ToColumnMatrix());
        return T;
    }
}
public class QuaternionSelf
{
    public double W { get; set; }
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }

    public QuaternionSelf(double w, double x, double y, double z)
    {
        W = w;
        X = x;
        Y = y;
        Z = z;
    }

    // 四元数归一化
    public QuaternionSelf Normalize()
    {
        double norm = Math.Sqrt(W * W + X * X + Y * Y + Z * Z);
        return new QuaternionSelf(W / norm, X / norm, Y / norm, Z / norm);
    }

    // 四元数乘法
    public static QuaternionSelf Multiply(QuaternionSelf q1, QuaternionSelf q2)
    {
        double w = q1.W * q2.W - q1.X * q2.X - q1.Y * q2.Y - q1.Z * q2.Z;
        double x = q1.W * q2.X + q1.X * q2.W + q1.Y * q2.Z - q1.Z * q2.Y;
        double y = q1.W * q2.Y - q1.X * q2.Z + q1.Y * q2.W + q1.Z * q2.X;
        double z = q1.W * q2.Z + q1.X * q2.Y - q1.Y * q2.X + q1.Z * q2.W;
        return new QuaternionSelf(w, x, y, z);
    }

    // 将四元数转换为旋转矩阵
    public Matrix<double> ToRotationMatrix()
    {
        var matrix = Matrix<double>.Build.Dense(3, 3);

        matrix[0, 0] = 1 - 2 * (Y * Y + Z * Z);
        matrix[0, 1] = 2 * (X * Y - Z * W);
        matrix[0, 2] = 2 * (X * Z + Y * W);
        matrix[1, 0] = 2 * (X * Y + Z * W);
        matrix[1, 1] = 1 - 2 * (X * X + Z * Z);
        matrix[1, 2] = 2 * (Y * Z - X * W);
        matrix[2, 0] = 2 * (X * Z - Y * W);
        matrix[2, 1] = 2 * (Y * Z + X * W);
        matrix[2, 2] = 1 - 2 * (X * X + Y * Y);

        return matrix;
    }

    // 四元数的逆
    public QuaternionSelf Inverse()
    {
        double normSquared = W * W + X * X + Y * Y + Z * Z;
        return new QuaternionSelf(W / normSquared, -X / normSquared, -Y / normSquared, -Z / normSquared);
    }
}
