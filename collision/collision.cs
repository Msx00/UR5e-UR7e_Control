using System;
using System.Collections.Generic;
using System.Linq;
using Kitware.VTK;

namespace WpfRobot.collision
{
    public class CollisionPair
    {
        public int IndexA { get; set; }
        public int IndexB { get; set; }

        public string NameA { get; set; }
        public string NameB { get; set; }

        public double[] BoundsA { get; set; }
        public double[] BoundsB { get; set; }

        public double OverlapX { get; set; }
        public double OverlapY { get; set; }
        public double OverlapZ { get; set; }

        public double MinOverlap
        {
            get
            {
                return Math.Min(OverlapX, Math.Min(OverlapY, OverlapZ));
            }
        }

        public override string ToString()
        {
            return $"{NameA} <-> {NameB}, min overlap = {MinOverlap:F2} mm";
        }
    }

    public class CollisionReport
    {
        public bool HasCollision
        {
            get { return Pairs != null && Pairs.Count > 0; }
        }

        public List<CollisionPair> Pairs { get; set; } = new List<CollisionPair>();

        public string Summary
        {
            get
            {
                if (!HasCollision)
                    return "未检测到碰撞";

                return "检测到碰撞：" + string.Join("; ", Pairs.Select(p => p.ToString()));
            }
        }

        public static CollisionReport Empty()
        {
            return new CollisionReport();
        }
    }

    /// <summary>
    /// 基于 VTK PLY 模型的快速碰撞检测。
    ///
    /// 当前版本：
    /// 1. 使用 OBB 有向包围盒，而不是世界坐标 AABB；
    /// 2. fixed_end.ply 只作为显示模型，不参与碰撞；
    /// 3. wrist3.ply 的碰撞包围盒沿第七轴方向延长；
    /// 4. rotated_end.ply 的碰撞包围盒沿对应方向缩短；
    /// 5. 支持显示调试包围盒。
    /// </summary>
    public class collision
    {
        public bool Enabled { get; set; } = false;

        /// <summary>
        /// 安全余量，单位 mm。
        /// 设置为 0 表示严格按照包围盒相交判断。
        /// 设置为 5 表示两个包围盒距离小于 5mm 也认为危险。
        /// </summary>
        public double SafetyMarginMm { get; set; } = 0.0;

        /// <summary>
        /// 忽略相邻连杆。
        /// 1 表示忽略 i 和 i+1。
        /// 2 表示忽略 i 和 i+1、i+2。
        /// </summary>
        public int IgnoreNeighborSpan { get; set; } = 2;

        /// <summary>
        /// 是否使用自定义末端碰撞模型。
        /// </summary>
        public bool UseCustomEndCollisionModel { get; set; } = true;

        /// <summary>
        /// fixed_end.ply 只用于显示，不直接作为碰撞模型。
        /// </summary>
        public bool IgnoreFixedEndCollision { get; set; } = true;

        public string Wrist3LinkName { get; set; } = "wrist3";
        public string FixedEndLinkName { get; set; } = "fixed_end";
        public string RotatedEndLinkName { get; set; } = "rotated_end";

        /// <summary>
        /// wrist3 沿哪个局部轴延长。
        /// 0 = local X, 1 = local Y, 2 = local Z。
        /// </summary>
        public int Wrist3ExtendAxis { get; set; } = 2;

        /// <summary>
        /// wrist3 沿局部正方向延长多少 mm。
        /// 如果方向反了，把这个设为 0，把 Wrist3ExtendNegativeMm 设为 174。
        /// </summary>
        public double Wrist3ExtendPositiveMm { get; set; } = 174.0;

        /// <summary>
        /// wrist3 沿局部负方向延长多少 mm。
        /// </summary>
        public double Wrist3ExtendNegativeMm { get; set; } = 0.0;

        /// <summary>
        /// rotated_end 沿哪个局部轴缩短。
        /// 一般要和 Wrist3ExtendAxis 保持一致。
        /// </summary>
        public int RotatedEndShrinkAxis { get; set; } = 2;

        /// <summary>
        /// 从 rotated_end 局部正方向切掉多少 mm。
        /// </summary>
        public double RotatedEndCutPositiveMm { get; set; } = 174.0;

        /// <summary>
        /// 从 rotated_end 局部负方向切掉多少 mm。
        /// </summary>
        public double RotatedEndCutNegativeMm { get; set; } = 0.0;

        /// <summary>
        /// 防止 rotated_end 被切成负长度或退化盒子。
        /// </summary>
        public double MinCollisionBoxLengthMm { get; set; } = 5.0;
        /// <summary>
        /// fixed_end 沿哪个局部轴缩短。
        /// 一般和 Wrist3ExtendAxis 保持一致。
        /// </summary>
        public int FixedEndShrinkAxis { get; set; } = 2;

        /// <summary>
        /// 从 fixed_end 局部正方向切掉多少 mm。
        /// </summary>
        public double FixedEndCutPositiveMm { get; set; } = 0.0;

        /// <summary>
        /// 从 fixed_end 局部负方向切掉多少 mm。
        /// </summary>
        public double FixedEndCutNegativeMm { get; set; } = 174.0;

        /// <summary>
        /// 可以额外手动忽略某些 link 名字。
        /// 例如 IgnoredLinkNames.Add("some_link")
        /// </summary>
        public readonly HashSet<string> IgnoredLinkNames =
            new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        /// <summary>
        /// 可以额外手动忽略包含某些关键词的 link。
        /// 例如 IgnoredLinkKeywords.Add("screw")
        /// </summary>
        public readonly List<string> IgnoredLinkKeywords =
            new List<string>();

        private readonly Dictionary<vtkActor, double[]> originalColors =
            new Dictionary<vtkActor, double[]>();

        private readonly Dictionary<vtkActor, double> originalOpacity =
            new Dictionary<vtkActor, double>();

        private readonly List<ObbBox> lastObbBoxes = new List<ObbBox>();
        private readonly HashSet<int> lastCollidingIndices = new HashSet<int>();

        private class ObbBox
        {
            public int Index;
            public string Name;

            public double[] Center = new double[3];

            /// <summary>
            /// 三个世界坐标方向轴。
            /// Axis[0] 对应局部 X 轴方向。
            /// Axis[1] 对应局部 Y 轴方向。
            /// Axis[2] 对应局部 Z 轴方向。
            /// </summary>
            public double[][] Axis = new double[][]
            {
                new double[3],
                new double[3],
                new double[3]
            };

            /// <summary>
            /// 三个半边长。
            /// </summary>
            public double[] HalfSize = new double[3];
        }

        /// <summary>
        /// 自碰撞检测。
        ///
        /// assemblyChain: RobotVisualization 里的 urAssemblies。
        /// linkActors: 每个 PLY 对应的 vtkActor。
        /// linkNames: 每个 PLY 的文件名，不带后缀，例如 wrist3、fixed_end。
        /// </summary>
        public CollisionReport CheckSelfCollision(
            IList<vtkAssembly> assemblyChain,
            IList<vtkActor> linkActors,
            IList<string> linkNames)
        {
            CollisionReport report = new CollisionReport();

            lastObbBoxes.Clear();
            lastCollidingIndices.Clear();

            if (!Enabled)
                return report;

            if (assemblyChain == null || linkActors == null)
                return report;

            if (assemblyChain.Count < 2 || linkActors.Count == 0)
                return report;

            List<ObbBox> obbList = new List<ObbBox>();

            for (int i = 0; i < linkActors.Count; i++)
            {
                string linkName = GetLinkName(linkNames, i);

                if (IsIgnoredLink(linkName))
                    continue;

                // fixed_end.ply 形状复杂，空洞区域大，只显示，不直接参与碰撞检测。
                if (UseCustomEndCollisionModel &&
                    IgnoreFixedEndCollision &&
                    IsLinkNameMatch(linkName, FixedEndLinkName))
                {
                    continue;
                }

                ObbBox obb;
                bool ok = TryGetWorldObbForLink(
                    assemblyChain,
                    linkActors[i],
                    i,
                    linkName,
                    out obb
                );

                if (!ok || obb == null)
                    continue;

                obbList.Add(obb);
                lastObbBoxes.Add(obb);
            }

            for (int i = 0; i < obbList.Count; i++)
            {
                if (obbList[i] == null)
                    continue;

                for (int j = i + 1; j < obbList.Count; j++)
                {
                    if (obbList[j] == null)
                        continue;

                    // 同一个 actor 拆出来的多个 OBB 不互相检测。
                    if (obbList[i].Index == obbList[j].Index)
                        continue;

                    // 忽略相邻连杆，避免关节连接处天然接触误报。
                    if (Math.Abs(obbList[i].Index - obbList[j].Index) <= IgnoreNeighborSpan)
                        continue;

                    double minOverlap;

                    bool intersect = ObbIntersect(
                        obbList[i],
                        obbList[j],
                        SafetyMarginMm,
                        out minOverlap
                    );

                    if (!intersect)
                        continue;

                    CollisionPair pair = new CollisionPair
                    {
                        IndexA = obbList[i].Index,
                        IndexB = obbList[j].Index,
                        NameA = obbList[i].Name,
                        NameB = obbList[j].Name,

                        // OBB 模式下用 minOverlap 同时填充三个字段，保持原显示逻辑兼容。
                        OverlapX = minOverlap,
                        OverlapY = minOverlap,
                        OverlapZ = minOverlap
                    };

                    report.Pairs.Add(pair);

                    lastCollidingIndices.Add(obbList[i].Index);
                    lastCollidingIndices.Add(obbList[j].Index);
                }
            }

            return report;
        }

        /// <summary>
        /// 根据检测结果给碰撞连杆变红，未碰撞连杆恢复原色。
        /// </summary>
        public void ApplyVisualResult(
            IList<vtkActor> linkActors,
            CollisionReport report)
        {
            if (linkActors == null)
                return;

            HashSet<int> collisionIndices = new HashSet<int>();

            if (report != null && report.Pairs != null)
            {
                foreach (CollisionPair pair in report.Pairs)
                {
                    collisionIndices.Add(pair.IndexA);
                    collisionIndices.Add(pair.IndexB);
                }
            }

            for (int i = 0; i < linkActors.Count; i++)
            {
                vtkActor actor = linkActors[i];

                if (actor == null)
                    continue;

                SaveOriginalVisualIfNeeded(actor);

                if (collisionIndices.Contains(i))
                {
                    actor.GetProperty().SetColor(1.0, 0.05, 0.05);
                    actor.GetProperty().SetOpacity(1.0);
                }
                else
                {
                    RestoreActorVisual(actor);
                }
            }
        }

        /// <summary>
        /// 关闭碰撞检测时，恢复所有连杆颜色。
        /// </summary>
        public void ClearVisualState(IList<vtkActor> linkActors)
        {
            if (linkActors == null)
                return;

            foreach (vtkActor actor in linkActors)
            {
                if (actor == null)
                    continue;

                RestoreActorVisual(actor);
            }

            lastObbBoxes.Clear();
            lastCollidingIndices.Clear();
        }

        /// <summary>
        /// 创建当前检测状态下的 OBB 调试显示 actor。
        /// 绿色：正常包围盒。
        /// 红色：参与碰撞的包围盒。
        /// </summary>
        public List<vtkActor> CreateDebugObbActors()
        {
            List<vtkActor> actors = new List<vtkActor>();

            foreach (ObbBox box in lastObbBoxes)
            {
                if (box == null)
                    continue;

                bool isColliding = lastCollidingIndices.Contains(box.Index);

                vtkActor actor = CreateSingleObbWireActor(box, isColliding);

                if (actor != null)
                    actors.Add(actor);
            }

            return actors;
        }

        private bool TryGetWorldObbForLink(
            IList<vtkAssembly> assemblyChain,
            vtkActor actor,
            int actorIndex,
            string name,
            out ObbBox obb)
        {
            obb = null;

            if (actor == null)
                return false;

            double[] localBounds = actor.GetBounds();

            if (!IsValidBounds(localBounds))
                return false;

            // 根据 link 名字调整碰撞用局部 bounds。
            // 显示模型不变，只改变碰撞用 OBB。
            localBounds = GetAdjustedLocalBoundsForCollision(
                name,
                localBounds
            );

            if (!IsValidBounds(localBounds))
                return false;

            double[] localCenter = new double[]
            {
                0.5 * (localBounds[0] + localBounds[1]),
                0.5 * (localBounds[2] + localBounds[3]),
                0.5 * (localBounds[4] + localBounds[5])
            };

            double[] localHalfSize = new double[]
            {
                0.5 * (localBounds[1] - localBounds[0]),
                0.5 * (localBounds[3] - localBounds[2]),
                0.5 * (localBounds[5] - localBounds[4])
            };

            return TryBuildWorldObbFromLocalBox(
                assemblyChain,
                actorIndex,
                name,
                localCenter,
                localHalfSize,
                out obb
            );
        }

        private bool TryBuildWorldObbFromLocalBox(
            IList<vtkAssembly> assemblyChain,
            int actorIndex,
            string boxName,
            double[] localCenter,
            double[] localHalfSize,
            out ObbBox obb)
        {
            obb = null;

            if (assemblyChain == null)
                return false;

            if (localCenter == null || localCenter.Length < 3)
                return false;

            if (localHalfSize == null || localHalfSize.Length < 3)
                return false;

            // actorIndex = 0 对应 urAssemblies[1]
            // actorIndex = 1 对应 urAssemblies[2]
            // ...
            int assemblyIndex = actorIndex + 1;

            if (assemblyIndex < 0 || assemblyIndex >= assemblyChain.Count)
                return false;

            double[,] worldMatrix = GetCumulativeMatrix(
                assemblyChain,
                assemblyIndex
            );

            obb = new ObbBox();
            obb.Index = actorIndex;
            obb.Name = boxName;

            obb.Center = TransformPoint(localCenter, worldMatrix);

            for (int axisId = 0; axisId < 3; axisId++)
            {
                double[] axis = new double[]
                {
                    worldMatrix[0, axisId],
                    worldMatrix[1, axisId],
                    worldMatrix[2, axisId]
                };

                double scale = Norm(axis);

                if (scale < 1e-12)
                    return false;

                obb.Axis[axisId] = new double[]
                {
                    axis[0] / scale,
                    axis[1] / scale,
                    axis[2] / scale
                };

                obb.HalfSize[axisId] =
                    Math.Max(1e-6, localHalfSize[axisId] * scale);
            }

            return true;
        }

        private double[] GetAdjustedLocalBoundsForCollision( string linkName, double[] originalBounds)
        {
            double[] b = CopyBounds(originalBounds);

            if (!UseCustomEndCollisionModel)
                return b;

            // 1. wrist3.ply 的碰撞包围盒沿第七轴方向延长 d7 = 174mm
            if (IsLinkNameMatch(linkName, Wrist3LinkName))
            {
                ExtendBoundsAlongAxis(
                    b,
                    Wrist3ExtendAxis,
                    Wrist3ExtendNegativeMm,
                    Wrist3ExtendPositiveMm
                );

                return b;
            }

            // 2. fixed_end.ply 的碰撞包围盒沿同一方向缩短
            // 注意：fixed_end 仍然会显示完整 PLY，
            // 这里只是缩短它的碰撞 OBB。
            if (IsLinkNameMatch(linkName, FixedEndLinkName))
            {
                CutBoundsAlongAxis(
                    b,
                    FixedEndShrinkAxis,
                    FixedEndCutNegativeMm,
                    FixedEndCutPositiveMm,
                    MinCollisionBoxLengthMm
                );

                return b;
            }

            // 3. rotated_end.ply 当前不缩短
            // 这里保留接口，但你现在设置为 0，所以不会改变 bounds。
            if (IsLinkNameMatch(linkName, RotatedEndLinkName))
            {
                CutBoundsAlongAxis(
                    b,
                    RotatedEndShrinkAxis,
                    RotatedEndCutNegativeMm,
                    RotatedEndCutPositiveMm,
                    MinCollisionBoxLengthMm
                );

                return b;
            }

            return b;
        }
       

        private bool IsIgnoredLink(string linkName)
        {
            if (string.IsNullOrWhiteSpace(linkName))
                return false;

            if (IgnoredLinkNames.Contains(linkName))
                return true;

            foreach (string key in IgnoredLinkKeywords)
            {
                if (string.IsNullOrWhiteSpace(key))
                    continue;

                if (linkName.IndexOf(
                        key,
                        StringComparison.OrdinalIgnoreCase
                    ) >= 0)
                {
                    return true;
                }
            }

            return false;
        }

        private static bool IsLinkNameMatch(string linkName, string key)
        {
            if (string.IsNullOrWhiteSpace(linkName))
                return false;

            if (string.IsNullOrWhiteSpace(key))
                return false;

            return linkName.IndexOf(
                key,
                StringComparison.OrdinalIgnoreCase
            ) >= 0;
        }

        private static double[] CopyBounds(double[] b)
        {
            return new double[]
            {
                b[0], b[1],
                b[2], b[3],
                b[4], b[5]
            };
        }

        private static void GetAxisBoundIndex(
            int axis,
            out int minIndex,
            out int maxIndex)
        {
            if (axis == 0)
            {
                minIndex = 0;
                maxIndex = 1;
            }
            else if (axis == 1)
            {
                minIndex = 2;
                maxIndex = 3;
            }
            else
            {
                minIndex = 4;
                maxIndex = 5;
            }
        }

        private static void ExtendBoundsAlongAxis(
            double[] bounds,
            int axis,
            double extendNegativeMm,
            double extendPositiveMm)
        {
            if (bounds == null || bounds.Length < 6)
                return;

            GetAxisBoundIndex(axis, out int minIndex, out int maxIndex);

            if (extendNegativeMm > 0)
                bounds[minIndex] -= extendNegativeMm;

            if (extendPositiveMm > 0)
                bounds[maxIndex] += extendPositiveMm;
        }

        private static void CutBoundsAlongAxis(
            double[] bounds,
            int axis,
            double cutNegativeMm,
            double cutPositiveMm,
            double minLengthMm)
        {
            if (bounds == null || bounds.Length < 6)
                return;

            GetAxisBoundIndex(axis, out int minIndex, out int maxIndex);

            double length = bounds[maxIndex] - bounds[minIndex];

            if (length <= minLengthMm)
                return;

            double safeCutNegative = Math.Max(0.0, cutNegativeMm);
            double safeCutPositive = Math.Max(0.0, cutPositiveMm);

            double totalCut = safeCutNegative + safeCutPositive;
            double maxAllowedCut = Math.Max(0.0, length - minLengthMm);

            if (totalCut > maxAllowedCut && totalCut > 1e-9)
            {
                double scale = maxAllowedCut / totalCut;
                safeCutNegative *= scale;
                safeCutPositive *= scale;
            }

            if (safeCutNegative > 0)
                bounds[minIndex] += safeCutNegative;

            if (safeCutPositive > 0)
                bounds[maxIndex] -= safeCutPositive;
        }

        /// <summary>
        /// OBB 相交检测。
        /// 使用 SAT 分离轴定理。
        /// </summary>
        private static bool ObbIntersect(
            ObbBox a,
            ObbBox b,
            double margin,
            out double minOverlap)
        {
            minOverlap = double.PositiveInfinity;

            const double EPS = 1e-8;

            double[,] R = new double[3, 3];
            double[,] AbsR = new double[3, 3];

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    R[i, j] = Dot(a.Axis[i], b.Axis[j]);
                    AbsR[i, j] = Math.Abs(R[i, j]) + EPS;
                }
            }

            double[] tWorld = new double[]
            {
                b.Center[0] - a.Center[0],
                b.Center[1] - a.Center[1],
                b.Center[2] - a.Center[2]
            };

            double[] t = new double[]
            {
                Dot(tWorld, a.Axis[0]),
                Dot(tWorld, a.Axis[1]),
                Dot(tWorld, a.Axis[2])
            };

            double ra;
            double rb;
            double dist;
            double overlap;

            // A 的三个轴。
            for (int i = 0; i < 3; i++)
            {
                ra = a.HalfSize[i];

                rb =
                    b.HalfSize[0] * AbsR[i, 0] +
                    b.HalfSize[1] * AbsR[i, 1] +
                    b.HalfSize[2] * AbsR[i, 2];

                dist = Math.Abs(t[i]);
                overlap = ra + rb + margin - dist;

                if (overlap < 0)
                    return false;

                minOverlap = Math.Min(minOverlap, overlap);
            }

            // B 的三个轴。
            for (int j = 0; j < 3; j++)
            {
                ra =
                    a.HalfSize[0] * AbsR[0, j] +
                    a.HalfSize[1] * AbsR[1, j] +
                    a.HalfSize[2] * AbsR[2, j];

                rb = b.HalfSize[j];

                dist = Math.Abs(
                    t[0] * R[0, j] +
                    t[1] * R[1, j] +
                    t[2] * R[2, j]
                );

                overlap = ra + rb + margin - dist;

                if (overlap < 0)
                    return false;

                minOverlap = Math.Min(minOverlap, overlap);
            }

            // A_i x B_j 的 9 个叉乘轴。
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    int i1 = (i + 1) % 3;
                    int i2 = (i + 2) % 3;
                    int j1 = (j + 1) % 3;
                    int j2 = (j + 2) % 3;

                    ra =
                        a.HalfSize[i1] * AbsR[i2, j] +
                        a.HalfSize[i2] * AbsR[i1, j];

                    rb =
                        b.HalfSize[j1] * AbsR[i, j2] +
                        b.HalfSize[j2] * AbsR[i, j1];

                    dist = Math.Abs(
                        t[i2] * R[i1, j] -
                        t[i1] * R[i2, j]
                    );

                    overlap = ra + rb + margin - dist;

                    if (overlap < 0)
                        return false;

                    minOverlap = Math.Min(minOverlap, overlap);
                }
            }

            if (double.IsPositiveInfinity(minOverlap))
                minOverlap = 0.0;

            return true;
        }

        private static vtkActor CreateSingleObbWireActor(
            ObbBox box,
            bool isColliding)
        {
            if (box == null)
                return null;

            vtkCubeSource cube = vtkCubeSource.New();

            cube.SetCenter(0.0, 0.0, 0.0);
            cube.SetXLength(Math.Max(1e-6, 2.0 * box.HalfSize[0]));
            cube.SetYLength(Math.Max(1e-6, 2.0 * box.HalfSize[1]));
            cube.SetZLength(Math.Max(1e-6, 2.0 * box.HalfSize[2]));

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputConnection(cube.GetOutputPort());

            vtkActor actor = vtkActor.New();
            actor.SetMapper(mapper);

            actor.GetProperty().SetRepresentationToWireframe();
            actor.GetProperty().SetLineWidth(isColliding ? 3.0f : 1.5f);

            if (isColliding)
            {
                actor.GetProperty().SetColor(1.0, 0.0, 0.0);
                actor.GetProperty().SetOpacity(1.0);
            }
            else
            {
                actor.GetProperty().SetColor(0.0, 1.0, 0.2);
                actor.GetProperty().SetOpacity(0.6);
            }

            vtkMatrix4x4 matrix = vtkMatrix4x4.New();

            // 第一列：OBB 的 X 轴方向。
            matrix.SetElement(0, 0, box.Axis[0][0]);
            matrix.SetElement(1, 0, box.Axis[0][1]);
            matrix.SetElement(2, 0, box.Axis[0][2]);
            matrix.SetElement(3, 0, 0.0);

            // 第二列：OBB 的 Y 轴方向。
            matrix.SetElement(0, 1, box.Axis[1][0]);
            matrix.SetElement(1, 1, box.Axis[1][1]);
            matrix.SetElement(2, 1, box.Axis[1][2]);
            matrix.SetElement(3, 1, 0.0);

            // 第三列：OBB 的 Z 轴方向。
            matrix.SetElement(0, 2, box.Axis[2][0]);
            matrix.SetElement(1, 2, box.Axis[2][1]);
            matrix.SetElement(2, 2, box.Axis[2][2]);
            matrix.SetElement(3, 2, 0.0);

            // 第四列：OBB 中心。
            matrix.SetElement(0, 3, box.Center[0]);
            matrix.SetElement(1, 3, box.Center[1]);
            matrix.SetElement(2, 3, box.Center[2]);
            matrix.SetElement(3, 3, 1.0);

            vtkTransform transform = vtkTransform.New();
            transform.SetMatrix(matrix);

            actor.SetUserTransform(transform);

            return actor;
        }

        private static double[,] GetCumulativeMatrix(
            IList<vtkAssembly> assemblyChain,
            int assemblyIndex)
        {
            double[,] result = Identity4x4();

            for (int i = 0; i <= assemblyIndex; i++)
            {
                vtkAssembly assembly = assemblyChain[i];

                if (assembly == null)
                    continue;

                vtkMatrix4x4 vtkM = assembly.GetMatrix();
                double[,] local = FromVtkMatrix(vtkM);

                result = Multiply4x4(result, local);
            }

            return result;
        }

        private static double[,] Identity4x4()
        {
            double[,] m = new double[4, 4];

            for (int i = 0; i < 4; i++)
                m[i, i] = 1.0;

            return m;
        }

        private static double[,] FromVtkMatrix(vtkMatrix4x4 vtkM)
        {
            double[,] m = new double[4, 4];

            if (vtkM == null)
                return Identity4x4();

            for (int r = 0; r < 4; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    m[r, c] = vtkM.GetElement(r, c);
                }
            }

            return m;
        }

        private static double[,] Multiply4x4(double[,] a, double[,] b)
        {
            double[,] c = new double[4, 4];

            for (int r = 0; r < 4; r++)
            {
                for (int col = 0; col < 4; col++)
                {
                    double sum = 0.0;

                    for (int k = 0; k < 4; k++)
                    {
                        sum += a[r, k] * b[k, col];
                    }

                    c[r, col] = sum;
                }
            }

            return c;
        }

        private static double[] TransformPoint(double[] p, double[,] m)
        {
            double x =
                m[0, 0] * p[0] +
                m[0, 1] * p[1] +
                m[0, 2] * p[2] +
                m[0, 3];

            double y =
                m[1, 0] * p[0] +
                m[1, 1] * p[1] +
                m[1, 2] * p[2] +
                m[1, 3];

            double z =
                m[2, 0] * p[0] +
                m[2, 1] * p[1] +
                m[2, 2] * p[2] +
                m[2, 3];

            return new double[] { x, y, z };
        }

        private static double Dot(double[] a, double[] b)
        {
            return
                a[0] * b[0] +
                a[1] * b[1] +
                a[2] * b[2];
        }

        private static double Norm(double[] a)
        {
            return Math.Sqrt(Dot(a, a));
        }

        private static bool IsValidBounds(double[] b)
        {
            if (b == null || b.Length < 6)
                return false;

            for (int i = 0; i < 6; i++)
            {
                if (double.IsNaN(b[i]) || double.IsInfinity(b[i]))
                    return false;
            }

            if (b[0] > b[1])
                return false;

            if (b[2] > b[3])
                return false;

            if (b[4] > b[5])
                return false;

            return true;
        }

        private static string GetLinkName(IList<string> names, int index)
        {
            if (names == null || index < 0 || index >= names.Count)
                return "link_" + index;

            return names[index];
        }

        private void SaveOriginalVisualIfNeeded(vtkActor actor)
        {
            if (actor == null)
                return;

            if (!originalColors.ContainsKey(actor))
            {
                double[] c = actor.GetProperty().GetColor();

                originalColors[actor] = new double[]
                {
                    c[0],
                    c[1],
                    c[2]
                };
            }

            if (!originalOpacity.ContainsKey(actor))
            {
                originalOpacity[actor] = actor.GetProperty().GetOpacity();
            }
        }

        private void RestoreActorVisual(vtkActor actor)
        {
            if (actor == null)
                return;

            if (originalColors.ContainsKey(actor))
            {
                double[] c = originalColors[actor];
                actor.GetProperty().SetColor(c[0], c[1], c[2]);
            }

            if (originalOpacity.ContainsKey(actor))
            {
                actor.GetProperty().SetOpacity(originalOpacity[actor]);
            }
        }
    }
}