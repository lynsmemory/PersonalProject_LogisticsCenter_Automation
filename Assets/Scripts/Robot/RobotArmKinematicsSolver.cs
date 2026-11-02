using System;
using System.Collections.Generic;

namespace Logistics
{
    // FR5 순기구학과 흡착 툴이 아래를 향하는 자세의 해석적 역기구학을 계산한다. (FR-10)
    // Unity 형식을 쓰지 않는 계산 전용 클래스라 Unity 밖에서도 시험할 수 있다. 로봇 구동은 RobotArmInverseKinematics가 한다.
    // 좌표: 로봇 베이스 기준 오른손 좌표 (X 앞, Y 왼쪽, Z 위, m). 각도: 라디안, URDF 관절 부호.
    // 근거: fairino5_v6.urdf 치수·관절 제한, Docs/Research/calculate_fr5_layout.py의 fk·ik_down,
    //       Docs/Research/FR5_CAD_Validation/verify.py의 ik_all (위치 계산식만 옮기고, 옛 받침·툴 치수는 쓰지 않는다)
    public static class RobotArmKinematicsSolver
    {
        #region 1. FR5 치수와 관절 제한
        public const double ShoulderHeight = 0.152;   // D1: 베이스 → 어깨 관절 높이
        public const double UpperArmLength = 0.425;   // A: 어깨 → 팔꿈치
        public const double ForearmLength = 0.39501;  // B: 팔꿈치 → 손목1
        public const double Wrist1Offset = 0.1021;    // D4: 손목1 → 손목2 옆 방향 거리
        public const double Wrist2Offset = 0.102;     // D5: 손목2 → 손목3 거리
        public const double FlangeOffset = 0.100;     // D6: 손목3 원점 → 툴 장착면

        // 관절 제한 (라디안). URDF 값이며 j3만 제조사 ±160도를 쓴다.
        public static readonly double[] LowerLimits = { -3.0543, -4.6251, -2.7925268, -4.6251, -3.0543, -3.0543 };
        public static readonly double[] UpperLimits = { 3.0543, 1.4835, 2.7925268, 1.4835, 3.0543, 3.0543 };

        // 관절마다 베이스 쪽 원점 위치와 X축 회전 (URDF joint origin)
        static readonly double[][] JointOrigins =
        {
            new[] { 0.0, 0.0, 0.0 },
            new[] { 0.0, 0.0, ShoulderHeight },
            new[] { -UpperArmLength, 0.0, 0.0 },
            new[] { -ForearmLength, 0.0, 0.0 },
            new[] { 0.0, 0.0, Wrist1Offset },
            new[] { 0.0, 0.0, Wrist2Offset },
        };
        static readonly double[] JointRolls = { 0.0, Math.PI / 2, 0.0, 0.0, Math.PI / 2, -Math.PI / 2 };
        #endregion

        #region 2. 순기구학
        // 관절 각도 → 흡착면 중심 위치와 툴 자세(3×3, 열 = 툴 X·Y·Z축). 툴 Z축이 흡착 방향이다.
        public static void ForwardKinematics(double[] jointAngles, double toolLength, double[] position, double[,] rotation)
        {
            double[,] r = Identity();
            double px = 0.0, py = 0.0, pz = 0.0;
            for (int i = 0; i < 6; i++)
            {
                //1. 현재 자세에서 관절 원점까지 이동
                double[] o = JointOrigins[i];
                px += r[0, 0] * o[0] + r[0, 1] * o[1] + r[0, 2] * o[2];
                py += r[1, 0] * o[0] + r[1, 1] * o[1] + r[1, 2] * o[2];
                pz += r[2, 0] * o[0] + r[2, 1] * o[1] + r[2, 2] * o[2];

                //2. 관절 원점의 X축 회전 뒤 관절 Z축으로 회전
                r = Multiply(Multiply(r, RotationX(JointRolls[i])), RotationZ(jointAngles[i]));
            }

            //3. 툴 축(Z)을 따라 장착면 + 툴 길이만큼 이동
            double reach = FlangeOffset + toolLength;
            position[0] = px + r[0, 2] * reach;
            position[1] = py + r[1, 2] * reach;
            position[2] = pz + r[2, 2] * reach;
            if (rotation != null)
            {
                Array.Copy(r, rotation, 9);
            }
        }
        #endregion

        #region 3. 역기구학 (툴 아래 방향)
        // 툴이 정확히 아래(-Z)를 향하고 흡착면 중심이 (x, y, z)에 오며 툴 X축이 yaw 방향(또는 반대)을 향하는 관절 해를 모두 구한다.
        // 흡착 헤드와 박스 윗면이 180도 대칭이라 yaw와 yaw+180도를 같은 목표로 본다. 관절 제한 밖의 해는 빼고 2π 차이 해는 모두 넣는다.
        public static List<double[]> SolveDownwardToolAll(double x, double y, double z, double yaw, double toolLength)
        {
            var solutions = new List<double[]>();
            double r2 = x * x + y * y;
            if (r2 < Wrist1Offset * Wrist1Offset)
            {
                return solutions;   // 손목1 옆 거리보다 베이스 축에 가까운 점은 닿을 수 없다
            }

            double planar = Math.Sqrt(r2 - Wrist1Offset * Wrist1Offset);   // 손목1 옆 거리를 뺀 앞쪽 거리
            var position = new double[3];
            var orientation = new double[3, 3];
            foreach (double w in new[] { planar, -planar })
            {
                foreach (double q1 in Equivalents(Math.Atan2(y, x) - Math.Atan2(-Wrist1Offset, w), 0))
                {
                    foreach (int sign in new[] { 1, -1 })
                    {
                        //1. 손목 위치까지를 2링크(위팔·아래팔) 평면 문제로 푼다
                        double u = w - sign * Wrist2Offset;
                        double v = z - ShoulderHeight + FlangeOffset + toolLength;
                        double c = (u * u + v * v - UpperArmLength * UpperArmLength - ForearmLength * ForearmLength)
                                   / (2.0 * UpperArmLength * ForearmLength);
                        if (Math.Abs(c) > 1.0 + 1e-9)
                        {
                            continue;
                        }

                        double elbowMagnitude = Math.Acos(Math.Max(-1.0, Math.Min(1.0, c)));
                        foreach (double elbow in new[] { elbowMagnitude, -elbowMagnitude })
                        {
                            double shoulder = Math.Atan2(-v, -u)
                                              - Math.Atan2(ForearmLength * Math.Sin(elbow), UpperArmLength + ForearmLength * Math.Cos(elbow));
                            foreach (double q2 in Equivalents(shoulder, 1))
                            foreach (double q3 in Equivalents(elbow, 2))
                            foreach (double q4 in Equivalents(sign * Math.PI / 2 - q2 - q3, 3))
                            {
                                //2. 손목2를 ±90도로 두어 툴을 아래로 향하게 하고, 손목3으로 yaw를 맞춘다
                                double[] q = { q1, q2, q3, q4, sign * Math.PI / 2, 0.0 };
                                if (!WithinLimits(q, 4))
                                {
                                    continue;
                                }

                                ForwardKinematics(q, toolLength, position, orientation);
                                double phi = Math.Atan2(orientation[1, 0], orientation[0, 0]);
                                foreach (double desired in new[] { yaw, yaw + Math.PI })
                                {
                                    foreach (double q6 in Equivalents(phi - desired, 5))
                                    {
                                        var candidate = (double[])q.Clone();
                                        candidate[5] = q6;
                                        if (!ContainsSame(solutions, candidate))
                                        {
                                            solutions.Add(candidate);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            //3. 계산식 확인: 순기구학으로 위치와 아래 방향을 다시 검사해 어긋난 해는 버린다
            solutions.RemoveAll(q =>
            {
                ForwardKinematics(q, toolLength, position, orientation);
                double error = Math.Sqrt(Square(position[0] - x) + Square(position[1] - y) + Square(position[2] - z));
                return error > 1e-6 || orientation[2, 2] > -1.0 + 1e-6;
            });
            return solutions;
        }

        // 해 중에서 현재 관절 각도와 가장 가까운(가장 크게 움직이는 관절의 변화가 작은) 해를 고른다. 해가 없으면 false.
        // referenceAngles(보통 홈 자세)를 주면 그 자세와 같은 계열(팔꿈치·손목 방향, 위팔이 위쪽)의 해를 먼저 고른다.
        // 가장 가까운 해만 고르면 위팔이 아래로 꺾여 받침에 닿는 자세가 뽑힐 수 있어서다. 같은 계열에 해가 없을 때만 다른 해를 쓴다.
        public static bool SolveDownwardTool(double x, double y, double z, double yaw, double toolLength, double[] currentAngles,
                                             double[] referenceAngles, out double[] best)
        {
            best = null;
            double bestCost = double.MaxValue;
            bool bestSameFamily = false;
            foreach (double[] q in SolveDownwardToolAll(x, y, z, yaw, toolLength))
            {
                double cost = 0.0;
                for (int i = 0; i < 6; i++)
                {
                    cost = Math.Max(cost, Math.Abs(q[i] - currentAngles[i]));
                }
                bool sameFamily = referenceAngles != null && SameFamily(q, referenceAngles);
                if ((sameFamily && !bestSameFamily) || (sameFamily == bestSameFamily && cost < bestCost))
                {
                    bestCost = cost;
                    best = q;
                    bestSameFamily = sameFamily;
                }
            }
            return best != null;
        }

        // 자세 계열: 팔꿈치(j3)·손목2(j5) 부호가 같고 위팔(j2)이 -180~0도 (베이스 위쪽으로 든 자세)
        public static bool SameFamily(double[] q, double[] reference)
        {
            return Math.Sign(q[2]) == Math.Sign(reference[2])
                   && Math.Sign(q[4]) == Math.Sign(reference[4])
                   && q[1] >= -Math.PI && q[1] <= 0.0;
        }

        // 관절 각도가 모두 제한 안에 있는지 (count개 관절까지)
        public static bool WithinLimits(double[] jointAngles, int count = 6)
        {
            for (int i = 0; i < count; i++)
            {
                if (jointAngles[i] < LowerLimits[i] || jointAngles[i] > UpperLimits[i])
                {
                    return false;
                }
            }
            return true;
        }
        #endregion

        #region 4. 계산 도구
        // 같은 자세를 뜻하는 q + 2πk 중 관절 제한 안의 값들
        static IEnumerable<double> Equivalents(double angle, int joint)
        {
            for (int k = -2; k <= 2; k++)
            {
                double candidate = angle + 2.0 * Math.PI * k;
                if (candidate >= LowerLimits[joint] && candidate <= UpperLimits[joint])
                {
                    yield return candidate;
                }
            }
        }

        static bool ContainsSame(List<double[]> list, double[] q)
        {
            foreach (double[] other in list)
            {
                double difference = 0.0;
                for (int i = 0; i < 6; i++)
                {
                    difference = Math.Max(difference, Math.Abs(other[i] - q[i]));
                }
                if (difference < 1e-6)
                {
                    return true;
                }
            }
            return false;
        }

        static double Square(double value) => value * value;

        static double[,] Identity() => new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        static double[,] RotationX(double angle)
        {
            double c = Math.Cos(angle), s = Math.Sin(angle);
            return new double[,] { { 1, 0, 0 }, { 0, c, -s }, { 0, s, c } };
        }

        static double[,] RotationZ(double angle)
        {
            double c = Math.Cos(angle), s = Math.Sin(angle);
            return new double[,] { { c, -s, 0 }, { s, c, 0 }, { 0, 0, 1 } };
        }

        static double[,] Multiply(double[,] a, double[,] b)
        {
            var result = new double[3, 3];
            for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                result[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
            }
            return result;
        }
        #endregion
    }
}
