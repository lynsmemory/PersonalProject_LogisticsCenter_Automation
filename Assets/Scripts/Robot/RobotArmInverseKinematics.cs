using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Logistics
{
    // FR5 팔을 움직인다. (FR-10)
    // 흡착 툴이 아래를 향하는 목표(월드 위치·수평 회전)를 RobotArmKinematicsSolver로 관절 각도로 바꾸고 ArticulationBody 구동 목표로 옮긴다.
    // 닿지 않거나 관절 제한 밖인 목표는 움직이지 않고 실패로 남긴다. 충돌체를 끄거나 박스를 순간 이동시키지 않는다.
    // 로봇 루트(FR5_LaneN)에 1개씩 붙인다. 루트의 원점과 방향이 로봇 베이스다.
    [DisallowMultipleComponent]
    public class RobotArmInverseKinematics : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("URDF 관절 j1~j6 순서의 링크 (shoulder_link, upperarm_link, forearm_link, wrist1_link, wrist2_link, wrist3_link)")]
        [SerializeField] private ArticulationBody[] joints = new ArticulationBody[6];

        [Tooltip("홈 자세 관절 각도 (도, URDF 부호). 팔을 세우고 툴이 아래를 향하는 자세")]
        [SerializeField] private float[] homeJointAngles = { 0f, -90f, -90f, -90f, 90f, 0f };

        [Tooltip("툴 장착면 → 흡착면 중심 길이 (m). 명목값 0.15, 실제 흡착점 보정은 검수 후 조정")]
        [SerializeField, Min(0f)] private float toolLength = 0.15f;

        [Tooltip("관절 최대 속도 (도/초). 가장 많이 움직이는 관절의 가장 빠른 순간 속도이며 나머지 관절은 같이 끝나도록 맞춘다")]
        [SerializeField, Min(1f)] private float jointSpeedDegrees = 45f;

        [Tooltip("목표 각도 도달 판정 오차 (도)")]
        [SerializeField, Min(0.01f)] private float reachToleranceDegrees = 1f;

        [Tooltip("구동 목표를 다 보낸 뒤 실제 관절이 따라올 때까지 기다리는 최대 시간 (초)")]
        [SerializeField, Min(0.1f)] private float settleTimeoutSeconds = 3f;

        [Tooltip("한 번 이동의 최소 시간 (초). 짧은 이동도 급하게 출발·정지하지 않게 한다")]
        [SerializeField, Min(0.02f)] private float minimumMoveSeconds = 0.3f;

        [Tooltip("직선 이동 경로점 간격 (m)")]
        [SerializeField, Min(0.005f)] private float linearStepMeters = 0.02f;
        #endregion

        #region 2. 상태
        public bool IsMoving { get; private set; }               // 이동 중
        public bool LastMoveSucceeded { get; private set; }      // 마지막 이동 성공 여부 (해 없음·제한 밖·시간 초과·초기화 중단이면 false)
        public float LastReachError { get; private set; }        // 마지막 이동 뒤 흡착면 중심과 목표의 거리 (m)
        public float ToolLength => toolLength;

        // 관절 0도가 가리키는 URDF 각도 (라디안). 씬에 다른 자세로 저장되며 관절 기준이 그 자세로 바뀐 경우를 보정한다.
        private readonly double[] referenceOffsets = new double[6];
        private int moveVersion;   // 초기화 때 진행 중인 이동을 끊기 위한 번호

        // URDF 0도일 때 각 링크의 부모 기준 회전 (fairino5_v6.urdf joint origin을 URDF Importer Y Axis로 불러온 값)
        private static readonly Quaternion[] UrdfZeroLocalRotations =
        {
            Quaternion.identity,
            new Quaternion(0f, 0f, -0.70710677f, 0.70710677f),
            Quaternion.identity,
            Quaternion.identity,
            new Quaternion(0f, 0f, -0.70710677f, 0.70710677f),
            new Quaternion(0f, 0f, 0.70710677f, 0.70710677f),
        };
        #endregion

        #region 3. 시작과 초기화
        private void Awake()
        {
            CalibrateJointReferences();
            ApplyJointLimits();
        }

        // 시작 때 홈 자세로 즉시 배치한다. 0도 자세는 팔이 수평으로 뻗어 레인과 겹치므로 그 자세에서 움직이게 두지 않는다.
        private void Start()
        {
            ResetToInitialState();
        }

        // 초기화 버튼: 진행 중인 이동을 멈추고 관절을 홈 자세로 즉시 옮기며 구동 목표도 같은 각도로 맞춘다.
        public void ResetToInitialState()
        {
            moveVersion++;
            IsMoving = false;
            for (int i = 0; i < joints.Length && i < homeJointAngles.Length; i++)
            {
                ArticulationBody joint = joints[i];
                if (joint == null)
                {
                    continue;
                }

                //1. 관절 각도와 속도를 바로 바꾼다 (라디안, 관절 기준 보정 적용)
                double articulationAngle = homeJointAngles[i] * Mathf.Deg2Rad - referenceOffsets[i];
                joint.jointPosition = new ArticulationReducedSpace((float)articulationAngle);
                joint.jointVelocity = new ArticulationReducedSpace(0f);

                //2. 구동 목표를 같은 각도로 맞춰 그 자리를 유지한다 (도)
                SetDriveTarget(i, articulationAngle);
            }
        }

        // 관절마다 저장된 기준(관절 0도)이 URDF 0도에서 얼마나 돌아가 있는지 앵커 회전으로 계산한다.
        private void CalibrateJointReferences()
        {
            for (int i = 0; i < joints.Length && i < UrdfZeroLocalRotations.Length; i++)
            {
                ArticulationBody joint = joints[i];
                if (joint == null)
                {
                    continue;
                }

                //1. URDF 0도일 때의 부모 앵커 회전과 실제 부모 앵커 회전의 차이
                Quaternion expected = UrdfZeroLocalRotations[i] * joint.anchorRotation;
                Quaternion difference = Quaternion.Inverse(expected) * joint.parentAnchorRotation;
                if (difference.w < 0f)
                {
                    difference = new Quaternion(-difference.x, -difference.y, -difference.z, -difference.w);
                }

                //2. 차이는 관절 축(앵커 X축) 회전이어야 한다. 다른 축 성분이 있으면 URDF와 다른 연결이므로 알린다
                float offAxis = Mathf.Sqrt(difference.y * difference.y + difference.z * difference.z);
                if (offAxis > 0.01f)
                {
                    Debug.LogWarning($"[RobotArmInverseKinematics] {name} {joint.name}: 관절 기준이 관절 축 밖으로 {offAxis:F3} 어긋남");
                }
                referenceOffsets[i] = 2.0 * System.Math.Atan2(difference.x, difference.w);
            }
        }

        // URDF 관절 제한을 관절 기준 보정에 맞춰 구동 제한으로 넣는다.
        private void ApplyJointLimits()
        {
            for (int i = 0; i < joints.Length; i++)
            {
                if (joints[i] == null)
                {
                    continue;
                }
                ArticulationDrive drive = joints[i].xDrive;
                drive.lowerLimit = (float)((RobotArmKinematicsSolver.LowerLimits[i] - referenceOffsets[i]) * Mathf.Rad2Deg);
                drive.upperLimit = (float)((RobotArmKinematicsSolver.UpperLimits[i] - referenceOffsets[i]) * Mathf.Rad2Deg);
                joints[i].xDrive = drive;
            }
        }
        #endregion

        #region 4. 좌표 변환 (Unity 월드 ↔ 로봇 베이스 오른손 좌표)
        // 월드 위치 → 베이스 좌표 (X 앞, Y 왼쪽, Z 위)
        public Vector3 WorldToBase(Vector3 worldPosition)
        {
            Vector3 local = transform.InverseTransformPoint(worldPosition);
            return new Vector3(local.z, -local.x, local.y);
        }

        // 베이스 좌표 → 월드 위치
        public Vector3 BaseToWorld(Vector3 basePosition)
        {
            return transform.TransformPoint(new Vector3(-basePosition.y, basePosition.z, basePosition.x));
        }

        // 월드 Y축 회전(도) → 베이스 좌표의 수평 회전(라디안). 툴 X축이 향할 월드 방향으로 맞춘다.
        private double WorldYawToBase(float worldYawDegrees)
        {
            Vector3 local = transform.InverseTransformDirection(Quaternion.Euler(0f, worldYawDegrees, 0f) * Vector3.forward);
            return System.Math.Atan2(-local.x, local.z);
        }
        #endregion

        #region 5. 기구학
        // 현재 관절 각도 (도, URDF 부호)
        public float[] GetJointAngles()
        {
            double[] radians = ReadJointAngles();
            var degrees = new float[6];
            for (int i = 0; i < 6; i++)
            {
                degrees[i] = (float)(radians[i] * Mathf.Rad2Deg);
            }
            return degrees;
        }

        // 현재 흡착면 중심의 월드 위치
        public Vector3 CurrentToolPosition => ToolPositionOf(ReadJointAngles());

        // 관절 각도(도) → 흡착면 중심의 월드 위치
        public Vector3 ForwardKinematics(float[] jointAnglesDegrees)
        {
            var radians = new double[6];
            for (int i = 0; i < 6; i++)
            {
                radians[i] = jointAnglesDegrees[i] * Mathf.Deg2Rad;
            }
            return ToolPositionOf(radians);
        }

        // 툴이 아래를 향한 채 흡착면 중심이 월드 위치에 오는 관절 각도(도). 홈 자세와 같은 계열 중 현재 자세와 가장 가까운 해를 고른다. 해가 없으면 false.
        public bool SolveDownwardTool(Vector3 worldPosition, float worldYawDegrees, out float[] jointAnglesDegrees)
        {
            jointAnglesDegrees = null;
            if (!Solve(worldPosition, worldYawDegrees, ReadJointAngles(), out double[] radians))
            {
                return false;
            }
            jointAnglesDegrees = new float[6];
            for (int i = 0; i < 6; i++)
            {
                jointAnglesDegrees[i] = (float)(radians[i] * Mathf.Rad2Deg);
            }
            return true;
        }

        private bool Solve(Vector3 worldPosition, float worldYawDegrees, double[] nearAngles, out double[] radians)
        {
            Vector3 target = WorldToBase(worldPosition);
            var home = new double[6];
            for (int i = 0; i < 6 && i < homeJointAngles.Length; i++)
            {
                home[i] = homeJointAngles[i] * Mathf.Deg2Rad;
            }
            return RobotArmKinematicsSolver.SolveDownwardTool(target.x, target.y, target.z, WorldYawToBase(worldYawDegrees),
                                                              toolLength, nearAngles, home, out radians);
        }

        private Vector3 ToolPositionOf(double[] radians)
        {
            var position = new double[3];
            RobotArmKinematicsSolver.ForwardKinematics(radians, toolLength, position, null);
            return BaseToWorld(new Vector3((float)position[0], (float)position[1], (float)position[2]));
        }

        // 실제 관절 각도 (라디안, URDF 부호) = 관절 위치 + 기준 보정
        private double[] ReadJointAngles()
        {
            var radians = new double[6];
            for (int i = 0; i < 6 && i < joints.Length; i++)
            {
                if (joints[i] != null)
                {
                    radians[i] = joints[i].jointPosition[0] + referenceOffsets[i];
                }
            }
            return radians;
        }
        #endregion

        #region 6. 이동
        // 툴을 아래로 향한 채 목표 위치·회전으로 관절 이동한다. 해가 없으면 움직이지 않고 LastMoveSucceeded = false.
        public IEnumerator MoveTo(Vector3 worldPosition, float worldYawDegrees)
        {
            if (!BeginMove())
            {
                yield break;
            }
            if (!Solve(worldPosition, worldYawDegrees, ReadJointAngles(), out double[] goal))
            {
                EndMove(false, worldPosition);
                yield break;
            }
            int version = moveVersion;
            yield return DrivePath(new List<double[]> { ReadJointAngles(), goal }, version);
            EndMove(version == moveVersion && driveSucceeded, worldPosition);
        }

        // 흡착면 중심을 현재 위치에서 목표까지 직선으로 옮긴다 (접근·상승·하강용). 경로점 하나라도 해가 없으면 출발하지 않는다.
        public IEnumerator MoveLinear(Vector3 worldPosition, float worldYawDegrees)
        {
            if (!BeginMove())
            {
                yield break;
            }

            //1. 경로점마다 이전 해와 가장 가까운 해를 미리 구한다
            Vector3 start = CurrentToolPosition;
            int steps = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(start, worldPosition) / linearStepMeters));
            double[] previous = ReadJointAngles();
            var path = new List<double[]> { previous };
            for (int s = 1; s <= steps; s++)
            {
                if (!Solve(Vector3.Lerp(start, worldPosition, (float)s / steps), worldYawDegrees, previous, out double[] point))
                {
                    EndMove(false, worldPosition);
                    yield break;
                }
                path.Add(point);
                previous = point;
            }

            //2. 경로 전체를 하나의 부드러운 속도 곡선으로 지난다 (경로점마다 멈추지 않는다)
            int version = moveVersion;
            yield return DrivePath(path, version);
            EndMove(version == moveVersion && driveSucceeded, worldPosition);
        }

        // 홈 자세로 관절 이동한다.
        public IEnumerator MoveHome()
        {
            yield return MoveToJointAngles(homeJointAngles);
        }

        // 관절 각도(도, URDF 부호)로 이동한다. 관절 제한 밖이면 움직이지 않는다.
        public IEnumerator MoveToJointAngles(float[] jointAnglesDegrees)
        {
            if (!BeginMove())
            {
                yield break;
            }
            var goal = new double[6];
            for (int i = 0; i < 6; i++)
            {
                goal[i] = jointAnglesDegrees[i] * Mathf.Deg2Rad;
            }
            Vector3 goalPosition = ToolPositionOf(goal);
            if (!RobotArmKinematicsSolver.WithinLimits(goal))
            {
                EndMove(false, goalPosition);
                yield break;
            }
            int version = moveVersion;
            yield return DrivePath(new List<double[]> { ReadJointAngles(), goal }, version);
            EndMove(version == moveVersion && driveSucceeded, goalPosition);
        }

        private bool driveSucceeded;

        // 관절 각도 경로(points[0] = 현재)를 시작·끝이 부드러운 하나의 속도 곡선(3t²−2t³)으로 지나고, 마지막 점 도달을 기다린다.
        // 시작·정지를 부드럽게 해서 흡착 중인 박스에 큰 가속도가 걸리지 않게 한다. 가장 빠른 순간의 관절 속도가 jointSpeedDegrees가 되도록 시간을 정한다.
        private IEnumerator DrivePath(List<double[]> points, int version)
        {
            driveSucceeded = false;

            //1. 구간마다 가장 크게 움직이는 관절 변화로 경로 길이를 잰다
            var cumulative = new double[points.Count];
            for (int k = 1; k < points.Count; k++)
            {
                double largest = 0.0;
                for (int i = 0; i < 6; i++)
                {
                    largest = System.Math.Max(largest, System.Math.Abs(points[k][i] - points[k - 1][i]));
                }
                cumulative[k] = cumulative[k - 1] + largest;
            }
            double total = cumulative[points.Count - 1];
            float duration = Mathf.Max(minimumMoveSeconds, 1.5f * (float)(total * Mathf.Rad2Deg) / jointSpeedDegrees);

            //2. 매 물리 단계 곡선 위치에 해당하는 경로 위 각도를 구동 목표로 보낸다
            double[] goal = points[points.Count - 1];
            int segment = 1;
            for (float elapsed = 0f; elapsed < duration; )
            {
                if (version != moveVersion)
                {
                    yield break;
                }
                elapsed += Time.fixedDeltaTime;
                float t = Mathf.Clamp01(elapsed / duration);
                double along = total * (t * t * (3f - 2f * t));
                while (segment < points.Count - 1 && cumulative[segment] < along)
                {
                    segment++;
                }
                double span = cumulative[segment] - cumulative[segment - 1];
                double f = span > 1e-12 ? (along - cumulative[segment - 1]) / span : 1.0;
                for (int i = 0; i < 6; i++)
                {
                    double angle = points[segment - 1][i] + (points[segment][i] - points[segment - 1][i]) * f;
                    SetDriveTarget(i, angle - referenceOffsets[i]);
                }
                yield return new WaitForFixedUpdate();
            }

            //3. 실제 관절이 목표 오차 안에 들어올 때까지 기다린다
            for (float waited = 0f; waited < settleTimeoutSeconds; waited += Time.fixedDeltaTime)
            {
                if (version != moveVersion)
                {
                    yield break;
                }
                if (WithinTolerance(goal))
                {
                    driveSucceeded = true;
                    yield break;
                }
                yield return new WaitForFixedUpdate();
            }
            driveSucceeded = WithinTolerance(goal);
        }

        private bool WithinTolerance(double[] goal)
        {
            double[] actual = ReadJointAngles();
            for (int i = 0; i < 6; i++)
            {
                if (System.Math.Abs(actual[i] - goal[i]) * Mathf.Rad2Deg > reachToleranceDegrees)
                {
                    return false;
                }
            }
            return true;
        }

        // 이미 이동 중이면 새 이동을 받지 않는다
        private bool BeginMove()
        {
            if (IsMoving)
            {
                LastMoveSucceeded = false;
                return false;
            }
            IsMoving = true;
            return true;
        }

        private void EndMove(bool succeeded, Vector3 target)
        {
            IsMoving = false;
            LastMoveSucceeded = succeeded;
            LastReachError = Vector3.Distance(CurrentToolPosition, target);
        }

        private void SetDriveTarget(int index, double articulationAngle)
        {
            if (joints[index] == null)
            {
                return;
            }
            ArticulationDrive drive = joints[index].xDrive;
            drive.target = (float)(articulationAngle * Mathf.Rad2Deg);
            joints[index].xDrive = drive;
        }
        #endregion
    }
}
