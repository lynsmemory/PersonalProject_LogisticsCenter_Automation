using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Logistics
{
    // 전담 레인의 완료 신호를 받으면 경로를 탐색해 이동하고 장애물이면 정지하고, 장애물이 사라진 뒤 5초가 지나면 재개한다.
    // 완성 파레트를 인양해 전담 도크 차량에 상차하고 실패하면 재시도한다. (FR-13, FR-14, FR-15)
    // 순서: 대기점 → 인양 위치(직선 진입) → 대기점(후진) → [경로 탐색] 도크 접근점 → [경로 탐색] 적재 자리 정렬점 → 적재 자리(직선 진입)
    //       → 정렬점(후진) → 도크 접근점(후진) → [경로 탐색] 대기점.
    // 경로 탐색은 ForkliftPathPlanner(바닥 격자 A*)가 한다. 설계의 NavMeshAgent는 씬에 베이크된 NavMesh가 없어 쓰지 않는다.
    //   경로가 없으면 replanIntervalSeconds마다 다시 탐색하고 maximumPlanAttempts번 실패하면 고장 정지(도달 불가)한다.
    // 장애물: 점유 상자를 차체 / 포크(등판·왼쪽 날·오른쪽 날) / 화물로 나눠, 진행 방향 앞쪽(lookAheadDistance)·회전할 각도 범위에서 조회한다.
    //   자기 차체·운반물·트리거·바닥면(높이 floorTolerance 이하)을 뺀 모든 충돌체가 장애물이다 (박스·설비·차량·적재된 파레트·다른 AMR 포함).
    //   예외는 포크 날이 정상으로 드나드는 파레트 하나뿐이다: 인양 진입 때 대상 파레트와 그 박스, 상차 직후 인출 때 방금 내려놓은 파레트와 그 박스.
    //   이 예외는 포크 부분 상자에만 적용하고 차체·화물 상자는 그대로 검사한다. 인출 예외는 포크 상자가 그 파레트에서 빠지면 바로 끝난다.
    //   막히면 정지하고, 비고 나서 resumeDelaySeconds가 지나면 재개한다. 경로 구간에서 replanAfterBlockedSeconds 넘게 막히면 현재 장애물을 반영해 다시 탐색한다.
    // 지게차는 URDF ArticulationBody 모델이다. 바퀴 구동 주행 대신 관절 루트를 immovable로 고정하고 매 물리 단계 TeleportRoot로 경로 위 자세를 준다.
    // 포크 승강은 포크 관절 드라이브 목표로 움직이고, 인양한 파레트와 그 위 박스는 kinematic으로 바꿔 포크 링크에 붙여 운반한다 (포크·파레트 접촉 물리로 들지 않는다).
    public class AutonomousForkliftController : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("전담 레인 번호 (표시·기록용)")]
        [SerializeField] private DestinationLane lane = DestinationLane.Lane1;

        [Tooltip("지게차 모델 루트 (AMRForklifts/AMR_LaneN)")]
        [SerializeField] private Transform vehicle;

        [Tooltip("대기 위치. 위치·방향 (방향은 파레트를 바라보는 쪽)")]
        [SerializeField] private Transform homePoint;

        [Tooltip("전담 도크 접근점 (차량 뒤쪽 바깥)")]
        [SerializeField] private Transform dockApproachPoint;

        [Tooltip("포크 승강 관절 링크 이름")]
        [SerializeField] private string forkLinkName = "compact_forklift_fork_root_link";

        [Tooltip("인양 위치: 포크 등판 앞면과 파레트 앞면 사이 간격 (m). 날은 등판 앞면부터 약 0.385m (fork_mockup × 축척 0.4)")]
        [SerializeField, Min(0f)] private float backrestClearance = 0.01f;

        [Tooltip("포크 관절 목표: 파레트 아래로 들어가고 내려놓는 높이 (관절 드라이브 단위, 모델 초기값 0.05)")]
        [SerializeField] private float forkEntryTarget = 0.05f;

        [Tooltip("포크 관절 목표: 인양 높이 (관절 상한 0.2)")]
        [SerializeField] private float forkLiftTarget = 0.2f;

        [Tooltip("포크 승강 대기 시간 (초)")]
        [SerializeField, Min(0.1f)] private float forkMoveSeconds = 1.5f;

        [Tooltip("주행 속도 (m/s)")]
        [SerializeField, Min(0.05f)] private float travelSpeed = 0.6f;

        [Tooltip("파레트·적재 자리 진입 속도 (m/s)")]
        [SerializeField, Min(0.02f)] private float approachSpeed = 0.25f;

        [Tooltip("제자리 회전 속도 (도/초)")]
        [SerializeField, Min(5f)] private float turnSpeedDegrees = 90f;

        [Tooltip("적재 자리 정렬점: 적재함 뒤쪽 끝에서 바깥으로 떨어진 거리 (m)")]
        [SerializeField, Min(0.1f)] private float truckEntryMargin = 0.6f;

        [Tooltip("적재함 바닥 위 차체 높이 여유 (m). 대기 위치 높이와 같은 값")]
        [SerializeField, Min(0f)] private float groundClearance = 0.005f;

        [Header("경로 탐색")]
        [Tooltip("격자 칸 크기 (m)")]
        [SerializeField, Min(0.02f)] private float pathCellSize = 0.1f;

        [Tooltip("차체·화물 반폭에 더하는 통로 여유 (m)")]
        [SerializeField, Min(0f)] private float pathClearance = 0.05f;

        [Tooltip("출발·목표를 감싼 사각형 바깥 탐색 여유 (m)")]
        [SerializeField, Min(0.5f)] private float pathSearchMargin = 4f;

        [Tooltip("경로 탐색 시도 횟수. 모두 실패하면 도달 불가로 고장 정지")]
        [SerializeField, Min(1)] private int maximumPlanAttempts = 5;

        [Tooltip("경로가 없을 때 다시 탐색하기까지 대기 (초)")]
        [SerializeField, Min(0.1f)] private float replanIntervalSeconds = 3f;

        [Header("장애물")]
        [Tooltip("진행 방향 앞쪽 점유 검사 거리 (m)")]
        [SerializeField, Min(0.05f)] private float lookAheadDistance = 0.3f;

        [Tooltip("회전할 때 앞서 검사하는 각도 (도)")]
        [SerializeField, Min(1f)] private float lookAheadDegrees = 5f;

        [Tooltip("바닥면으로 보는 높이: 충돌체 윗면이 차체 기준 이 높이 이하이면 장애물이 아님 (m)")]
        [SerializeField, Min(0.005f)] private float floorTolerance = 0.04f;

        [Tooltip("장애물이 사라진 뒤 재개까지 대기 시간 (초). 설계 제안값, SRS상 재개 조건 확인 필요")]
        [SerializeField, Min(0f)] private float resumeDelaySeconds = 5f;

        [Tooltip("경로 구간에서 이 시간(초) 넘게 막히면 현재 장애물을 반영해 경로를 다시 탐색")]
        [SerializeField, Min(1f)] private float replanAfterBlockedSeconds = 10f;

        [Tooltip("파레트·적재 자리 직선 진입·후퇴가 이 시간(초) 넘게 막히면 고장 정지")]
        [SerializeField, Min(1f)] private float dockingBlockTimeoutSeconds = 30f;

        [Tooltip("반출 완료 판정 여유 (m): 차체 위치가 완성 파레트 자리 중심에서 (차체·포크·화물 회전 반경 + 파레트 자리 반경 + 이 값) 이상 떨어지면 빈 파레트를 만들게 알린다")]
        [SerializeField, Min(0f)] private float removalMargin = 0.1f;

        [Header("상차 판정")]
        [Tooltip("상차 재시도 횟수 (시험 후 확정)")]
        [SerializeField, Min(0)] private int maximumLoadRetries = 3;

        [Tooltip("상차 재시도 간격 (초, 시험 후 확정)")]
        [SerializeField, Min(0f)] private float retryIntervalSeconds = 2f;

        [Tooltip("파레트 중심과 적재 자리의 수평 거리 허용 (m)")]
        [SerializeField, Min(0.005f)] private float placeTolerance = 0.03f;

        [Tooltip("파레트 바닥과 적재 자리 높이 차 허용 (m)")]
        [SerializeField, Min(0.001f)] private float heightTolerance = 0.015f;

        [Tooltip("파레트 방향 허용 (도, 적재 자리 축과 90도 단위로 비교)")]
        [SerializeField, Min(0.1f)] private float yawTolerance = 3f;

        [Tooltip("파레트 기울기 허용 (도)")]
        [SerializeField, Min(0.1f)] private float tiltTolerance = 2f;

        [Tooltip("파레트 위 박스 기울기 허용 (도, PalletizingSequence 적재 확인과 같은 5도)")]
        [SerializeField, Min(0.1f)] private float cargoTiltTolerance = 5f;

        [Tooltip("포크 관절 실제 위치와 목표 차이 허용 (관절 단위)")]
        [SerializeField, Min(0.0005f)] private float forkPositionTolerance = 0.01f;

        [Tooltip("시험용: 다음 상차 시도 중 이 횟수만큼 강제로 실패시킨다 (TC-15 재시도 확인용, 평소 0)")]
        [SerializeField, Min(0)] private int forcedLoadFailures = 0;
        #endregion

        #region 2. 상태
        // 모델 루트 기준(축척 없는) 점유 상자
        private struct LocalBox
        {
            public Vector3 Centre;
            public Vector3 Half;
        }

        private static readonly List<AutonomousForkliftController> all = new List<AutonomousForkliftController>();
        private Collider[] hits = new Collider[64];
        private readonly WaitForFixedUpdate waitStep = new WaitForFixedUpdate();
        private readonly List<Transform> forkPassRoots = new List<Transform>();   // 포크 날이 정상으로 드나드는 파레트·박스 (포크 상자에만 예외)
        private bool forkWithdrawing;   // 상차 직후 포크 인출 중: 포크 상자가 예외 파레트에서 빠지면 예외 끝
        private ForkliftPathPlanner planner;

        private ArticulationBody rootBody;
        private ArticulationBody forkBody;
        private Transform forkLink;
        private Vector3 rootLocalPosition;
        private Quaternion rootLocalRotation;
        private float initialForkTarget;
        private Vector3 startPosition;
        private Quaternion startRotation;
        private LocalBox chassisBox;                          // 포크를 뺀 차체
        private readonly LocalBox[] forkBoxes = new LocalBox[3];   // 등판, 왼쪽 날, 오른쪽 날 (Start 때 포크 높이 기준)
        private float forkHeightAtStart;
        private LocalBox loadBox;
        private float palletSiteRadius = 0.31f;   // 완성 파레트 자리 반경 (파레트 반 대각선)
        private bool initialized;

        private Vector3 position;   // 경로 위 현재 위치 (모델 루트)
        private float yaw;          // 현재 방향 (도)
        private Coroutine routine;
        private Transform carried;  // 들고 있는 파레트
        private float carriedReach; // 모델 루트에서 들고 있는 파레트 중심까지 앞쪽 거리 (m)
        private int carriedBoxCount;
        private PalletLoadState targetPallet;
        private DockVehicleManager targetDock;
        private bool removalPending;
        private Vector3 removedFrom;
        private bool moveGaveUp;    // 마지막 이동이 오래 막혀 중단됨 (다시 탐색 필요)
        private Transform lastPlaced;   // 방금 상차한 파레트 (포크 인출 예외 대상)
        private struct CarriedPose
        {
            public Rigidbody Body;
            public Vector3 Position;
            public Quaternion Rotation;
        }
        private readonly List<CarriedPose> carriedPoses = new List<CarriedPose>();

        public DestinationLane Lane => lane;
        public ForkliftState State { get; private set; } = ForkliftState.Idle;
        public bool IsBusy => routine != null;
        public bool IsHalted => State == ForkliftState.Faulted;
        // 새 운반 요청을 받을 수 있음
        public bool IsAvailable => initialized && routine == null && !IsHalted;
        public string FaultReason { get; private set; }     // 고장 정지 이유 (도달 불가·상차 재시도 초과·화물 수 불일치)
        public int LoadRetryCount { get; private set; }     // 이번 상차의 재시도 수
        public int DeliveredCount { get; private set; }     // 상차 성공 파레트 수
        public int ObstacleStopCount { get; private set; }  // 장애물 정지 횟수
        public int ReplanCount { get; private set; }        // 경로 재탐색 횟수
        public string LastObstacleName { get; private set; }
        public string LastPlanResult { get; private set; }  // 마지막 경로 탐색 결과 (경유점 수 또는 실패 이유)

        public event Action PalletRemoved;              // 완성 파레트를 적재 위치에서 반출 (빈 파레트 생성용)
        public event Action<ForkliftState> StateChanged;
        #endregion

        #region 3. 준비
        private void OnEnable()
        {
            if (!all.Contains(this)) all.Add(this);
        }

        private void OnDisable()
        {
            all.Remove(this);
        }

        private void Start()
        {
            if (vehicle == null || homePoint == null || dockApproachPoint == null)
            {
                Debug.LogWarning($"[AutonomousForkliftController {lane}] 지게차·대기 위치·도크 접근점 연결이 없음", this);
                return;
            }
            foreach (ArticulationBody body in vehicle.GetComponentsInChildren<ArticulationBody>())
            {
                if (body.isRoot && rootBody == null) rootBody = body;
                if (body.name == forkLinkName) forkBody = body;
            }
            if (rootBody == null || forkBody == null)
            {
                Debug.LogWarning($"[AutonomousForkliftController {lane}] 지게차 관절 루트 또는 포크 링크({forkLinkName})를 찾지 못함", this);
                return;
            }
            forkLink = forkBody.transform;
            initialForkTarget = forkBody.xDrive.target;

            // 관절 루트를 고정하고 모델 루트 기준 상대 자세를 저장해 두었다가 매 단계 같이 옮긴다
            rootLocalPosition = vehicle.InverseTransformPoint(rootBody.transform.position);
            rootLocalRotation = Quaternion.Inverse(vehicle.rotation) * rootBody.transform.rotation;
            rootBody.immovable = true;
            startPosition = vehicle.position;
            startRotation = vehicle.rotation;
            position = startPosition;
            yaw = vehicle.eulerAngles.y;
            MeasureFootprint();
            planner = new ForkliftPathPlanner(pathCellSize, other => IsObstacle(other, false));
            initialized = true;
        }
        #endregion

        #region 4. 운반
        // 완료 신호 수신 후 운반 시작. 이미 운반 중이거나 고장 정지 상태면 false (레인이 다시 요청한다)
        public bool RequestPickup(PalletLoadState pallet, DockVehicleManager dock)
        {
            if (!IsAvailable || pallet == null || dock == null || pallet.CurrentPallet == null)
            {
                return false;
            }
            targetPallet = pallet;
            targetDock = dock;
            LoadRetryCount = 0;
            FaultReason = null;
            routine = StartCoroutine(TransportRoutine());
            return true;
        }

        private IEnumerator TransportRoutine()
        {
            //1. 인양 위치로 직선 진입. 포크 등판이 파레트 앞면 backrestClearance 앞에 서는 자리 (날은 파레트 아래).
            //   인양할 파레트와 그 위 박스는 정상 진입 대상이라 장애물에서 뺀다
            SetState(ForkliftState.MovingToPallet);
            Transform palletTransform = targetPallet.CurrentPallet;
            Vector3 home = homePoint.position;
            Vector3 toPallet = Flat(palletTransform.position - home);
            Vector3 pick = palletTransform.position - toPallet * (BackrestFront + HalfDepth(palletTransform, toPallet) + backrestClearance);
            pick.y = home.y;
            List<LogisticsBox> cargo = CollectPalletBoxes(palletTransform);
            SetForkPass(palletTransform, cargo, false);
            yield return MoveFork(forkEntryTarget);
            yield return Drive(pick, false, approachSpeed, false);
            if (IsHalted) { Finish(); yield break; }

            //2. 인양 전 화물 확인: 파레트 위 박스 수가 완료 적재 수와 같아야 한다. 다르면 인양하지 않고 물러나 정지
            SetState(ForkliftState.Lifting);
            cargo = CollectPalletBoxes(palletTransform);
            int expected = targetPallet.PlacedCount;
            if (cargo.Count != expected)
            {
                forkWithdrawing = true;
                yield return Drive(home, true, approachSpeed, false);
                ClearForkPass();
                Fault($"화물 수 불일치 (파레트 위 {cargo.Count}개 / 적재 완료 {expected}개). 인양 중단");
                Finish();
                yield break;
            }
            carried = targetPallet.TakeCurrentPallet();
            ClearForkPass();
            AttachCarried(carried, cargo);
            yield return MoveFork(forkLiftTarget);

            //3. 대기점으로 후진. 차체·포크·화물의 회전 범위가 완성 파레트 자리를 벗어나면 빈 파레트를 만들게 알린다 (NotifyRemovedIfPending)
            removalPending = true;
            removedFrom = palletTransform.position;
            yield return Drive(home, true, approachSpeed, false);
            if (IsHalted) { Finish(); yield break; }

            //4. 전담 도크로 경로 탐색 이동. 차량 교체 중이면 대기점에서 기다린다
            SetState(ForkliftState.MovingToDock);
            while (!targetDock.CanAcceptPallet)
            {
                yield return waitStep;
            }
            Transform slot = targetDock.GetNextCargoSlot();
            Vector3 approach = dockApproachPoint.position;
            approach.y = home.y;
            yield return Travel(approach, travelSpeed);
            if (IsHalted) { Finish(); yield break; }
            NotifyRemovedIfPending();
            if (removalPending)
            {
                // 도크 접근점이 완성 파레트 자리와 너무 가까운 배치: 강제로 반출 처리하지 않고 기록만 남긴다 (레인은 빈 파레트 없이 대기)
                Debug.LogWarning($"[AutonomousForkliftController {lane}] 도크 접근점에서도 반출 이격 조건 미충족. 빈 파레트를 만들지 않음 (배치 확인 필요)", this);
            }

            //5. 적재 자리 정렬점까지 경로 탐색 → 정렬 → 적재 자리 직선 진입 → 내려놓기·확인. 실패하면 물러났다가 다시 시도
            Vector3 inDirection = targetDock.InDirection;
            Vector3 load = slot.position - inDirection * carriedReach;
            load.y = slot.position.y + groundClearance;
            Vector3 entry = load - inDirection * (Vector3.Dot(load - targetDock.RearPoint, inDirection) + truckEntryMargin);
            entry.y = home.y;
            yield return Travel(entry, travelSpeed);
            if (IsHalted) { Finish(); yield break; }
            yield return Turn(Yaw(inDirection), false);
            if (IsHalted) { Finish(); yield break; }
            while (true)
            {
                SetState(ForkliftState.Loading);
                yield return Drive(load, false, approachSpeed, false);
                if (IsHalted) { Finish(); yield break; }
                yield return MoveFork(forkEntryTarget);
                if (TryPlace(slot, load, out string reason))
                {
                    break;
                }
                LoadRetryCount++;
                Debug.LogWarning($"[AutonomousForkliftController {lane}] 상차 실패 ({reason}). 재시도 {LoadRetryCount}/{maximumLoadRetries}", this);
                yield return MoveFork(forkLiftTarget);
                yield return Drive(entry, true, approachSpeed, false);
                if (IsHalted) { Finish(); yield break; }
                if (LoadRetryCount > maximumLoadRetries)
                {
                    // 반복 실패 처리는 SRS상 시험 후 확정: 파레트를 든 채 정렬점에서 정지하고 초기화를 기다린다
                    Fault($"상차 재시도 {maximumLoadRetries}회 초과 (마지막: {reason})");
                    Finish();
                    yield break;
                }
                yield return new WaitForSeconds(retryIntervalSeconds);
            }
            DeliveredCount++;

            //6. 복귀: 방금 내려놓은 파레트에서 포크를 인출(그 파레트만 포크 상자 예외, 빠지면 예외 끝)하고 적재함을 빠져나와
            //   도크 접근점에서 차량에 알리고, 경로 탐색으로 대기점까지
            SetState(ForkliftState.Returning);
            SetForkPass(lastPlaced, new List<LogisticsBox>(lastPlaced.GetComponentsInChildren<LogisticsBox>()), true);
            yield return Drive(entry, true, approachSpeed, false);
            ClearForkPass();
            if (IsHalted) { Finish(); yield break; }
            yield return Drive(approach, true, approachSpeed, false);
            if (IsHalted) { Finish(); yield break; }
            targetDock.LoaderCleared();
            yield return Travel(home, travelSpeed);
            if (IsHalted) { Finish(); yield break; }
            yield return Turn(Yaw(toPallet), false);
            if (IsHalted) { Finish(); yield break; }
            SetState(ForkliftState.Idle);
            Finish();
        }

        // 작업 종료 (정상·고장 공통). 반출 알림은 보내지 않는다: 반출 성공은 NotifyRemovedIfPending의 이격 조건으로만 판정한다
        private void Finish()
        {
            ClearForkPass();
            routine = null;
        }

        // 운반 중 고장 정지 (도달 불가·상차 재시도 초과·화물 수 불일치). 초기화 전까지 새 요청을 받지 않는다
        private void Fault(string reason)
        {
            FaultReason = reason;
            SetState(ForkliftState.Faulted);
            Debug.LogWarning($"[AutonomousForkliftController {lane}] 고장 정지: {reason} (초기화 필요)", this);
        }

        // 완성 파레트를 들고 있고, 차체·포크·화물이 어느 방향으로 회전해도 완성 파레트 자리(중심 removedFrom, 반경 palletSiteRadius)에
        // 닿지 않는 거리까지 벗어났을 때만 반출 완료를 한 번 알린다. 고장·초기화·작업 종료는 반출로 보지 않는다
        private void NotifyRemovedIfPending()
        {
            if (!removalPending || carried == null || IsHalted) return;
            Vector3 d = position - removedFrom;
            d.y = 0f;
            Vector3 c = carried.position - removedFrom;
            c.y = 0f;
            float required = RotationRadius + palletSiteRadius + removalMargin;
            if (d.magnitude < required || c.magnitude < palletSiteRadius * 2f + removalMargin) return;
            removalPending = false;
            Debug.Log($"[AutonomousForkliftController {lane}] 완성 파레트 반출 (자리에서 {d.magnitude:F2}m, 필요 {required:F2}m)", this);
            PalletRemoved?.Invoke();
        }

        // 포크 날이 정상으로 드나드는 파레트 하나와 그 박스를 포크 상자 검사에서 뺀다 (withdrawing = 인출, 빠지면 예외 끝)
        private void SetForkPass(Transform pallet, List<LogisticsBox> boxes, bool withdrawing)
        {
            forkPassRoots.Clear();
            if (pallet != null) forkPassRoots.Add(pallet);
            foreach (LogisticsBox box in boxes) forkPassRoots.Add(box.transform);
            forkWithdrawing = withdrawing;
        }

        private void ClearForkPass()
        {
            forkPassRoots.Clear();
            forkWithdrawing = false;
        }

        // 포크 등판 앞면까지의 거리 (모델 루트 기준 앞쪽, m). 등판 상자(forkBoxes[0]) 앞면
        private float BackrestFront => forkBoxes[0].Centre.z + forkBoxes[0].Half.z;

        // 파레트 충돌체 전체의 진행 방향 반 길이 (m)
        private static float HalfDepth(Transform pallet, Vector3 direction)
        {
            Collider[] parts = pallet.GetComponentsInChildren<Collider>();
            if (parts.Length == 0) return 0.23f;
            Bounds b = parts[0].bounds;
            foreach (Collider part in parts) b.Encapsulate(part.bounds);
            return Mathf.Abs(b.extents.x * direction.x) + Mathf.Abs(b.extents.z * direction.z);
        }

        // 파레트 위 박스 수집: 파레트 윗면 460×400mm 범위 위의 상자를 조회하고, 조회 버퍼가 차면 늘려서 다시 조회한다.
        // 한 박스의 여러 충돌체는 한 번만 센다. 흡착 중인 박스와 중심이 파레트 범위 밖인 박스(옆 픽업 구역 등)는 뺀다
        private List<LogisticsBox> CollectPalletBoxes(Transform pallet)
        {
            var found = new HashSet<LogisticsBox>();
            int n = OverlapAll(pallet.position + Vector3.up * 0.8f, new Vector3(0.25f, 0.75f, 0.22f), pallet.rotation);
            for (int i = 0; i < n; i++)
            {
                var box = hits[i].GetComponentInParent<LogisticsBox>();
                if (box == null || box.IsHeld || box.Body == null) continue;
                Vector3 local = pallet.InverseTransformPoint(box.Body.worldCenterOfMass);
                if (Mathf.Abs(local.x) > 0.25f || Mathf.Abs(local.z) > 0.22f || local.y < 0.1f) continue;
                found.Add(box);
            }
            return new List<LogisticsBox>(found);
        }

        // 파레트와 박스를 kinematic으로 바꿔 포크 링크에 붙인다
        private void AttachCarried(Transform pallet, List<LogisticsBox> cargo)
        {
            foreach (LogisticsBox box in cargo)
            {
                box.Body.isKinematic = true;
                box.transform.SetParent(pallet, true);
            }
            carriedBoxCount = cargo.Count;
            palletSiteRadius = PalletHalfDiagonal(pallet);
            var body = pallet.GetComponent<Rigidbody>();
            if (body != null) body.isKinematic = true;
            pallet.SetParent(forkLink, true);
            carriedPoses.Clear();
            foreach (Rigidbody loadBody in pallet.GetComponentsInChildren<Rigidbody>())
            {
                carriedPoses.Add(new CarriedPose
                {
                    Body = loadBody,
                    Position = forkLink.InverseTransformPoint(loadBody.transform.position),
                    Rotation = Quaternion.Inverse(forkLink.rotation) * loadBody.transform.rotation
                });
            }
            Vector3 reach = pallet.position - vehicle.position;
            reach.y = 0f;
            carriedReach = Vector3.Dot(reach, vehicle.forward);
            // kinematic 파레트가 포크·차체 관절 링크를 밀어내지 않도록 지게차 충돌체와의 충돌을 끈다
            Collider[] own = vehicle.GetComponentsInChildren<Collider>();
            Collider[] loadParts = pallet.GetComponentsInChildren<Collider>();
            foreach (Collider part in loadParts)
            {
                foreach (Collider ownPart in own)
                {
                    if (!ownPart.isTrigger && !part.isTrigger) Physics.IgnoreCollision(part, ownPart, true);
                }
            }
            // 바닥에 놓인 상태로 잰 화물 범위에 인양 높이(관절 상한 0.2)만큼 위쪽 여유를 더한다
            loadBox = MeasureLocalBox(loadParts, true);
            loadBox.Centre.y += 0.1f;
            loadBox.Half.y += 0.1f;
        }

        // 상차 성공 판정: 차체·포크 도달, 파레트 위치·높이·방향·기울기, 화물 수·자세, 자리 비어 있음을 모두 확인한다.
        // 위치를 고쳐 놓지 않는다 (실패는 재시도로 보낸다). 성공하면 지금 자세 그대로 적재 자리에 붙이고 차량에 등록한다
        private bool TryPlace(Transform slot, Vector3 loadPose, out string reason)
        {
            SyncCarriedPose();
            Physics.SyncTransforms();
            reason = null;
            if (carried == null)
            {
                reason = "파레트 없음";
                return false;
            }
            if (forcedLoadFailures > 0)
            {
                forcedLoadFailures--;
                reason = "시험용 강제 실패";
                return false;
            }
            if ((position - loadPose).magnitude > 0.01f)
            {
                reason = $"차체 미도달 {(position - loadPose).magnitude * 1000f:F0}mm";
                return false;
            }
            float fork = forkBody.jointPosition.dofCount > 0 ? forkBody.jointPosition[0] : float.NaN;
            if (float.IsNaN(fork) || Mathf.Abs(fork - forkEntryTarget) > forkPositionTolerance)
            {
                reason = $"포크 미도달 (관절 {fork:F3} / 목표 {forkEntryTarget:F3})";
                return false;
            }
            Vector3 offset = carried.position - slot.position;
            float vertical = offset.y;
            offset.y = 0f;
            if (offset.magnitude > placeTolerance)
            {
                reason = $"자리 수평 오차 {offset.magnitude * 1000f:F0}mm";
                return false;
            }
            if (Mathf.Abs(vertical) > heightTolerance)
            {
                reason = $"파레트 높이 오차 {vertical * 1000f:F0}mm";
                return false;
            }
            float yawError = Mathf.Abs(Mathf.DeltaAngle(0f, Mathf.Repeat(carried.eulerAngles.y - slot.eulerAngles.y + 45f, 90f) - 45f));
            if (yawError > yawTolerance)
            {
                reason = $"파레트 방향 오차 {yawError:F1}°";
                return false;
            }
            float tilt = Vector3.Angle(carried.up, Vector3.up);
            if (tilt > tiltTolerance)
            {
                reason = $"파레트 기울기 {tilt:F1}°";
                return false;
            }
            LogisticsBox[] boxes = carried.GetComponentsInChildren<LogisticsBox>();
            if (boxes.Length != carriedBoxCount)
            {
                reason = $"화물 수 변화 ({boxes.Length}/{carriedBoxCount})";
                return false;
            }
            foreach (LogisticsBox box in boxes)
            {
                Vector3 local = carried.InverseTransformPoint(box.transform.position);
                float boxTilt = Vector3.Angle(box.transform.up, carried.up);
                if (Mathf.Abs(local.x) > 0.25f || Mathf.Abs(local.z) > 0.22f || boxTilt > cargoTiltTolerance)
                {
                    reason = $"화물 자세 이상 ({box.name}, 기울기 {boxTilt:F1}°)";
                    return false;
                }
            }
            int n = OverlapAll(slot.position + Vector3.up * 0.09f, new Vector3(0.22f, 0.06f, 0.19f), carried.rotation);
            for (int i = 0; i < n; i++)
            {
                Transform hit = hits[i].transform;
                if (hits[i].isTrigger || hit.IsChildOf(carried) || hit.IsChildOf(vehicle)) continue;
                // 차량 구조(적재함 바닥 등)는 빼고, 차량에 실린 다른 파레트·박스는 점유로 본다
                if (targetDock.Truck != null && hit.IsChildOf(targetDock.Truck) && hits[i].attachedRigidbody == null) continue;
                reason = $"적재 자리에 다른 물체 ({hits[i].name})";
                return false;
            }
            carried.SetParent(slot, true);
            targetDock.RegisterLoaded(carried);
            lastPlaced = carried;
            carried = null;
            carriedPoses.Clear();
            carriedBoxCount = 0;
            return true;
        }
        #endregion

        #region 5. 경로 탐색·이동
        // 목표까지 경로를 탐색해 따라간다. 경로가 없으면 다시 탐색하고, 시도를 모두 실패하면 도달 불가로 고장 정지한다.
        // 따라가다 replanAfterBlockedSeconds 넘게 막히면 현재 위치에서 다시 탐색한다 (막은 물체가 격자에 반영된다)
        private IEnumerator Travel(Vector3 goal, float speed)
        {
            int failures = 0;
            var path = new List<Vector3>();
            while (true)
            {
                if (!PlanRoute(goal, path, out string reason))
                {
                    failures++;
                    LastPlanResult = reason;
                    Debug.LogWarning($"[AutonomousForkliftController {lane}] 경로 없음 ({reason}). 탐색 {failures}/{maximumPlanAttempts}", this);
                    if (failures >= maximumPlanAttempts)
                    {
                        Fault($"도달 불가: {reason}");
                        yield break;
                    }
                    ForkliftState previous = State;
                    SetState(ForkliftState.WaitingForRoute);
                    yield return new WaitForSeconds(replanIntervalSeconds);
                    SetState(previous);
                    continue;
                }
                LastPlanResult = $"경유점 {path.Count}개";
                bool gaveUp = false;
                foreach (Vector3 waypoint in path)
                {
                    yield return Drive(waypoint, false, speed, true);
                    if (IsHalted) yield break;
                    if (moveGaveUp) { gaveUp = true; break; }
                }
                if (!gaveUp)
                {
                    yield break;
                }
                ReplanCount++;
                Debug.Log($"[AutonomousForkliftController {lane}] 장애물이 계속 막아 경로 재탐색", this);
            }
        }

        // 현재 위치·방향에서 목표까지 경로 탐색. 각 경유점의 제자리 회전 공간도 확인하고, 막히면 그 지점을 막아 다시 탐색한다
        private bool PlanRoute(Vector3 goal, List<Vector3> path, out string reason)
        {
            var settings = new ForkliftPathPlanner.Settings
            {
                HalfWidth = FootprintHalfWidth + pathClearance,
                BandBottom = position.y + floorTolerance,
                BandTop = position.y + FootprintTop,
                SearchMargin = pathSearchMargin,
                MaxCells = 400000,
            };
            var blocked = new List<Vector3>();
            for (int attempt = 0; attempt < 6; attempt++)
            {
                if (!planner.TryPlan(position, goal, settings, SegmentClear, blocked, path, out reason))
                {
                    return false;
                }
                // 회전은 출발점(-1)과 각 경유점에서 일어난다. 회전 공간이 없는 경유점은 막고 다시 탐색한다
                Vector3 from = position;
                float heading = yaw;
                int turnAt = -1, failedAt = int.MinValue;
                for (int i = 0; i < path.Count; i++)
                {
                    Vector3 d = path[i] - from;
                    d.y = 0f;
                    if (d.magnitude < 0.01f) continue;
                    float next = Yaw(d);
                    if (!RotationClear(from, heading, next, out _)) { failedAt = turnAt; break; }
                    from = path[i];
                    heading = next;
                    turnAt = i;
                }
                if (failedAt == int.MinValue)
                {
                    reason = null;
                    return true;
                }
                if (failedAt < 0)
                {
                    reason = "출발 지점에서 회전할 공간 없음";
                    return false;
                }
                blocked.Add(path[failedAt]);
            }
            reason = "경유점 회전 공간을 찾지 못함";
            return false;
        }

        // 목표 방향으로 제자리 회전. 다음 각도에서 차체·화물 점유를 검사하고 막히면 정지한다
        private IEnumerator Turn(float targetYaw, bool canGiveUp)
        {
            moveGaveUp = false;
            while (Mathf.Abs(Mathf.DeltaAngle(yaw, targetYaw)) > 0.1f)
            {
                float next = Mathf.MoveTowardsAngle(yaw, targetYaw, turnSpeedDegrees * Time.fixedDeltaTime);
                // 정지 원인과 해제 조건을 같은 검사로 본다: 지금 각도에서 (다음 단계, lookAheadDegrees) 중 큰 쪽까지 1도 간격 연속 검사
                string obstacle = TurnBlocked(targetYaw);
                if (obstacle != null)
                {
                    yield return WaitForClear(() => TurnBlocked(targetYaw), obstacle, canGiveUp);
                    if (moveGaveUp || IsHalted) yield break;
                    continue;
                }
                yaw = next;
                Place();
                yield return waitStep;
            }
            yaw = targetYaw;
            Place();
        }

        // 목표를 향해 회전 후 직선 이동 (reverse = 뒤로 가기, 방향은 목표 반대쪽).
        // 진행 방향 lookAheadDistance(남은 거리까지) 앞의 차체·화물 점유를 검사하고 막히면 정지한다
        private IEnumerator Drive(Vector3 target, bool reverse, float speed, bool canGiveUp)
        {
            moveGaveUp = false;
            Vector3 planar = target - position;
            planar.y = 0f;
            if (planar.magnitude > 0.01f)
            {
                yield return Turn(Yaw(reverse ? -planar : planar), canGiveUp);
                if (moveGaveUp || IsHalted) yield break;
            }
            while ((target - position).sqrMagnitude > 1e-6f)
            {
                Vector3 ahead = AheadPose(target);
                if (PoseBlocked(ahead, yaw, out string obstacle))
                {
                    yield return WaitForClear(() => PoseBlocked(AheadPose(target), yaw, out string o) ? o : null, obstacle, canGiveUp);
                    if (moveGaveUp || IsHalted) yield break;
                    continue;
                }
                position = Vector3.MoveTowards(position, target, speed * Time.fixedDeltaTime);
                Place();
                NotifyRemovedIfPending();
                // 상차 직후 인출: 포크 상자가 예외 파레트에서 완전히 빠지면 예외를 끝낸다 (이후 그 파레트도 장애물)
                if (forkWithdrawing && !ForkOverlapsPassRoots()) ClearForkPass();
                yield return waitStep;
            }
        }

        // 회전 점유 검사: 현재 각도부터 목표 방향으로 max(한 단계, lookAheadDegrees)까지 1도 간격. 막은 물체 이름, 비어 있으면 null
        private string TurnBlocked(float targetYaw)
        {
            float remaining = Mathf.Abs(Mathf.DeltaAngle(yaw, targetYaw));
            float span = Mathf.Min(remaining, Mathf.Max(turnSpeedDegrees * Time.fixedDeltaTime, lookAheadDegrees));
            int steps = Mathf.Max(1, Mathf.CeilToInt(span));
            for (int i = 1; i <= steps; i++)
            {
                if (PoseBlocked(position, Mathf.MoveTowardsAngle(yaw, targetYaw, span * i / steps), out string obstacle)) return obstacle;
            }
            return null;
        }

        private Vector3 AheadPose(Vector3 target)
        {
            Vector3 remaining = target - position;
            float distance = remaining.magnitude;
            return distance < 1e-4f ? position : position + remaining / distance * Mathf.Min(lookAheadDistance, distance);
        }

        // 장애물 정지 후 사라짐을 확인하고 resumeDelaySeconds 동안 계속 비어 있으면 재개.
        // canGiveUp이면 replanAfterBlockedSeconds 넘게 막힐 때 이동을 중단한다(다시 탐색). 아니면 dockingBlockTimeoutSeconds 넘게 막힐 때 고장 정지
        private IEnumerator WaitForClear(Func<string> probe, string obstacle, bool canGiveUp)
        {
            ForkliftState previous = State;
            ObstacleStopCount++;
            LastObstacleName = obstacle;
            SetState(ForkliftState.StoppedByObstacle);
            Debug.Log($"[AutonomousForkliftController {lane}] 장애물 정지 ({obstacle})", this);
            float clear = 0f, blockedTime = 0f;
            while (clear < resumeDelaySeconds)
            {
                yield return waitStep;
                string now = probe();
                if (now == null)
                {
                    clear += Time.fixedDeltaTime;
                    continue;
                }
                clear = 0f;
                blockedTime += Time.fixedDeltaTime;
                LastObstacleName = now;
                if (canGiveUp && blockedTime >= replanAfterBlockedSeconds)
                {
                    SetState(previous);
                    moveGaveUp = true;
                    yield break;
                }
                if (!canGiveUp && blockedTime >= dockingBlockTimeoutSeconds)
                {
                    Fault($"진입·후퇴 경로가 {dockingBlockTimeoutSeconds:F0}초 넘게 막힘 ({now})");
                    yield break;
                }
            }
            Debug.Log($"[AutonomousForkliftController {lane}] 장애물 해소 {resumeDelaySeconds:F0}초 후 재개", this);
            SetState(previous);
        }

        // 이 자세에서 차체·화물 점유 상자에 장애물이 있는지
        private bool PoseBlocked(Vector3 at, float headingDegrees, out string obstacle)
        {
            obstacle = null;
            Quaternion rotation = Quaternion.Euler(0f, headingDegrees, 0f);
            if (BoxBlocked(chassisBox, at, rotation, false, out obstacle)) return true;
            Vector3 lift = Vector3.up * ForkLift;
            foreach (LocalBox fork in forkBoxes)
            {
                LocalBox raised = fork;
                raised.Centre += lift;
                if (BoxBlocked(raised, at, rotation, true, out obstacle)) return true;
            }
            return carried != null && BoxBlocked(loadBox, at, rotation, false, out obstacle);
        }

        private bool BoxBlocked(LocalBox box, Vector3 at, Quaternion rotation, bool forkPart, out string obstacle)
        {
            obstacle = null;
            int n = OverlapAll(at + rotation * box.Centre, box.Half, rotation);
            for (int i = 0; i < n; i++)
            {
                if (!IsObstacle(hits[i], forkPart)) continue;
                obstacle = hits[i].name;
                return true;
            }
            return false;
        }

        // 장애물 판정: 자기 차체·운반물·트리거·바닥면을 뺀 모든 충돌체 (박스·설비·건물·차량·적재된 파레트·다른 AMR).
        // forkPart(포크 상자 검사)일 때만 포크 날이 정상으로 드나드는 파레트·박스(forkPassRoots)를 뺀다
        private bool IsObstacle(Collider other, bool forkPart)
        {
            if (other == null || other.isTrigger) return false;
            Transform t = other.transform;
            if (t.IsChildOf(vehicle) || (carried != null && t.IsChildOf(carried))) return false;
            if (forkPart)
            {
                foreach (Transform root in forkPassRoots)
                {
                    if (root != null && t.IsChildOf(root)) return false;
                }
            }
            return other.bounds.max.y > position.y + floorTolerance;
        }

        // 지금 자세의 포크 상자가 예외 파레트·박스와 겹치는지 (인출 예외 종료 판정)
        private bool ForkOverlapsPassRoots()
        {
            Quaternion rotation = Quaternion.Euler(0f, yaw, 0f);
            Vector3 lift = Vector3.up * ForkLift;
            foreach (LocalBox fork in forkBoxes)
            {
                int n = OverlapAll(position + rotation * (fork.Centre + lift), fork.Half, rotation);
                for (int i = 0; i < n; i++)
                {
                    if (hits[i].isTrigger) continue;
                    foreach (Transform root in forkPassRoots)
                    {
                        if (root != null && hits[i].transform.IsChildOf(root)) return true;
                    }
                }
            }
            return false;
        }

        // Start 이후 포크가 올라간 높이 (m)
        private float ForkLift => Vector3.Dot(forkLink.position - vehicle.position, Vector3.up) - forkHeightAtStart;

        // 경로 구간을 그 방향으로 지나갈 때 차체·화물 휩쓸기 검사 (경로 탐색용)
        private bool SegmentClear(Vector3 a, Vector3 b)
        {
            Vector3 d = b - a;
            d.y = 0f;
            float length = d.magnitude;
            if (length < 1e-4f) return true;
            float heading = Yaw(d);
            int steps = Mathf.Max(1, Mathf.CeilToInt(length / (pathCellSize * 0.5f)));
            for (int i = 1; i <= steps; i++)
            {
                if (PoseBlocked(Vector3.Lerp(a, b, (float)i / steps), heading, out _)) return false;
            }
            return true;
        }

        // 제자리 회전 공간 검사 (5도 간격)
        private bool RotationClear(Vector3 at, float from, float to, out string obstacle)
        {
            obstacle = null;
            float delta = Mathf.DeltaAngle(from, to);
            int steps = Mathf.CeilToInt(Mathf.Abs(delta) / 5f);
            for (int i = 1; i <= steps; i++)
            {
                if (PoseBlocked(at, from + delta * i / steps, out obstacle)) return false;
            }
            return true;
        }

        // 경로 탐색 격자를 넓힐 차체·화물 반폭 (진행 방향 옆쪽)
        private float FootprintHalfWidth
        {
            get
            {
                float half = Mathf.Abs(chassisBox.Centre.x) + chassisBox.Half.x;
                foreach (LocalBox fork in forkBoxes) half = Mathf.Max(half, Mathf.Abs(fork.Centre.x) + fork.Half.x);
                if (carried != null) half = Mathf.Max(half, Mathf.Abs(loadBox.Centre.x) + loadBox.Half.x);
                return half;
            }
        }

        private float FootprintTop
        {
            get
            {
                float top = chassisBox.Centre.y + chassisBox.Half.y;
                foreach (LocalBox fork in forkBoxes) top = Mathf.Max(top, fork.Centre.y + fork.Half.y + 0.2f);
                if (carried != null) top = Mathf.Max(top, loadBox.Centre.y + loadBox.Half.y);
                return top + 0.05f;
            }
        }

        // 제자리 회전 때 차체·포크·화물이 휩쓰는 반경 (모델 루트 기준 수평, m)
        private float RotationRadius
        {
            get
            {
                float radius = Reach(chassisBox);
                foreach (LocalBox fork in forkBoxes) radius = Mathf.Max(radius, Reach(fork));
                if (carried != null) radius = Mathf.Max(radius, Reach(loadBox));
                return radius;
            }
        }

        private static float Reach(LocalBox box)
        {
            float x = Mathf.Abs(box.Centre.x) + box.Half.x, z = Mathf.Abs(box.Centre.z) + box.Half.z;
            return Mathf.Sqrt(x * x + z * z);
        }

        private static float PalletHalfDiagonal(Transform pallet)
        {
            Collider[] parts = pallet.GetComponentsInChildren<Collider>();
            if (parts.Length == 0) return 0.31f;
            Bounds b = parts[0].bounds;
            foreach (Collider part in parts) b.Encapsulate(part.bounds);
            return new Vector2(b.extents.x, b.extents.z).magnitude;
        }

        // 점유 상자 측정: 포크 링크 아래 충돌체를 뺀 차체 상자, 포크 범위를 fork_mockup 형상 비율로 나눈 등판·날 2개
        // (fork_mockup: 길이 1.003, 폭 ±0.2285, 높이 0.835, 등판 두께 0.04, 날 폭 0.0395(0.189~0.2285), 날 두께 0.04)
        private void MeasureFootprint()
        {
            var chassisParts = new List<Collider>();
            var forkParts = new List<Collider>();
            foreach (Collider part in vehicle.GetComponentsInChildren<Collider>())
            {
                if (part.transform.IsChildOf(forkLink)) forkParts.Add(part); else chassisParts.Add(part);
            }
            chassisBox = MeasureLocalBox(chassisParts.ToArray(), true);
            LocalBox fork = MeasureLocalBox(forkParts.ToArray(), false);
            Vector3 min = fork.Centre - fork.Half, max = fork.Centre + fork.Half;
            float depth = max.z - min.z, width = max.x - min.x, heightAll = max.y - min.y;
            float back = depth * (0.04f / 1.003f), tine = width * (0.0395f / 0.457f), tineHeight = heightAll * (0.04f / 0.835f);
            forkBoxes[0] = FromMinMax(min, new Vector3(max.x, max.y, min.z + back));
            forkBoxes[1] = FromMinMax(min, new Vector3(min.x + tine, min.y + tineHeight, max.z));
            forkBoxes[2] = FromMinMax(new Vector3(max.x - tine, min.y, min.z), new Vector3(max.x, min.y + tineHeight, max.z));
            forkHeightAtStart = Vector3.Dot(forkLink.position - vehicle.position, Vector3.up);
        }

        private static LocalBox FromMinMax(Vector3 min, Vector3 max) => new LocalBox { Centre = (min + max) * 0.5f, Half = Vector3.Max((max - min) * 0.5f, Vector3.one * 0.002f) };

        // 충돌체들의 월드 범위를 모델 루트 기준(축척 없는) 상자로 잰다. clipFloor면 바닥면 높이(floorTolerance) 아래는 뺀다
        private LocalBox MeasureLocalBox(Collider[] parts, bool clipFloor)
        {
            Quaternion inverse = Quaternion.Inverse(vehicle.rotation);
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (Collider part in parts)
            {
                if (part.isTrigger) continue;
                // Articulation 이동 직후 Collider.bounds는 이전 물리 자세일 수 있다.
                // 로컬 형상과 현재 Transform으로 계산해 포크 높이 기준이 어긋나지 않게 한다.
                Bounds b;
                bool localBounds = true;
                if (part is MeshCollider mesh && mesh.sharedMesh != null) b = mesh.sharedMesh.bounds;
                else if (part is BoxCollider box) b = new Bounds(box.center, box.size);
                else { b = part.bounds; localBounds = false; }
                for (int i = 0; i < 8; i++)
                {
                    Vector3 corner = new Vector3((i & 1) == 0 ? b.min.x : b.max.x, (i & 2) == 0 ? b.min.y : b.max.y, (i & 4) == 0 ? b.min.z : b.max.z);
                    if (localBounds) corner = part.transform.TransformPoint(corner);
                    Vector3 local = inverse * (corner - vehicle.position);
                    min = Vector3.Min(min, local);
                    max = Vector3.Max(max, local);
                }
            }
            if (min.x > max.x)
            {
                return new LocalBox { Centre = new Vector3(0f, 0.2f, 0f), Half = new Vector3(0.15f, 0.15f, 0.3f) };
            }
            if (clipFloor) min.y = Mathf.Max(min.y, floorTolerance + 0.005f);
            max.y = Mathf.Max(max.y, min.y + 0.01f);
            return new LocalBox { Centre = (min + max) * 0.5f, Half = (max - min) * 0.5f };
        }

        // 겹침 조회. 버퍼가 차면 늘려서 다시 조회한다
        private int OverlapAll(Vector3 centre, Vector3 half, Quaternion rotation)
        {
            int n;
            while ((n = Physics.OverlapBoxNonAlloc(centre, half, hits, rotation, ~0, QueryTriggerInteraction.Ignore)) == hits.Length)
            {
                hits = new Collider[hits.Length * 2];
            }
            return n;
        }

        // 경로 위 자세를 모델 루트와 관절 루트에 같이 준다
        private void Place()
        {
            Quaternion rotation = Quaternion.Euler(0f, yaw, 0f);
            vehicle.SetPositionAndRotation(position, rotation);
            rootBody.TeleportRoot(vehicle.TransformPoint(rootLocalPosition), rotation * rootLocalRotation);
            SyncCarriedPose();
        }

        // 키네마틱 Rigidbody의 물리 자세가 부모 Transform 이동을 덮어쓰며
        // 포크 승강 오프셋을 누적하지 않도록 인양 때 저장한 상대 자세로 동기화한다.
        // 적재 슬롯으로 보정하는 동작이 아니라 실제 포크에 고정된 운반물의 추종이다.
        private void SyncCarriedPose()
        {
            if (carried == null || forkLink == null) return;
            foreach (CarriedPose pose in carriedPoses)
            {
                if (pose.Body == null) continue;
                pose.Body.position = forkLink.TransformPoint(pose.Position);
                pose.Body.rotation = forkLink.rotation * pose.Rotation;
            }
        }

        private void LateUpdate() => SyncCarriedPose();

        private IEnumerator MoveFork(float target)
        {
            ArticulationDrive drive = forkBody.xDrive;
            if (Mathf.Approximately(drive.target, target))
            {
                yield break;
            }
            drive.target = target;
            forkBody.xDrive = drive;
            for (float elapsed = 0f; elapsed < forkMoveSeconds; elapsed += Time.fixedDeltaTime)
            {
                SyncCarriedPose();
                yield return waitStep;
            }
            SyncCarriedPose();
        }

        private void SetState(ForkliftState state)
        {
            if (State == state)
            {
                return;
            }
            State = state;
            StateChanged?.Invoke(state);
        }

        // 초기화 버튼: 운반 중지, 들고 있던 파레트 제거, 대기 위치로 즉시 배치, State = Idle
        public void ResetToInitialState()
        {
            StopAllCoroutines();
            routine = null;
            if (carried != null)
            {
                // 같은 프레임의 빈 파레트 자리 확인에 걸리지 않도록 끄고 지운다
                carried.gameObject.SetActive(false);
                Destroy(carried.gameObject);
                carried = null;
            }
            carriedBoxCount = 0;
            carriedPoses.Clear();
            ClearForkPass();
            lastPlaced = null;
            removalPending = false;
            targetPallet = null;
            targetDock = null;
            FaultReason = null;
            LoadRetryCount = 0;
            DeliveredCount = 0;
            ObstacleStopCount = 0;
            ReplanCount = 0;
            LastObstacleName = null;
            LastPlanResult = null;
            if (initialized)
            {
                ArticulationDrive drive = forkBody.xDrive;
                drive.target = initialForkTarget;
                forkBody.xDrive = drive;
                position = startPosition;
                yaw = startRotation.eulerAngles.y;
                Place();
            }
            SetState(ForkliftState.Idle);
        }

        private static Vector3 Flat(Vector3 v)
        {
            v.y = 0f;
            return v.sqrMagnitude > 1e-6f ? v.normalized : Vector3.forward;
        }

        private static float Yaw(Vector3 direction) => Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;
        #endregion
    }
}
