using System;
using System.Collections;
using UnityEngine;

namespace Logistics
{
    // 레인 하나와 전담 설비를 묶고, 중간 스토퍼 방출 → 픽업 정렬 → (로봇 적재) 순서를 진행한다. 레인당 1개. (FR-13, FR-19, FR-20, FR-22)
    // 레인 설비(집계·스토퍼·픽업)와 FR5 적재(PalletizingSequence)·파레트(PalletLoadState), 전담 AMR(AutonomousForkliftController)·도크(DockVehicleManager)를 연결한다.
    // 6×L 완료 신호는 전담 AMR에 운반 요청으로 보내고, AMR이 바쁘면 받을 수 있을 때까지 다시 요청한다. AMR이 완성 파레트를 반출하면 빈 파레트를 재생성하고 적재를 다시 시작한다.
    // palletizing·pallet가 연결되어 있으면 정렬된 박스를 로봇이 실제로 적재하고, 놓인 위치 확인까지 성공했을 때만 처리 수를 늘린다.
    // 로봇이 연결되지 않은 레인만 시험용 simulatedPickSeconds(> 0)로 정렬된 박스를 그 시간 뒤 공정에서 뺀다(로봇 대체).
    // 로봇 적재는 레인 루프와 병행해 돌고(한 번에 하나), 로봇이 박스를 들어 올려 픽업 구역을 벗어나면(PalletizingSequence.State가 Picking이 아님)
    // 적재·복귀를 기다리지 않고 다음 박스를 방출한다. 진행 중인 박스까지 세어 파레트 자리가 남을 때만 방출한다.
    public class LaneProcessController : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("레인 번호")]
        [SerializeField] private DestinationLane lane = DestinationLane.Lane1;

        [Tooltip("레인 대기 수")]
        [SerializeField] private LaneOccupancyMonitor occupancyMonitor;

        [Tooltip("중간 스토퍼")]
        [SerializeField] private MiddleStopper middleStopper;

        [Tooltip("픽업 구역")]
        [SerializeField] private PickupZoneAligner pickupZone;

        [Tooltip("이 레인 FR5 적재 순서")]
        [SerializeField] private PalletizingSequence palletizing;

        [Tooltip("이 레인 파레트")]
        [SerializeField] private PalletLoadState pallet;

        [Tooltip("전담 AMR (레인 번호와 같은 번호)")]
        [SerializeField] private AutonomousForkliftController forklift;

        [Tooltip("전담 도크 (레인 번호와 같은 번호)")]
        [SerializeField] private DockVehicleManager dock;

        [Tooltip("박스 정리 (로봇 대체 시험에서 집어 간 박스 제거)")]
        [SerializeField] private BoxSpawner boxSpawner;

        [Tooltip("로봇 대체 시험용: palletizing·pallet가 연결되지 않았을 때만 쓴다. 정렬 후 이 시간(초)이 지나면 박스를 집어 간 것으로 처리. 0이면 사용 안 함")]
        [SerializeField, Min(0f)] private float simulatedPickSeconds = 0f;

        [Tooltip("방출 조건 확인 주기 (초)")]
        [SerializeField, Min(0.05f)] private float pollInterval = 0.1f;
        #endregion

        #region 2. 상태
        private Coroutine loop;
        private Rigidbody inTransit;     // 중간 스토퍼를 지나 픽업으로 내려가는 박스
        private float releasedAt;
        private bool transitWarned;

        [Tooltip("방출한 박스가 이 시간(초) 안에 픽업에 도착하지 않으면 경고 (다음 방출은 계속 막음)")]
        [SerializeField, Min(1f)] private float transitTimeout = 30f;

        // 방출 후 픽업 도착 대기 중
        public bool HasBoxInTransit => inTransit != null;

        public DestinationLane Lane => lane;
        public LaneOccupancyMonitor OccupancyMonitor => occupancyMonitor;
        public MiddleStopper MiddleStopper => middleStopper;
        public PickupZoneAligner PickupZone => pickupZone;
        public bool IsRunning => loop != null;
        public PalletizingSequence Palletizing => palletizing;
        public PalletLoadState Pallet => pallet;
        public AutonomousForkliftController Forklift => forklift;
        public DockVehicleManager Dock => dock;
        // 로봇 적재가 연결되어 있음 (대체 타이머 대신 실제 적재)
        public bool UsesRobot => palletizing != null && pallet != null;
        // 적재 실패로 멈춤 (실패 후 처리 규칙은 SRS상 시험 후 결정)
        public bool IsHaltedByFailure { get; private set; }
        public bool PalletCompleteSignaled { get; private set; }   // 6×L 완료 신호를 보냈음 (AMR 반출 전까지 적재 중지)
        public bool PickupRequested { get; private set; }          // 전담 AMR이 운반 요청을 받음
        public string FaultReason { get; private set; }            // 레인 대기 이유 (빈 파레트 재생성 실패 등). 없으면 null
        // 이 레인 파레트가 적재 준비됨 (로봇 미연결 레인은 항상 true). 아니면 방출·적재·라벨 투입을 막는다
        public bool IsPalletReady => !UsesRobot || pallet.IsReady;
        private float nextRespawnRetry;

        [Tooltip("빈 파레트 재생성 실패 후 다시 시도하는 간격 (초)")]
        [SerializeField, Min(0.5f)] private float respawnRetryInterval = 2f;
        // 이 레인에서 처리(픽업 완료)한 박스 수
        public int ProcessedCount { get; private set; }

        // 박스 1개 처리 완료 (기록용)
        public event Action<DestinationLane> BoxProcessed;
        #endregion

        #region 3. 운전
        private void OnEnable()
        {
            if (middleStopper != null) middleStopper.Released += OnStopperReleased;
            if (pickupZone != null) pickupZone.BoxAligned += OnBoxAligned;
            if (forklift != null) forklift.PalletRemoved += OnPalletRemoved;
        }

        private void OnDisable()
        {
            if (middleStopper != null) middleStopper.Released -= OnStopperReleased;
            if (pickupZone != null) pickupZone.BoxAligned -= OnBoxAligned;
            if (forklift != null) forklift.PalletRemoved -= OnPalletRemoved;
        }

        private void OnStopperReleased(Rigidbody body)
        {
            inTransit = body;
            releasedAt = Time.time;
            transitWarned = false;
        }

        private void OnBoxAligned(LogisticsBox box)
        {
            inTransit = null;
        }

        // 설정 적용 후 레인 운전 시작 (적재 층 수 L을 파레트에 전달)
        public void Begin(SimulationSettings settings)
        {
            if (pallet != null && settings != null)
            {
                pallet.Configure(settings.PalletLayerCount);
            }
            if (dock != null && settings != null)
            {
                dock.Configure(settings.TruckPalletCapacity);
            }
            if (!IsPalletReady)
            {
                Debug.LogWarning($"[LaneProcessController {lane}] 파레트 준비 안 됨 ({FaultReason ?? "파레트 없음"}). 복구될 때까지 이 레인 방출·적재 대기", this);
            }
            if (loop == null)
            {
                loop = StartCoroutine(RunLaneLoop());
            }
        }

        // 일시정지·재개는 전체 시간 정지로 처리하므로 별도 동작 없음 (LogisticsSimulationController 참조)
        public void Pause() { }
        public void Resume() { }

        // 방출 판단 반복 코루틴. 다음 박스는 "앞서 방출한 박스 도착 + 픽업 구역 빔 + 로봇이 픽업 구역을 벗어남 + 방출 금지 상태 아님"일 때 1개 방출
        private IEnumerator RunLaneLoop()
        {
            var wait = new WaitForSeconds(pollInterval);
            bool placing = false;   // 이 루프가 시작한 로봇 적재가 진행 중
            while (true)
            {
                // 빈 파레트 재생성 실패 상태면 일정 간격으로 다시 만들어 본다. 성공해야만 다음 적재 주기가 열린다
                if (UsesRobot && pallet.NeedsRespawn && Time.time >= nextRespawnRetry)
                {
                    nextRespawnRetry = Time.time + respawnRetryInterval;
                    TryRecoverPallet();
                }
                // 완료 신호를 보냈는데 AMR이 아직 요청을 받지 못했으면(운반 중·연결 지연) 다시 요청한다
                if (PalletCompleteSignaled && !PickupRequested) RequestForkliftPickup();
                // 실패/완료 결과를 다음 방출보다 먼저 반영한다. 실패 직후 빈 픽업으로 추가 방출하지 않는다.
                if (placing && !palletizing.IsBusy)
                {
                    placing = false;
                    if (palletizing.LastResult == PalletizingResult.Success)
                    {
                        OnBoxProcessed();
                        if (pallet.IsComplete) OnPalletCompleted();
                    }
                    else
                    {
                        IsHaltedByFailure = true;
                        Debug.LogWarning($"[LaneProcessController {lane}] 적재 실패 {palletizing.LastResult} (놓인 위치 오차 {palletizing.LastPlaceError * 1000f:F0}mm). 이 레인 적재를 멈춤", this);
                    }
                }
                //1. 픽업 구역이 비고 앞서 방출한 박스가 픽업에 도착했으면 중간 스토퍼에서 1개 방출
                //   (방출한 박스가 경사 구간을 내려가는 동안 다음 박스를 내보내지 않는다: 중간 1·픽업 1 제한)
                if (inTransit != null && Time.time - releasedAt > transitTimeout && !transitWarned)
                {
                    transitWarned = true;
                    Debug.LogWarning($"[LaneProcessController {lane}] 방출한 {inTransit.name}가 {transitTimeout:F0}초 안에 픽업에 도착하지 않음", this);
                }
                //   로봇 연결 레인: 실패로 멈추지 않았고, 로봇이 픽업 구역을 벗어났고(Picking 아님), 진행 중 박스까지 세어 파레트 자리가 남을 때만
                bool robotAllows = !UsesRobot ||
                    (!IsHaltedByFailure && IsPalletReady && palletizing.State != PalletizingState.Picking && pallet.PlacedCount + (palletizing.State == PalletizingState.Placing ? 1 : 0) < pallet.Capacity);
                if (inTransit == null && robotAllows && pickupZone != null && pickupZone.IsEmpty && middleStopper != null && !middleStopper.IsBusy && middleStopper.HasWaitingBox)
                {
                    middleStopper.ReleaseOne();
                }

                //3. 정렬된 박스: 로봇이 연결되어 있고 적재 중이 아니면 적재 시작 (파레트 완료·실패로 멈춘 동안은 대기)
                if (UsesRobot && !placing && pickupZone != null && pickupZone.CurrentBox != null && !IsHaltedByFailure && IsPalletReady && !pallet.IsComplete)
                {
                    StartCoroutine(palletizing.PickAndPlace(pickupZone, pallet));
                    placing = true;
                }
                //4. 로봇이 연결되지 않은 레인: 시험용 대체 타이머로 일정 시간 뒤 집어 간 것으로 처리
                else if (!UsesRobot && pickupZone != null && pickupZone.CurrentBox != null && simulatedPickSeconds > 0f)
                {
                    LogisticsBox box = pickupZone.CurrentBox;
                    // 대기 중에 처리 시간이 바뀌어도 반영되도록 매 프레임 확인한다
                    float waited = 0f;
                    while (waited < simulatedPickSeconds && pickupZone.CurrentBox == box)
                    {
                        waited += Time.deltaTime;
                        yield return null;
                    }
                    if (pickupZone.CurrentBox == box && box != null)
                    {
                        pickupZone.ClearZone();
                        if (boxSpawner != null) boxSpawner.Despawn(box);
                        else Destroy(box.gameObject);
                        OnBoxProcessed();
                    }
                }
                yield return wait;
            }
        }

        private void OnBoxProcessed()
        {
            ProcessedCount++;
            BoxProcessed?.Invoke(lane);
        }

        // 적재 완료(6×L) 신호를 전담 AMR에 1회 전달한다. AMR이 연결되지 않은 레인은 신호만 기록하고 이 레인 적재를 멈춘다
        private void OnPalletCompleted()
        {
            if (PalletCompleteSignaled)
            {
                return;
            }
            PalletCompleteSignaled = true;
            Debug.Log($"[LaneProcessController {lane}] 파레트 적재 완료 {pallet.PlacedCount}/{pallet.Capacity}. {(forklift != null ? "전담 AMR에 운반 요청" : "AMR 미연결, 반출 대기")}", this);
            RequestForkliftPickup();
        }

        private void RequestForkliftPickup()
        {
            if (forklift == null || dock == null || PickupRequested)
            {
                return;
            }
            if (forklift.RequestPickup(pallet, dock))
            {
                PickupRequested = true;
            }
        }

        // AMR이 완성 파레트를 적재 위치에서 가져감: 빈 파레트가 실제로 만들어졌을 때만 다음 적재 주기를 연다 (FR-20).
        // 실패하면 완료 상태(적재 수 6×L)를 유지해 방출·적재가 멈춘 채 대기하고 이유를 표시한다 (초기화로 해제)
        private void OnPalletRemoved()
        {
            string reason = "파레트 상태 연결 없음";
            if (pallet == null || !pallet.RespawnEmptyPallet(out reason))
            {
                FaultReason = $"빈 파레트 재생성 실패: {reason}";
                Debug.LogWarning($"[LaneProcessController {lane}] {FaultReason}. 레인 적재 대기", this);
                return;
            }
            FaultReason = null;
            PalletCompleteSignaled = false;
            PickupRequested = false;
            Debug.Log($"[LaneProcessController {lane}] 빈 파레트 재생성. 적재 재개", this);
        }

        // 재생성 실패 복구: 실제로 만들어졌을 때만 대기 이유와 완료·요청 플래그를 해제한다
        private void TryRecoverPallet()
        {
            if (!pallet.RespawnEmptyPallet(out string reason))
            {
                FaultReason = $"빈 파레트 재생성 실패: {reason}";
                return;
            }
            FaultReason = null;
            PalletCompleteSignaled = false;
            PickupRequested = false;
            Debug.Log($"[LaneProcessController {lane}] 빈 파레트 복구. 적재 재개", this);
        }

        // 초기화 버튼: 레인 순서 진행 중지, 레인 설비 초기화 호출
        public void ResetToInitialState()
        {
            // 레인 루프와 진행 중인 로봇 적재 코루틴을 함께 멈춘다
            StopAllCoroutines();
            loop = null;
            if (occupancyMonitor != null) occupancyMonitor.ResetToInitialState();
            if (middleStopper != null) middleStopper.ResetToInitialState();
            if (pickupZone != null) pickupZone.ResetToInitialState();
            if (palletizing != null) palletizing.ResetToInitialState();
            // AMR이 들고 있던 파레트·차량 적재물을 먼저 정리한 뒤 이 자리 파레트를 되돌리거나 다시 만든다
            if (forklift != null) forklift.ResetToInitialState();
            if (dock != null) dock.ResetToInitialState();
            if (pallet != null) pallet.ResetToInitialState();
            inTransit = null;
            ProcessedCount = 0;
            IsHaltedByFailure = false;
            PalletCompleteSignaled = false;
            PickupRequested = false;
            // 초기화 중 재생성이 실패하면 준비 안 됨(IsPalletReady=false)으로 남아 방출·적재가 막히고, 루프가 복구를 계속 시도한다
            FaultReason = pallet != null && pallet.LastRespawnFailure != null ? $"빈 파레트 재생성 실패: {pallet.LastRespawnFailure}" : null;
            nextRespawnRetry = 0f;
        }
        #endregion
    }
}
