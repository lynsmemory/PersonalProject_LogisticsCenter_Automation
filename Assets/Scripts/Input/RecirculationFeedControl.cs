using System.Collections.Generic;
using RoboLacyTech.Conveyors;
using UnityEngine;

namespace Logistics
{
    // 합류 직전 메인 구간(Main_East_02)에서 순환 박스를 감지하면, 그 박스가 합류 구간을 다 지나갈 때까지만 신규 투입선과 스폰을 보류한다.
    // 보류 시간 = (박스 뒷면에서 합류 구간 끝까지 남은 거리 + 여유) ÷ 실제 메인 벨트 속도. 속도가 바뀌면 보류 시간도 그에 비례해 바뀐다.
    // 감지 구역에 박스가 계속 들어오면 보류를 연장한다. 메인은 계속 운전하며(FR-05) 정체 레인 라벨 제외(FR-19)는 BoxSpawner가 따로 한다.
    // 감지 구역(detectionZone)의 로컬 +Z가 메인 진행 방향이다. 합류 구간 위의 박스는 메인이 구동하므로 투입선 정지와 상관없이 계속 지나간다.
    public class RecirculationFeedControl : MonoBehaviour
    {
        #region 1. 연결과 설정
        [SerializeField] private LogisticsSimulationController simulation;
        [SerializeField] private MainConveyorController conveyor;
        [SerializeField] private BoxSpawner spawner;

        [Tooltip("합류 직전 메인 구간의 감지 구역 (로컬 +Z = 메인 진행 방향)")]
        [SerializeField] private BoxCollider detectionZone;

        [Tooltip("감지 구역 아래 메인 구동 컨베이어. 실제 벨트 속도를 읽어 보류 시간을 계산한다")]
        [SerializeField] private LinearConveyor approachConveyor;

        [Tooltip("감지 구역 중심에서 합류 구간 끝까지 진행 방향 거리 (m)")]
        [SerializeField, Min(0f)] private float mergeExitDistance = 2.675f;

        [Tooltip("박스 뒷면이 합류 구간 끝을 지난 뒤 더 기다리는 거리 (m)")]
        [SerializeField, Min(0f)] private float clearanceMargin = 0.2f;

        [SerializeField] private Transform inputRoot;
        #endregion

        #region 2. 상태
        private readonly Collider[] hits = new Collider[64];
        private readonly HashSet<Rigidbody> detected = new HashSet<Rigidbody>();
        private float holdUntil;   // 이 시각(Time.time)까지 투입 보류
        private readonly Dictionary<Rigidbody, float> resting = new Dictionary<Rigidbody, float>();
        private Collider[] inputSurfaces;
        private LinearConveyor[] inputDrives;
        // 지금 감지 구역 안의 순환 박스 수
        public int PendingCount => detected.Count;
        public bool IsHolding { get; private set; }
        #endregion

        #region 3. 운전
        private void Start()
        {
            if (inputRoot == null) return;
            inputSurfaces = inputRoot.GetComponentsInChildren<Collider>();
            inputDrives = inputRoot.GetComponentsInChildren<LinearConveyor>();
        }

        private void OnEnable()
        {
            if (simulation != null) simulation.StateChanged += OnStateChanged;
        }

        private void OnDisable()
        {
            if (simulation != null) simulation.StateChanged -= OnStateChanged;
            detected.Clear();
            holdUntil = 0f;
            Apply(false);
            RestoreRestingBoxes();
        }

        private void OnStateChanged(SimulationRunState state)
        {
            if (state != SimulationRunState.Stopped) return;
            detected.Clear();
            holdUntil = 0f;
            Apply(false);
        }

        private void FixedUpdate()
        {
            if (simulation == null || conveyor == null || spawner == null || detectionZone == null) return;
            if (simulation.State == SimulationRunState.Stopped)
            {
                detected.Clear();
                holdUntil = 0f;
                Apply(false);
                return;
            }
            if (simulation.State != SimulationRunState.Running) return;

            //1. 감지 구역 안의 순환 박스마다 합류 구간 끝까지 남은 시간을 구해 가장 늦은 시각까지 보류
            Transform zone = detectionZone.transform;
            Vector3 flow = zone.forward;
            float exit = Vector3.Dot(zone.TransformPoint(detectionZone.center), flow) + mergeExitDistance + clearanceMargin;
            float speed = approachConveyor != null ? Mathf.Abs(approachConveyor.CurrentSpeed) : 0f;
            detected.Clear();
            int n = Physics.OverlapBoxNonAlloc(zone.TransformPoint(detectionZone.center), Vector3.Scale(detectionZone.size, zone.lossyScale) * 0.5f,
                hits, zone.rotation, ~0, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < n; i++)
            {
                Rigidbody body = hits[i].attachedRigidbody;
                var box = body != null ? body.GetComponent<LogisticsBox>() : null;
                if (box == null || box.IsHeld || body.isKinematic || !detected.Add(body)) continue;
                Bounds b = hits[i].bounds;
                float rear = Vector3.Dot(b.center, flow) - (Mathf.Abs(b.extents.x * flow.x) + Mathf.Abs(b.extents.y * flow.y) + Mathf.Abs(b.extents.z * flow.z));
                float remaining = Mathf.Max(0f, exit - rear);
                // 벨트가 거의 멈춰 있으면 시간 계산 대신 다음 물리 단계까지 계속 보류한다
                float wait = speed > 0.01f ? remaining / speed : Time.fixedDeltaTime;
                holdUntil = Mathf.Max(holdUntil, Time.time + wait);
            }
            // 조회가 포화되면 다음 단계까지 보류를 유지한다
            if (n == hits.Length) holdUntil = Mathf.Max(holdUntil, Time.time + Time.fixedDeltaTime);

            //2. 보류 시각 전이면 투입선·스폰 정지, 지나면 재개
            Apply(Time.time < holdUntil);
            if (IsHolding) AllowStationaryInputBoxesToRest();
        }

        private void AllowStationaryInputBoxesToRest()
        {
            if (inputSurfaces == null || inputDrives == null) return;
            foreach (var drive in inputDrives)
                if (drive.enabled && Mathf.Abs(drive.CurrentSpeed) > 0.001f) return;
            // 공정 박스의 sleepThreshold=0은 장시간 정지된 경사 벨트에서 미세 드리프트를 만든다.
            // 모터 감속이 끝나고 투입면 위에서 거의 정지한 박스만 기본 수면을 허용한다.
            // 위치·속도·충돌은 강제로 고정하지 않으며 재개 시 원래 설정으로 복구한다.
            foreach (var box in spawner.SpawnedBoxes)
            {
                if (box == null || box.IsHeld || box.Body.isKinematic || resting.ContainsKey(box.Body) || box.Body.linearVelocity.sqrMagnitude > 0.0025f) continue;
                var shape = box.GetComponentInChildren<BoxCollider>();
                if (shape == null) continue;
                Vector3 bottom = shape.bounds.center - Vector3.up * shape.bounds.extents.y;
                foreach (var surface in inputSurfaces)
                {
                    if (!surface.enabled || surface.isTrigger) continue;
                    if ((surface.ClosestPoint(bottom) - bottom).sqrMagnitude > 0.0016f) continue;
                    resting.Add(box.Body, box.Body.sleepThreshold);
                    box.Body.sleepThreshold = Physics.sleepThreshold;
                    break;
                }
            }
        }

        private void RestoreRestingBoxes()
        {
            foreach (var pair in resting)
                if (pair.Key != null) { pair.Key.sleepThreshold = pair.Value; pair.Key.WakeUp(); }
            resting.Clear();
        }

        private void Apply(bool hold)
        {
            if (IsHolding == hold) return;
            IsHolding = hold;
            if (!hold) RestoreRestingBoxes();
            if (spawner != null) spawner.SetFeedBlocked(hold);
            if (conveyor != null) conveyor.SetInputBlocked(hold);
            Debug.Log($"[RecirculationFeedControl] 투입 {(hold ? $"보류 (합류 직전 순환 박스 {detected.Count}개, {holdUntil - Time.time:F1}초)" : "재개")}", this);
        }
        #endregion
    }
}
