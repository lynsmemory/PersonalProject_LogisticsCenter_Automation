using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Logistics
{
    // 투입점에 정한 간격으로 박스를 생성한다. 정체된 레인 바코드의 박스는 투입하지 않는다. (FR-03, FR-04, FR-19)
    // 설정된 라벨 후보 중 정체되지 않은 레인을 같은 확률로 무작위 선택하고, 모두 정체면 기다린다.
    // 투입점 주변에 앞 박스가 남아 있으면 생성을 미룬다 (겹쳐 생성 방지).
    public class BoxSpawner : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("박스 타입별 Prefab (BoxType 순서)")]
        [SerializeField] private GameObject[] boxPrefabs = new GameObject[1];

        [Tooltip("투입점 (EntryLine/UpperEntryStart/BoxSpawnPoint)")]
        [SerializeField] private Transform spawnPoint;

        [Tooltip("생성할 박스 타입")]
        [SerializeField] private BoxType spawnBoxType = BoxType.Standard;

        [Tooltip("무작위로 선택할 바코드 후보 (중복 레인은 한 번만 반영)")]
        [SerializeField] private DestinationLane[] labelSequence = { DestinationLane.Lane1, DestinationLane.Lane2, DestinationLane.Lane3 };

        [Tooltip("라벨 이미지 생성")]
        [SerializeField] private BarcodeLabelPrinter labelPrinter;

        [Tooltip("레인별 투입 허용 확인 (DestinationLane 순서)")]
        [SerializeField] private LaneOccupancyMonitor[] laneMonitors = new LaneOccupancyMonitor[3];

        [Tooltip("전담 공정. 적재 실패나 파레트 완료로 멈춘 레인의 신규 라벨은 생성하지 않는다")]
        [SerializeField] private LaneProcessController[] laneProcesses = new LaneProcessController[3];

        [Tooltip("투입점 주변 비움 확인 반경 (m)")]
        [SerializeField, Min(0.05f)] private float clearRadius = 0.3f;

        [Tooltip("직전 생성 박스가 투입점에서 이 거리(m) 이상 멀어진 뒤에만 생성한다. 투입선이 합류 보류로 멈췄다 풀리면 앞 박스가 투입점 0.45 m 앞에 서 있어 " +
                 "두 박스가 붙어 나가고, 분기 장치 밀판(0.8 m)이 하나만 밀 수 없어 계속 순환한다 (11/2 시험). 밀판 판정에 필요한 틈 약 0.52 m + 박스 길이 + 여유")]
        [SerializeField, Min(0f)] private float minSpacing = 1.0f;

        [Tooltip("생성한 박스를 모아 둘 부모 (비우면 이 오브젝트)")]
        [SerializeField] private Transform boxContainer;
        #endregion

        #region 2. 상태
        private readonly List<LogisticsBox> spawned = new List<LogisticsBox>();
        private readonly List<DestinationLane> availableLanes = new List<DestinationLane>();
        private readonly Collider[] hits = new Collider[8];
        private Coroutine loop;
        private float interval = 6f;

        public bool IsSpawning => loop != null;
        public int SpawnedCount { get; private set; }
        public bool IsFeedBlocked { get; private set; }
        public IReadOnlyList<LogisticsBox> SpawnedBoxes => spawned;

        // 생성 알림. 두 번째 인자는 붙인 라벨의 레인 (기록·시험 비교용이며 판독 결과로 쓰지 않는다)
        public event Action<LogisticsBox, DestinationLane> BoxSpawned;
        #endregion

        #region 3. 투입
        // 투입 시작
        public void Begin(float intervalSeconds)
        {
            interval = Mathf.Max(0.5f, intervalSeconds);
            if (loop == null)
            {
                loop = StartCoroutine(SpawnLoop());
            }
        }

        // 투입 간격만 바꾼다 (속도 배율 변경)
        public void SetInterval(float intervalSeconds)
        {
            interval = Mathf.Max(0.5f, intervalSeconds);
        }

        // 투입 중지
        public void Stop()
        {
            if (loop != null)
            {
                StopCoroutine(loop);
                loop = null;
            }
        }

        // 투입점이 비면 무작위 바코드를 선택해 생성하고 라벨을 붙이는 코루틴
        private IEnumerator SpawnLoop()
        {
            while (true)
            {
                //1. 투입점이 비었을 때만 추첨한다. 기다리는 동안에는 재추첨하지 않는다.
                DestinationLane lane;
                while (IsFeedBlocked || !IsSpawnPointClear() || !IsSpacingClear() || !TryNextLane(out lane))
                {
                    yield return new WaitForSeconds(0.25f);
                }

                //2. 생성
                Spawn(lane);
                yield return new WaitForSeconds(interval);
            }
        }

        private bool TryNextLane(out DestinationLane lane)
        {
            lane = DestinationLane.Lane1;
            if (labelSequence == null || labelSequence.Length == 0)
            {
                return false;
            }
            availableLanes.Clear();
            for (int i = 0; i < labelSequence.Length; i++)
            {
                DestinationLane candidate = labelSequence[i];
                LaneOccupancyMonitor monitor = MonitorFor(candidate);
                LaneProcessController process = laneProcesses != null ? Array.Find(laneProcesses, p => p != null && p.Lane == candidate) : null;
                bool unavailable = process != null && (process.IsHaltedByFailure || (process.UsesRobot && (process.Pallet.IsComplete || !process.IsPalletReady)));
                if (!unavailable && (monitor == null || !monitor.IsCongested) && !availableLanes.Contains(candidate))
                {
                    availableLanes.Add(candidate);
                }
            }
            if (availableLanes.Count == 0)
            {
                return false;
            }
            lane = availableLanes[UnityEngine.Random.Range(0, availableLanes.Count)];
            return true;
        }

        // 투입 컨베이어 정지와 함께 생성만 보류한다. 기존 투입 간격과 코루틴은 유지한다.
        public void SetFeedBlocked(bool blocked) => IsFeedBlocked = blocked;

        private LaneOccupancyMonitor MonitorFor(DestinationLane lane)
        {
            int i = (int)lane;
            return laneMonitors != null && i < laneMonitors.Length ? laneMonitors[i] : null;
        }

        private bool IsSpawnPointClear()
        {
            int n = Physics.OverlapSphereNonAlloc(spawnPoint.position, clearRadius, hits, ~0, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < n; i++)
            {
                if (hits[i].attachedRigidbody != null && hits[i].attachedRigidbody.GetComponent<LogisticsBox>() != null)
                {
                    return false;
                }
            }
            return true;
        }

        // 직전 생성 박스가 최소 간격만큼 멀어졌는지 (제거됐거나 들어 올려졌으면 통과)
        private bool IsSpacingClear()
        {
            if (spawned.Count == 0)
            {
                return true;
            }
            LogisticsBox last = spawned[spawned.Count - 1];
            if (last == null || last.Body == null || last.Body.isKinematic)
            {
                return true;
            }
            return (last.transform.position - spawnPoint.position).sqrMagnitude >= minSpacing * minSpacing;
        }

        private LogisticsBox Spawn(DestinationLane lane)
        {
            int typeIndex = (int)spawnBoxType;
            GameObject prefab = boxPrefabs != null && typeIndex < boxPrefabs.Length ? boxPrefabs[typeIndex] : null;
            if (prefab == null || spawnPoint == null)
            {
                Debug.LogWarning("[BoxSpawner] 박스 Prefab 또는 투입점이 연결되지 않음", this);
                return null;
            }

            SpawnedCount++;
            GameObject go = Instantiate(prefab, spawnPoint.position, spawnPoint.rotation, boxContainer != null ? boxContainer : transform);
            go.name = $"Box_{SpawnedCount:0000}";
            LogisticsBox box = go.GetComponent<LogisticsBox>();
            if (box == null)
            {
                Debug.LogWarning("[BoxSpawner] 박스 Prefab에 LogisticsBox가 없음", go);
                return null;
            }
            if (labelPrinter != null)
            {
                box.ApplyLabel(labelPrinter.GetLabel(lane));
            }
            spawned.Add(box);
            BoxSpawned?.Invoke(box, lane);
            return box;
        }

        // 박스를 공정에서 뺀다 (적재 완료·반출 후 정리용)
        public void Despawn(LogisticsBox box)
        {
            if (box == null)
            {
                return;
            }
            spawned.Remove(box);
            Destroy(box.gameObject);
        }

        // 초기화 버튼: 투입 중지, 씬의 모든 박스 제거, 투입 누계 0, 라벨 후보 임시 목록 비움
        public void ResetToInitialState()
        {
            Stop();
            IsFeedBlocked = false;
            SpawnedCount = 0;
            foreach (LogisticsBox box in spawned)
            {
                if (box != null)
                {
                    Destroy(box.gameObject);
                }
            }
            spawned.Clear();
            availableLanes.Clear();
        }
        #endregion
    }
}
