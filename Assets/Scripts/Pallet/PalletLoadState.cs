using System;
using UnityEngine;

namespace Logistics
{
    // 파레트 1개의 사전 지정 적재 위치(층당 6자리)와 적재 수를 관리한다. (FR-10, FR-11, FR-12)
    // 자리 순서는 layerSlots 배열 순서이며, 한 층 6자리를 모두 채워야 다음 층 자리를 준다. 6×L개를 놓으면 완료다.
    // 적재 수는 PalletizingSequence가 놓은 위치를 확인한 뒤 ConfirmPlaced를 부를 때만 늘어난다(실패한 자리는 세지 않는다).
    // 레인마다 파레트 적재 기준점(RobotCells/LaneN_RobotCell_*/PalletLoadPoint)에 1개 붙인다.
    // 적재 자리는 이 기준점의 자식이므로 파레트 물체(palletObject)를 새로 만들어도 자리 지정은 그대로다.
    // AMR이 완성 파레트를 가져가면(TakeCurrentPallet) 처음 파레트가 있던 위치에 빈 파레트를 다시 만든다(RespawnEmptyPallet). 별도 공급 공정은 없다. (FR-20)
    [DisallowMultipleComponent]
    public class PalletLoadState : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("한 층 6자리 지정 위치 (1단, 파레트 윗면). 위치 = 박스 바닥 중심, 회전 = 박스 방향. 배열 순서가 적재 순서")]
        [SerializeField] private Transform[] layerSlots = new Transform[SimulationSettings.BoxesPerLayer];

        [Tooltip("층 높이 (m). 박스 높이 0.17")]
        [SerializeField, Min(0.01f)] private float layerHeight = 0.17f;

        [Tooltip("적재 층 수 L. 운전 시작 때 설정값(SimulationSettings)으로 바뀐다")]
        [SerializeField, Min(1)] private int layerCount = 3;

        [Tooltip("지금 이 자리에 놓인 파레트 물체 (AMR 인양 대상). 처음 위치가 빈 파레트 재생성 위치가 된다")]
        [SerializeField] private Transform palletObject;

        [Tooltip("빈 파레트 재생성용 Prefab (Pallet_460x400x150)")]
        [SerializeField] private GameObject emptyPalletPrefab;
        #endregion

        #region 2. 상태
        public int PlacedCount { get; private set; }                                  // 적재 완료 수
        public int CurrentLayer => PlacedCount / SimulationSettings.BoxesPerLayer;      // 지금 채우는 층 (0부터)
        public int LayerCount => layerCount;
        public int Capacity => SimulationSettings.BoxesPerLayer * layerCount;          // 6 × L
        public bool IsComplete => PlacedCount >= Capacity;
        public float LayerHeight => layerHeight;

        // L층을 모두 쌓았을 때 맨 위 박스 윗면 높이 (월드 Y). 운반 높이 계산용
        public float StackTopHeight => transform.position.y + layerHeight * layerCount;

        public event Action BoxPlaced;   // 박스 1개 적재 완료 (기록용)
        public event Action Completed;   // 6×L 완료 (1회)
        public event Action PalletRespawned;   // 빈 파레트 재생성 (다음 적재 주기 시작)

        // 지금 이 자리의 파레트 물체. AMR이 가져간 뒤 재생성 전까지 null
        public Transform CurrentPallet => palletObject;
        public int RespawnCount { get; private set; }
        public string LastRespawnFailure { get; private set; }   // 마지막 재생성 실패 이유 (성공하면 null)
        // 적재 준비됨: 파레트 물체가 이 자리에 있고 재생성 실패 상태가 아님. 아니면 방출·적재·라벨 투입을 막는다
        public bool IsReady => palletObject != null && LastRespawnFailure == null;
        // 반출 후 또는 초기화 중 재생성이 실패해 복구가 필요함 (레인이 주기적으로 RespawnEmptyPallet을 다시 시도)
        public bool NeedsRespawn => palletObject == null && LastRespawnFailure != null;

        private Vector3 homePosition;
        private Quaternion homeRotation;
        private Transform homeParent;
        private bool homeSaved;
        #endregion

        #region 3. 적재
        private void Awake()
        {
            if (palletObject != null)
            {
                homePosition = palletObject.position;
                homeRotation = palletObject.rotation;
                homeParent = palletObject.parent;
                homeSaved = true;
            }
        }

        // 적재 층 수 L 적용. 적재 중에는 바꾸지 않는다
        public void Configure(int layers)
        {
            if (PlacedCount > 0)
            {
                Debug.LogWarning($"[PalletLoadState] {name}: 적재 중({PlacedCount}개)에는 층 수를 바꾸지 않음", this);
                return;
            }
            layerCount = Mathf.Max(1, layers);
        }

        // 다음 적재 자리 (박스 바닥 중심 위치와 박스 방향). 완료됐거나 자리 지정이 없으면 false
        public bool GetNextSlotPose(out Pose pose)
        {
            pose = default;
            if (IsComplete || !IsReady)
            {
                return false;
            }
            Transform slot = layerSlots[PlacedCount % SimulationSettings.BoxesPerLayer];
            if (slot == null)
            {
                return false;
            }
            pose = new Pose(slot.position + Vector3.up * (layerHeight * CurrentLayer), slot.rotation);
            return true;
        }

        // 놓은 위치 확인이 끝난 박스 1개를 센다. 6×L이 되면 Completed를 한 번 알린다
        public void ConfirmPlaced()
        {
            if (IsComplete)
            {
                return;
            }
            PlacedCount++;
            BoxPlaced?.Invoke();
            if (IsComplete)
            {
                Completed?.Invoke();
            }
        }

        // AMR 인양: 지금 파레트 물체를 넘기고 이 자리를 비운다. 적재 수는 빈 파레트를 다시 만들 때 0이 된다
        public Transform TakeCurrentPallet()
        {
            Transform taken = palletObject;
            palletObject = null;
            return taken;
        }

        // 완성 파레트 반출 후 처음 위치에 빈 파레트를 만든다 (FR-20). 실제로 만들어졌을 때만 적재 수를 0으로 하고 true를 돌려준다.
        // 실패(Prefab·처음 위치 없음, 자리 점유, 생성 결과 이상)하면 적재 수를 그대로 두어 다음 적재 주기가 시작되지 않게 하고 이유를 남긴다
        public bool RespawnEmptyPallet(out string reason)
        {
            if (palletObject == null && !TryCreatePallet(out reason))
            {
                LastRespawnFailure = reason;
                Debug.LogWarning($"[PalletLoadState] {name}: 빈 파레트 재생성 실패 ({reason}). 적재 대기", this);
                return false;
            }
            reason = null;
            LastRespawnFailure = null;
            PlacedCount = 0;
            RespawnCount++;
            PalletRespawned?.Invoke();
            return true;
        }

        // 처음 위치가 비어 있는지 확인하고 빈 파레트를 만든 뒤, 충돌체·Rigidbody·위치를 확인한다
        private bool TryCreatePallet(out string reason)
        {
            if (emptyPalletPrefab == null || !homeSaved)
            {
                reason = "빈 파레트 Prefab 또는 처음 위치 연결 없음";
                return false;
            }
            Physics.SyncTransforms();   // 같은 프레임에 옮긴 AMR·파레트 위치를 조회에 반영
            Collider[] blocking = Physics.OverlapBox(homePosition + Vector3.up * 0.09f, new Vector3(0.22f, 0.07f, 0.19f), homeRotation, ~0, QueryTriggerInteraction.Ignore);
            foreach (Collider other in blocking)
            {
                if (other.bounds.max.y <= homePosition.y + 0.02f) continue;   // 바닥
                reason = $"자리 점유 ({other.name})";
                return false;
            }
            GameObject created = Instantiate(emptyPalletPrefab, homePosition, homeRotation, homeParent);
            if (created == null)
            {
                reason = "생성 실패";
                return false;
            }
            created.name = "Pallet";
            if (created.GetComponent<Rigidbody>() == null || created.GetComponentInChildren<Collider>() == null || (created.transform.position - homePosition).sqrMagnitude > 1e-6f)
            {
                created.SetActive(false);   // 같은 프레임 재조회에 걸리지 않게 끄고 지운다
                Destroy(created);
                reason = "생성한 파레트의 Rigidbody·충돌체·위치 이상";
                return false;
            }
            palletObject = created.transform;
            reason = null;
            return true;
        }

        // 초기화 버튼: 적재 수 0, 층 0 (쌓인 박스 제거는 BoxSpawner가 한다)
        // 파레트가 반출되어 없으면 빈 파레트를 다시 만들고, 남아 있으면 처음 위치로 되돌린다
        public void ResetToInitialState()
        {
            PlacedCount = 0;
            RespawnCount = 0;
            LastRespawnFailure = null;
            if (palletObject == null)
            {
                if (!TryCreatePallet(out string reason))
                {
                    LastRespawnFailure = reason;
                    Debug.LogWarning($"[PalletLoadState] {name}: 초기화 중 빈 파레트 재생성 실패 ({reason})", this);
                }
            }
            else if (homeSaved)
            {
                palletObject.SetPositionAndRotation(homePosition, homeRotation);
                var body = palletObject.GetComponent<Rigidbody>();
                if (body != null && !body.isKinematic)
                {
                    body.linearVelocity = Vector3.zero;
                    body.angularVelocity = Vector3.zero;
                }
            }
        }
        #endregion
    }
}
