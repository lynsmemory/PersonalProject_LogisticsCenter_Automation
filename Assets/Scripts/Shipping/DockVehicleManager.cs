using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Logistics
{
    // N개 만재 후 AMR이 빠져나오면 차량을 숨기고 화물을 정리한 뒤 빈 차량을 도크에 다시 표시한다. (FR-16, FR-17, FR-20)
    // 레인당 1개. 차량 적재함 자리는 차량 Prefab의 PalletSlots/PalletSlot_01~10을 이름 순서로 쓴다 (01·02가 가장 안쪽, 안쪽부터 채운다).
    // 상차한 파레트는 자리(PalletSlot)의 자식으로 붙고 차량 교체 때 정리한다. 차량 물체를 재사용해 도크·AMR 전담 관계를 유지한다.
    // N개째 상차로 만재가 되면 상차한 AMR이 적재함을 빠져나왔다고 알릴 때(LoaderCleared) 출발한다.
    public class DockVehicleManager : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("도크 전담 차량 (WingBody5t 루트)")]
        [SerializeField] private Transform truck;

        [Tooltip("정차 위치. 비우면 운전 시작 전 차량 위치를 쓴다")]
        [SerializeField] private Transform dockPoint;

        [Tooltip("적재함 파레트 자리. 비우면 차량의 PalletSlots 자식을 이름 순서로 쓴다")]
        [SerializeField] private Transform[] cargoSlots = new Transform[0];

        [Tooltip("만재 차량 출발 후 실린 박스 정리")]
        [SerializeField] private BoxSpawner boxSpawner;

        [Tooltip("만재 판정 후 출발까지 대기 (초)")]
        [SerializeField, Min(0f)] private float departDelaySeconds = 1f;

        [Tooltip("만재 차량이 사라진 뒤 빈 차량이 다시 나타나기까지 대기 시간 (초)")]
        [SerializeField, Min(0.5f)] private float arriveSeconds = 5f;
        #endregion

        #region 2. 상태
        private readonly List<Transform> cargo = new List<Transform>();
        private Vector3 dockPosition;
        private Quaternion dockRotation;
        private Transform dockReference;
        private int capacity = 10;

        public Transform Truck => truck;
        public int LoadedPalletCount { get; private set; }   // 현재 차량 상차 수
        public int Capacity => capacity;                       // 만재 파레트 수 N
        public bool IsFull => LoadedPalletCount >= capacity;
        public bool IsDeparting { get; private set; }          // 만재 출발·빈 차량 진입 중
        public int DepartureCount { get; private set; }        // 출발한 차량 수
        public int TotalLoadedPallets { get; private set; }    // 누적 상차 파레트 수
        // 다음 파레트를 실을 수 있음 (교체 중이 아니고 만재가 아니고 남은 자리가 있음)
        public bool CanAcceptPallet => truck != null && truck.gameObject.activeInHierarchy && !IsDeparting && !IsFull && LoadedPalletCount < cargoSlots.Length;
        // 차량 뒤쪽에서 앞쪽(운전석)으로 들어가는 방향 (수평)
        public Vector3 InDirection => truck != null ? Flat(truck.forward) : Vector3.forward;
        // 적재함 뒤쪽 끝 기준점 (차량의 DockReference, 없으면 차량 위치)
        public Vector3 RearPoint => dockReference != null ? dockReference.position : (truck != null ? truck.position : transform.position);

        public event Action<int> PalletLoaded;   // 상차 수 갱신 (현재 차량 상차 수)
        public event Action Departed;            // 만재 차량 출발 (실린 파레트 제거)
        public event Action Replaced;            // 빈 차량 도크 배치 완료
        #endregion

        #region 3. 상차
        private void Awake()
        {
            if (truck == null)
            {
                return;
            }
            dockPosition = dockPoint != null ? dockPoint.position : truck.position;
            dockRotation = dockPoint != null ? dockPoint.rotation : truck.rotation;
            dockReference = truck.Find("DockReference");
            if (cargoSlots == null || cargoSlots.Length == 0)
            {
                Transform root = truck.Find("PalletSlots");
                var found = new List<Transform>();
                if (root != null)
                {
                    foreach (Transform slot in root) found.Add(slot);
                }
                found.Sort((a, b) => string.CompareOrdinal(a.name, b.name));
                cargoSlots = found.ToArray();
            }
            if (cargoSlots.Length == 0)
            {
                Debug.LogWarning($"[DockVehicleManager] {name}: 차량 적재함 자리(PalletSlots)가 없음", this);
            }
        }

        // 만재 수량 N 적용. 상차 중에는 바꾸지 않는다
        public void Configure(int palletCapacity)
        {
            if (LoadedPalletCount > 0)
            {
                Debug.LogWarning($"[DockVehicleManager] {name}: 상차 중({LoadedPalletCount}개)에는 만재 수량을 바꾸지 않음", this);
                return;
            }
            capacity = Mathf.Max(1, palletCapacity);
            if (cargoSlots.Length > 0 && capacity > cargoSlots.Length)
            {
                Debug.LogWarning($"[DockVehicleManager] {name}: N={capacity}이 적재함 자리 {cargoSlots.Length}개보다 많아 {cargoSlots.Length}개로 제한", this);
                capacity = cargoSlots.Length;
            }
        }

        // 다음 상차 자리. 실을 수 없으면 null
        public Transform GetNextCargoSlot()
        {
            return CanAcceptPallet ? cargoSlots[LoadedPalletCount] : null;
        }

        // 상차 성공한 파레트 1개 등록: 상차 수 +1, N이면 출발·교체
        public void RegisterLoaded(Transform pallet)
        {
            if (pallet != null)
            {
                cargo.Add(pallet);
            }
            LoadedPalletCount++;
            TotalLoadedPallets++;
            Debug.Log($"[DockVehicleManager] {name}: 상차 {LoadedPalletCount}/{capacity}", this);
            PalletLoaded?.Invoke(LoadedPalletCount);
            if (IsFull)
            {
                Debug.Log($"[DockVehicleManager] {name}: 만재 판정 {LoadedPalletCount}/{capacity}. AMR이 빠져나오면 출발", this);
            }
        }

        // 상차한 AMR이 적재함 밖(도크 접근점)으로 나옴. 만재면 출발·교체를 시작한다
        public void LoaderCleared()
        {
            if (IsFull && !IsDeparting)
            {
                StartCoroutine(DepartAndReplaceTruck());
            }
        }

        // 사용자 지정: 출발 이동 없이 만재 차량 소멸 표시 → 빈 차량 재등장.
        // 기존 차량 인스턴스를 재사용해 도크·슬롯·AMR 연결을 유지한다.
        private IEnumerator DepartAndReplaceTruck()
        {
            IsDeparting = true;
            Debug.Log($"[DockVehicleManager] {name}: 만재 차량 출발 ({LoadedPalletCount}/{capacity})", this);
            yield return new WaitForSeconds(departDelaySeconds);
            truck.gameObject.SetActive(false);
            ClearCargo();
            LoadedPalletCount = 0;
            DepartureCount++;
            Departed?.Invoke();

            yield return new WaitForSeconds(arriveSeconds);
            truck.SetPositionAndRotation(dockPosition, dockRotation);
            truck.gameObject.SetActive(true);
            IsDeparting = false;
            Debug.Log($"[DockVehicleManager] {name}: 빈 차량 도크 배치 (출발 {DepartureCount}대)", this);
            Replaced?.Invoke();
        }

        // 실린 파레트와 그 위 박스 제거 (박스는 BoxSpawner 목록에서도 뺀다)
        private void ClearCargo()
        {
            foreach (Transform pallet in cargo)
            {
                if (pallet == null)
                {
                    continue;
                }
                foreach (LogisticsBox box in pallet.GetComponentsInChildren<LogisticsBox>())
                {
                    if (boxSpawner != null) boxSpawner.Despawn(box);
                }
                pallet.gameObject.SetActive(false);
                Destroy(pallet.gameObject);
            }
            cargo.Clear();
        }

        // 초기화 버튼: 차량의 파레트·박스 제거, 차량을 도크 위치에 빈 상태로 배치
        public void ResetToInitialState()
        {
            StopAllCoroutines();
            ClearCargo();
            if (truck != null)
            {
                truck.SetPositionAndRotation(dockPosition, dockRotation);
                truck.gameObject.SetActive(true);
            }
            LoadedPalletCount = 0;
            TotalLoadedPallets = 0;
            DepartureCount = 0;
            IsDeparting = false;
        }

        private static Vector3 Flat(Vector3 v)
        {
            v.y = 0f;
            return v.sqrMagnitude > 1e-6f ? v.normalized : Vector3.forward;
        }
        #endregion
    }
}
