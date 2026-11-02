using System;
using System.Collections.Generic;
using UnityEngine;

namespace Logistics
{
    // 레인별 순환 대기 수를 세고, 순환 3개에서 정체, 2개 유지, 1개 이하에서 해제한다. (FR-19)
    // 순환 대기 = 이 레인으로 판독됐지만 분기 장치를 분기 없이 지나 메인을 다시 도는 박스 (TC-05의 '분기하지 못한 박스').
    // 판독만 되고 아직 분기 장치에 오지 않은 박스는 세지 않는다. 판독 시점부터 세면 투입 간격 6초에서 이동 중 박스만으로 3개가 넘어
    // 정상 흐름에서도 정체가 난다 (10/30 시험에서 확인).
    // 정체 중에는 새 투입만 막고(BoxSpawner가 IsCongested 확인) 순환 중인 박스 분기는 계속 허용한다.
    // 레인 안 대기는 중간 스토퍼 앞 1개로 제한한다: 분기한 뒤 아직 중간 스토퍼를 지나지 않은 박스가 있으면
    // 분기 장치의 AcceptsBoxes를 끄고, 스토퍼가 그 박스를 방출하면 다시 켠다. 픽업 1개 제한은 PickupZoneAligner·LaneProcessController가 지킨다.
    public class LaneOccupancyMonitor : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("감시 레인")]
        [SerializeField] private DestinationLane lane = DestinationLane.Lane1;

        [Tooltip("판독 완료 수신 (레인별 판독 수 집계)")]
        [SerializeField] private BarcodeScanner barcodeScanner;

        [Tooltip("분기 성공·분기 없이 통과 수신, 레인 수용 여부 설정")]
        [SerializeField] private LaneDiverter diverter;

        [Tooltip("중간 스토퍼 방출 수신")]
        [SerializeField] private MiddleStopper middleStopper;

        [Tooltip("정체 판정 순환 대기 수")]
        [SerializeField, Min(1)] private int congestAt = 3;

        [Tooltip("정체 해제 순환 대기 수 (이하)")]
        [SerializeField, Min(0)] private int releaseAt = 1;
        #endregion

        #region 2. 상태
        private readonly HashSet<Rigidbody> circulating = new HashSet<Rigidbody>();
        private readonly HashSet<Rigidbody> inLane = new HashSet<Rigidbody>();

        public DestinationLane Lane => lane;
        // 순환 대기 수 (판독은 됐고 아직 분기하지 못한 이 레인 박스)
        public int CirculatingCount => circulating.Count;
        // 분기했지만 아직 중간 스토퍼를 지나지 않은 박스 수 (최대 1)
        public int LaneQueueCount => inLane.Count;
        public bool IsCongested { get; private set; }
        // 이 레인으로 판독된 박스 수 (누적)
        public int ReadCount { get; private set; }

        // 정체 알람
        public event Action<bool> CongestionChanged;
        #endregion

        #region 3. 연결
        private void OnEnable()
        {
            if (barcodeScanner != null) barcodeScanner.BoxRead += OnBoxRead;
            if (diverter != null) diverter.Diverted += OnBoxDiverted;
            if (diverter != null) diverter.BoxPassed += OnBoxPassed;
            if (middleStopper != null) middleStopper.Released += OnBoxReleased;
        }

        private void OnDisable()
        {
            if (barcodeScanner != null) barcodeScanner.BoxRead -= OnBoxRead;
            if (diverter != null) diverter.Diverted -= OnBoxDiverted;
            if (diverter != null) diverter.BoxPassed -= OnBoxPassed;
            if (middleStopper != null) middleStopper.Released -= OnBoxReleased;
        }
        #endregion

        #region 4. 집계
        // 이 레인 박스 판독 수 +1 (순환 대기는 분기 장치를 지날 때 센다)
        public void OnBoxRead(LogisticsBox box)
        {
            if (box != null && box.HasReadResult && box.ReadLane == lane)
            {
                ReadCount++;
            }
        }

        // 분기 없이 분기 장치를 지나면 순환 대기 +1 (같은 박스는 한 번만)
        public void OnBoxPassed(Rigidbody body)
        {
            circulating.Add(body);
            UpdateCongestion();
        }

        // 분기 성공 시 순환 대기 −1, 레인 대기 +1
        public void OnBoxDiverted(Rigidbody body)
        {
            circulating.Remove(body);
            inLane.Add(body);
            UpdateCongestion();
            UpdateAcceptance();
        }

        // 중간 스토퍼가 방출하면 레인 대기 −1
        public void OnBoxReleased(Rigidbody body)
        {
            inLane.Remove(body);
            UpdateAcceptance();
        }

        // 3/2/1 기준으로 정체 판정
        public void UpdateCongestion()
        {
            circulating.RemoveWhere(b => b == null);
            bool congested = IsCongested;
            if (!IsCongested && circulating.Count >= congestAt) congested = true;
            else if (IsCongested && circulating.Count <= releaseAt) congested = false;
            if (congested != IsCongested)
            {
                IsCongested = congested;
                Debug.Log($"[LaneOccupancyMonitor {lane}] 정체 {(congested ? "발생" : "해제")} (순환 대기 {circulating.Count})", this);
                CongestionChanged?.Invoke(congested);
            }
        }

        private void UpdateAcceptance()
        {
            inLane.RemoveWhere(b => b == null);
            if (diverter != null)
            {
                diverter.AcceptsBoxes = inLane.Count == 0;
            }
        }

        // 초기화 버튼: 집계 0, 정체 해제
        public void ResetToInitialState()
        {
            circulating.Clear();
            inLane.Clear();
            ReadCount = 0;
            bool was = IsCongested;
            IsCongested = false;
            UpdateAcceptance();
            if (was)
            {
                CongestionChanged?.Invoke(false);
            }
        }
        #endregion
    }
}
