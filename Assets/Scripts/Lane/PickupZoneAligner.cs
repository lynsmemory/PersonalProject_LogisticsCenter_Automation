using System;
using UnityEngine;

namespace Logistics
{
    // 레인을 내려온 박스가 픽업 구역에 도착하면 PickupPoint에 지정한 위치·회전값으로 박스를 옮겨 놓고 FR5가 집을 수 있게 한다.
    // 픽업 구역이 비었는지도 알려 준다. ML(CNN)을 추가하면 카메라로 박스 위치·회전을 확인해 집는 방식으로 바꾼다(FR-24). (FR-19, FR-22)
    // PickupPoint에 붙인다. 도착 감지는 자식 PickupArrivalSensor 트리거 영역을 직접 조회한다(자식 트리거 콜백은 부모로 오지 않음).
    // 박스가 끝 스토퍼에 닿아 거의 멈추면(settleSpeed 이하가 settleTime 유지) 지정 위치로 옮기고 kinematic으로 고정한다.
    public class PickupZoneAligner : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("박스를 놓을 지정 위치·회전 (박스 중심)")]
        [SerializeField] private Transform pickupPoint;

        [Tooltip("도착 감지 (PickupArrivalSensor 트리거)")]
        [SerializeField] private Collider arrivalSensor;

        [Tooltip("정지로 볼 박스 속도 (m/s)")]
        [SerializeField, Min(0f)] private float settleSpeed = 0.05f;

        [Tooltip("정지 상태 유지 시간 (초)")]
        [SerializeField, Min(0f)] private float settleTime = 0.3f;

        [Tooltip("정렬 허용 오차: 이보다 멀면 경고 (m)")]
        [SerializeField, Min(0.01f)] private float maxCorrection = 0.3f;

        [Tooltip("도착 감지 영역을 레인 상류(PickupPoint 로컬 +X)로 늘리는 길이 (m). 말단 수평부 입구에서 비스듬히 멈춘 박스도 정렬한다 (10/29 시험: 센서 4mm 앞 정지)")]
        [SerializeField, Min(0f)] private float approachLength = 0.25f;
        #endregion

        #region 2. 상태
        private readonly Collider[] hits = new Collider[8];
        private LogisticsBox candidate;
        private float settledFor;

        // 픽업 구역의 박스 (정렬 완료된 것)
        public LogisticsBox CurrentBox { get; private set; }
        // 구역이 비었음 (도착 중인 박스도 없음)
        public bool IsEmpty => CurrentBox == null && candidate == null;
        public Transform PickupPoint => pickupPoint;
        // 마지막 정렬 전 위치 오차 (m, 기록용)
        public float LastCorrection { get; private set; }

        // 박스 정렬 완료
        public event Action<LogisticsBox> BoxAligned;
        #endregion

        #region 3. 도착 감지와 정렬
        private void FixedUpdate()
        {
            if (CurrentBox != null || arrivalSensor == null)
            {
                return;
            }

            //1. 감지 영역 안의 움직이는 박스
            Bounds b = arrivalSensor.bounds;
            if (pickupPoint != null && approachLength > 0f)
            {
                Vector3 upstream = pickupPoint.right;
                b.Encapsulate(b.center + upstream * (Vector3.Dot(b.extents, new Vector3(Mathf.Abs(upstream.x), Mathf.Abs(upstream.y), Mathf.Abs(upstream.z))) + approachLength));
            }
            // 자기가 속한 장면의 물리에서 찾는다 (시험용 미리보기 장면에서도 같은 동작)
            int n = gameObject.scene.GetPhysicsScene().OverlapBox(b.center, b.extents, hits, Quaternion.identity, ~0, QueryTriggerInteraction.Ignore);
            LogisticsBox found = null;
            for (int i = 0; i < n; i++)
            {
                Rigidbody body = hits[i].attachedRigidbody;
                LogisticsBox box = body != null && !body.isKinematic ? body.GetComponent<LogisticsBox>() : null;
                if (box != null && !box.IsHeld)   // 흡착 툴이 들어 올리는 중인 박스는 다시 정렬하지 않는다
                {
                    found = box;
                    break;
                }
            }

            //2. 멈춘 상태가 이어지면 정렬
            if (found != candidate)
            {
                candidate = found;
                settledFor = 0f;
            }
            if (candidate == null)
            {
                return;
            }
            settledFor = candidate.Body.linearVelocity.magnitude <= settleSpeed ? settledFor + Time.fixedDeltaTime : 0f;
            if (settledFor >= settleTime)
            {
                AlignToPickupPoint(candidate);
            }
        }

        // 박스를 지정 위치·회전으로 옮기고 정지시킴
        public void AlignToPickupPoint(LogisticsBox box)
        {
            if (box == null || pickupPoint == null)
            {
                return;
            }
            LastCorrection = Vector3.Distance(box.transform.position, pickupPoint.position);
            if (LastCorrection > maxCorrection)
            {
                Debug.LogWarning($"[PickupZoneAligner] {box.name} 위치 오차 {LastCorrection:F3}m가 허용값보다 큼", box);
            }

            Rigidbody body = box.Body;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            body.isKinematic = true;
            body.position = pickupPoint.position;
            body.rotation = pickupPoint.rotation;
            box.transform.SetPositionAndRotation(pickupPoint.position, pickupPoint.rotation);

            CurrentBox = box;
            candidate = null;
            settledFor = 0f;
            BoxAligned?.Invoke(box);
        }

        // FR5가 박스를 가져가면(흡착 후) 구역을 비운다
        public void ClearZone()
        {
            CurrentBox = null;
            candidate = null;
            settledFor = 0f;
        }

        // 초기화 버튼: 구역 비움 (박스는 BoxSpawner가 제거)
        public void ResetToInitialState()
        {
            ClearZone();
        }
        #endregion
    }
}
