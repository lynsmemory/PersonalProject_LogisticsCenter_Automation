using System;
using System.Collections;
using UnityEngine;

namespace Logistics
{
    // 경사 레인 중간에서 박스 1개를 대기시키고, 명령을 받으면 1개만 통과시킨 뒤 닫는다. 끝 스토퍼와 별도 설비. (FR-19, FR-22)
    // MiddleStopper 오브젝트(레인 경사에 맞춰 회전됨)에 붙인다. 막대(StopperBody, kinematic)를 ClosedPose(막음)와
    // OpenPose(레인 면 아래) 사이로 옮긴다. 대기 박스 감지는 HoldSensor 트리거 영역을 직접 조회한다.
    public class MiddleStopper : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("스토퍼 막대 (kinematic Rigidbody)")]
        [SerializeField] private Rigidbody stopperBody;

        [Tooltip("스토퍼 앞 박스 감지 (HoldSensor 트리거)")]
        [SerializeField] private Collider holdSensor;

        [Tooltip("막대가 박스를 막는 위치")]
        [SerializeField] private Transform closedPose;

        [Tooltip("막대가 레인 면 아래로 내려간 위치")]
        [SerializeField] private Transform openPose;

        [Tooltip("열기·닫기 이동 시간 (초)")]
        [SerializeField, Min(0.05f)] private float moveTime = 0.25f;

        [Tooltip("방출한 박스가 감지 영역을 떠난 뒤 닫기까지 여유 (초)")]
        [SerializeField, Min(0f)] private float closeDelay = 0.15f;

        [Tooltip("박스가 떠나지 않으면 닫는 제한 시간 (초)")]
        [SerializeField, Min(0.5f)] private float releaseTimeout = 4f;
        #endregion

        #region 2. 상태
        private readonly Collider[] hits = new Collider[8];
        private float openAmount;   // 0 닫힘, 1 열림
        private float targetOpen;
        private Coroutine releasing;

        public bool IsOpen => openAmount > 0.99f;
        public bool IsBusy => releasing != null;
        // 대기 박스 있음
        public bool HasWaitingBox => FindWaitingBox() != null;

        // 박스 1개 방출 완료 (박스가 스토퍼를 지남)
        public event Action<Rigidbody> Released;
        #endregion

        #region 3. 막대 이동
        private void Awake()
        {
            if (stopperBody != null)
            {
                stopperBody.isKinematic = true;
                stopperBody.useGravity = false;
            }
        }

        private void FixedUpdate()
        {
            if (stopperBody == null || closedPose == null || openPose == null)
            {
                return;
            }
            openAmount = Mathf.MoveTowards(openAmount, targetOpen, Time.fixedDeltaTime / moveTime);
            stopperBody.MovePosition(Vector3.Lerp(closedPose.position, openPose.position, openAmount));
            stopperBody.MoveRotation(closedPose.rotation);
        }
        #endregion

        #region 4. 방출
        // 감지 영역 안의 박스 (가장 앞 1개)
        public Rigidbody FindWaitingBox()
        {
            if (holdSensor == null)
            {
                return null;
            }
            Bounds b = holdSensor.bounds;
            int n = Physics.OverlapBoxNonAlloc(b.center, b.extents, hits, Quaternion.identity, ~0, QueryTriggerInteraction.Ignore);
            Rigidbody nearest = null;
            float best = float.MaxValue;
            for (int i = 0; i < n; i++)
            {
                Rigidbody body = hits[i].attachedRigidbody;
                if (body == null || body.isKinematic || body.GetComponent<LogisticsBox>() == null)
                {
                    continue;
                }
                float d = (body.position - closedPose.position).sqrMagnitude;
                if (d < best)
                {
                    best = d;
                    nearest = body;
                }
            }
            return nearest;
        }

        // 열기 → 1개 통과 → 닫기. 대기 박스가 없거나 이미 방출 중이면 false
        public bool ReleaseOne()
        {
            if (releasing != null)
            {
                return false;
            }
            Rigidbody box = FindWaitingBox();
            if (box == null)
            {
                return false;
            }
            releasing = StartCoroutine(ReleaseRoutine(box));
            return true;
        }

        private IEnumerator ReleaseRoutine(Rigidbody box)
        {
            //1. 열기
            targetOpen = 1f;
            float elapsed = 0f;

            //2. 센서 이탈이 아니라 박스 뒷부분이 실제 스토퍼 면을 통과했는지 확인한다.
            // 옆으로 밀려 센서에서 빠지거나 다른 박스가 더 가까워져도 방출 완료가 아니다.
            while (box != null && (!IsOpen || !HasPassed(box)) && elapsed < releaseTimeout)
            {
                elapsed += Time.deltaTime;
                yield return null;
            }
            yield return new WaitForSeconds(closeDelay);

            //3. 닫기
            targetOpen = 0f;
            while (openAmount > 0.01f) yield return null;
            releasing = null;
            if (box != null && HasPassed(box) && elapsed < releaseTimeout)
            {
                Released?.Invoke(box);
            }
            else
            {
                Debug.LogWarning("[MiddleStopper] 방출한 박스가 제한 시간 안에 지나가지 않음", this);
            }
        }

        private bool HasPassed(Rigidbody box)
        {
            var shape = box.GetComponentInChildren<BoxCollider>();
            if (shape == null || closedPose == null) return false;
            Bounds b = shape.bounds;
            Vector3 direction = transform.forward;
            float half = Vector3.Dot(b.extents, new Vector3(Mathf.Abs(direction.x), Mathf.Abs(direction.y), Mathf.Abs(direction.z)));
            return Vector3.Dot(b.center - closedPose.position, direction) - half > 0.03f;
        }

        // 초기화 버튼: 스토퍼 닫힘
        public void ResetToInitialState()
        {
            if (releasing != null)
            {
                StopCoroutine(releasing);
                releasing = null;
            }
            targetOpen = 0f;
            openAmount = 0f;
        }
        #endregion
    }
}
