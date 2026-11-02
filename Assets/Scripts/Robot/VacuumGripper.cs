using System;
using UnityEngine;

namespace Logistics
{
    // FR5 흡착 툴. 흡착면이 박스 윗면에 실제로 닿아 있을 때만 박스를 붙이고, 적재점에서 해제한다. (FR-09)
    // 흡착 중 물리 상태: 박스(2 kg)는 동적 Rigidbody 그대로 두고 중력만 끈다. 붙인 순간의 흡착점 기준 상대 자세를 유지하도록
    //   매 물리 단계 속도·각속도를 맞춘다. 붙일 때 박스를 순간 이동시키지 않는다(현재 위치 그대로 상대 자세 저장). 부모 오브젝트는 바꾸지 않는다.
    //   10/30 배치 시험 기록: FixedJoint로 ArticulationBody에 연결하거나 박스·로봇 충돌체에 Physics.IgnoreCollision을 걸면 Unity가 비정상 종료됐다.
    //   kinematic 박스로 따라가게 하면 툴 컵과 겹칠 때 로봇을 무한한 힘으로 밀어 흔들려서 쓰지 않는다.
    // 흡착력: 툴 가속도로 박스를 붙잡는 데 필요한 힘 m·|a − g|가 holdForce를 넘으면 떨어뜨리고 Dropped로 알린다.
    // 픽업 정렬기(PickupZoneAligner)가 kinematic으로 고정해 둔 박스는 흡착할 때 고정을 풀어 동적으로 바꾼다(정렬기 고정은 픽업 대기용).
    // 해제 후 물리 상태: 동적 Rigidbody, 붙이기 전 저장한 중력 사용 여부 복구, 속도 0에서 다시 떨어진다.
    // 툴 루트(VacuumTool, 손목3 링크 자식)에 레인마다 1개 붙인다.
    [DisallowMultipleComponent]
    public class VacuumGripper : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("흡착면 중심 (VacuumTool/SuctionPoint). 이 점의 +Y가 흡착 방향(툴 축)이다")]
        [SerializeField] private Transform suctionPoint;

        [Tooltip("툴이 붙은 로봇 링크 (wrist3_link)")]
        [SerializeField] private ArticulationBody toolLink;

        [Tooltip("흡착면과 박스 윗면 사이 허용 간격 (m). 이보다 떨어져 있으면 붙지 않는다")]
        [SerializeField, Min(0f)] private float contactDistance = 0.01f;

        [Tooltip("흡착면 중심과 박스 윗면 중심의 허용 어긋남 (m)")]
        [SerializeField, Min(0f)] private float centerTolerance = 0.03f;

        [Tooltip("툴 축과 박스 윗면 수직의 허용 각도 (도)")]
        [SerializeField, Range(0f, 45f)] private float tiltToleranceDegrees = 10f;

        [Tooltip("흡착 유지 힘 (N). 박스를 붙잡는 데 필요한 힘이 이를 넘으면 떨어진다. 명목값: 2 kg 박스 무게 19.6 N의 약 5배, 실제 컵 사양 확인 후 조정")]
        [SerializeField, Min(1f)] private float holdForce = 100f;
        #endregion

        #region 2. 상태
        public LogisticsBox HeldBox { get; private set; }                 // 흡착 중인 박스 (없으면 null)
        public bool IsHolding => HeldBox != null;
        public VacuumAttachResult LastAttachResult { get; private set; }  // 마지막 흡착 시도 결과
        public Transform SuctionPoint => suctionPoint;

        public event Action<LogisticsBox> Attached;   // 흡착 성공
        public event Action<LogisticsBox> Released;   // 해제 명령으로 놓음
        public event Action<LogisticsBox> Dropped;    // 흡착력을 넘어 떨어짐

        private Vector3 heldLocalPosition;                         // 흡착점 기준 박스 위치 (붙인 순간 값)
        private Quaternion heldLocalRotation;                      // 흡착점 기준 박스 회전
        private bool savedUseGravity;                              // 붙이기 전 박스 중력 사용 여부 (해제 때 복구)
        private Vector3 previousSuctionPosition;                   // 흡착력 계산용 이전 위치·속도
        private Vector3 previousSuctionVelocity;
        private bool hasPreviousVelocity;
        public float LastRequiredForce { get; private set; }       // 마지막 물리 단계에서 박스를 붙잡는 데 필요한 힘 (N)
        public float PeakRequiredForce { get; private set; }       // 이번 흡착 동안 가장 컸던 필요 힘 (N)
        private Quaternion previousSuctionRotation;                // 다음 단계 자세 예측용 이전 회전
        private readonly Collider[] overlapBuffer = new Collider[16];
        #endregion

        #region 3. 흡착
        // 흡착면 바로 아래(반경 0.05 m)의 박스를 찾는다. 없으면 null
        public LogisticsBox FindBoxUnderSuction()
        {
            if (suctionPoint == null)
            {
                return null;
            }
            int count = gameObject.scene.GetPhysicsScene().OverlapSphere(suctionPoint.position, 0.05f, overlapBuffer, ~0, QueryTriggerInteraction.Ignore);
            LogisticsBox nearest = null;
            float best = float.MaxValue;
            for (int i = 0; i < count; i++)
            {
                LogisticsBox box = overlapBuffer[i].GetComponentInParent<LogisticsBox>();
                if (box == null)
                {
                    continue;
                }
                float distance = Vector3.Distance(box.transform.position, suctionPoint.position);
                if (distance < best)
                {
                    best = distance;
                    nearest = box;
                }
            }
            return nearest;
        }

        // 박스 윗면이 흡착면에 닿아 있으면 붙인다. 결과는 LastAttachResult에도 남는다.
        public VacuumAttachResult TryAttach(LogisticsBox box)
        {
            LastAttachResult = CheckContact(box);
            if (LastAttachResult != VacuumAttachResult.Attached)
            {
                return LastAttachResult;
            }

            //1. 현재 박스 자세를 흡착점 기준으로 저장하고, 중력 사용 여부를 저장한 뒤 끈다.
            //   정렬기의 kinematic 고정은 풀어서 동적 Rigidbody로 따라오게 한다 (위치는 그대로)
            Rigidbody body = box.Body;
            body.isKinematic = false;
            heldLocalPosition = Quaternion.Inverse(suctionPoint.rotation) * (body.position - suctionPoint.position);
            heldLocalRotation = Quaternion.Inverse(suctionPoint.rotation) * body.rotation;
            savedUseGravity = body.useGravity;
            body.useGravity = false;
            previousSuctionPosition = suctionPoint.position;
            previousSuctionRotation = suctionPoint.rotation;
            hasPreviousVelocity = false;
            PeakRequiredForce = 0f;

            HeldBox = box;
            box.SetHeld(true);
            Attached?.Invoke(box);
            return LastAttachResult;
        }

        // 흡착 조건 검사: 박스·충돌체 있음, 들고 있지 않음, 윗면이 툴 축에 수직, 간격·중심 허용 범위
        private VacuumAttachResult CheckContact(LogisticsBox box)
        {
            if (IsHolding)
            {
                return VacuumAttachResult.AlreadyHolding;
            }
            BoxCollider shape = box != null ? box.GetComponentInChildren<BoxCollider>() : null;
            if (box == null || box.Body == null || shape == null || suctionPoint == null || toolLink == null)
            {
                return VacuumAttachResult.NoBox;
            }

            //1. 박스 축 중 툴 축 반대(흡착면을 마주 보는 방향)에 가장 가까운 면을 윗면으로 본다
            Vector3 facing = -suctionPoint.up;
            Vector3 half = Vector3.Scale(shape.size, shape.transform.lossyScale) * 0.5f;
            Vector3[] axes = { shape.transform.right, shape.transform.up, shape.transform.forward };
            int top = 0;
            float bestDot = -1f;
            for (int i = 0; i < 3; i++)
            {
                float dot = Mathf.Abs(Vector3.Dot(axes[i], facing));
                if (dot > bestDot)
                {
                    bestDot = dot;
                    top = i;
                }
            }
            Vector3 normal = Vector3.Dot(axes[top], facing) > 0f ? axes[top] : -axes[top];
            if (Vector3.Angle(normal, facing) > tiltToleranceDegrees)
            {
                return VacuumAttachResult.Tilted;
            }

            //2. 윗면 중심에서 흡착면 중심까지: 수직 간격과 면 방향 어긋남
            Vector3 faceCenter = shape.transform.TransformPoint(shape.center) + normal * half[top];
            Vector3 offset = suctionPoint.position - faceCenter;
            float gap = Vector3.Dot(offset, normal);
            if (gap > contactDistance || gap < -contactDistance)
            {
                return VacuumAttachResult.TooFar;
            }
            Vector3 lateral = offset - normal * gap;
            if (lateral.magnitude > centerTolerance)
            {
                return VacuumAttachResult.OffCenter;
            }
            return VacuumAttachResult.Attached;
        }
        #endregion

        #region 4. 해제와 떨어짐
        // 흡착을 끊고 박스를 놓는다. 들고 있지 않으면 false
        public bool Release()
        {
            LogisticsBox box = HeldBox;
            if (box == null)
            {
                return false;
            }
            Detach();
            Released?.Invoke(box);
            return true;
        }

        // 매 물리 단계: 필요한 흡착력을 계산해 넘으면 떨어뜨리고, 아니면 박스를 흡착점 상대 자세로 옮긴다
        private void FixedUpdate()
        {
            if (HeldBox == null)
            {
                return;
            }

            //1. 흡착점 가속도로 박스를 붙잡는 데 필요한 힘 m·|a − g|
            float dt = Time.fixedDeltaTime;
            Vector3 velocity = (suctionPoint.position - previousSuctionPosition) / dt;
            Vector3 acceleration = hasPreviousVelocity ? (velocity - previousSuctionVelocity) / dt : Vector3.zero;
            previousSuctionPosition = suctionPoint.position;
            previousSuctionVelocity = velocity;
            hasPreviousVelocity = true;
            Quaternion spin = suctionPoint.rotation * Quaternion.Inverse(previousSuctionRotation);
            previousSuctionRotation = suctionPoint.rotation;
            LastRequiredForce = HeldBox.Body.mass * (acceleration - Physics.gravity).magnitude;
            PeakRequiredForce = Mathf.Max(PeakRequiredForce, LastRequiredForce);
            if (LastRequiredForce > holdForce)
            {
                LogisticsBox box = HeldBox;
                Detach();
                Dropped?.Invoke(box);
                return;
            }

            //2. 흡착점이 다음 물리 단계에 있을 자세(현재 속도·회전 속도로 예측)에 박스가 닿도록 속도·각속도를 맞춘다
            Rigidbody body = HeldBox.Body;
            Quaternion nextRotation = spin * suctionPoint.rotation;
            Vector3 targetPosition = suctionPoint.position + velocity * dt + nextRotation * heldLocalPosition;
            Quaternion targetRotation = nextRotation * heldLocalRotation;
            body.linearVelocity = (targetPosition - body.position) / dt;
            (targetRotation * Quaternion.Inverse(body.rotation)).ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f)
            {
                angle -= 360f;
            }
            body.angularVelocity = float.IsInfinity(axis.x) || Mathf.Abs(angle) < 1e-4f ? Vector3.zero : axis * (angle * Mathf.Deg2Rad / dt);
        }

        // 흡착 해제: 저장한 물리 상태 복구, 속도 0
        private void Detach()
        {
            Rigidbody body = HeldBox != null ? HeldBox.Body : null;
            if (body != null)
            {
                body.isKinematic = false;
                body.useGravity = savedUseGravity;
                body.linearVelocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
            if (HeldBox != null)
            {
                HeldBox.SetHeld(false);
            }
            HeldBox = null;
        }

        // 초기화 버튼: 흡착 해제 (박스 제거는 BoxSpawner가 한다)
        public void ResetToInitialState()
        {
            if (HeldBox != null)
            {
                Detach();
            }
            LastAttachResult = VacuumAttachResult.NoBox;
        }
        #endregion
    }
}
