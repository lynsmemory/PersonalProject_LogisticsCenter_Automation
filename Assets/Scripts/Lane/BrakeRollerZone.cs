using UnityEngine;

namespace Logistics
{
    // 무동력 경사 레인 구간의 감속 롤러 (FR-08). 레인은 동력이 없어 박스는 중력으로 출발·이동하며,
    // 이 컴포넌트는 내리막 방향 속도가 maxSpeed를 넘는 만큼만 없앤다 (급경사 슈트의 브레이크 롤러와 같은 역할).
    // 구간 바닥 BoxCollider에 붙인다. 로컬 +Z가 내리막, 로컬 +Y가 바닥 법선이다.
    [RequireComponent(typeof(BoxCollider))]
    public class BrakeRollerZone : MonoBehaviour
    {
        [Tooltip("내리막 방향 최대 속도 (m/s)")]
        [SerializeField, Min(0.05f)] private float maxSpeed = 0.6f;

        private BoxCollider surface;                           // 구간 바닥
        private readonly Collider[] hits = new Collider[16];

        public float MaxSpeed => maxSpeed;

        private void Awake() => surface = GetComponent<BoxCollider>();

        // 매 물리 단계: 바닥 위 0.5 m 안의 동적 박스 중 내리막 속도가 상한을 넘는 박스를 감속한다
        private void FixedUpdate()
        {
            Vector3 size = surface.size;
            Vector3 centre = transform.TransformPoint(surface.center + new Vector3(0f, size.y * 0.5f + 0.25f, 0f));
            int n = Physics.OverlapBoxNonAlloc(centre, new Vector3(size.x * 0.5f, 0.25f, size.z * 0.5f), hits, transform.rotation, ~0, QueryTriggerInteraction.Ignore);
            Vector3 downhill = transform.forward;
            for (int i = 0; i < n; i++)
            {
                var body = hits[i].attachedRigidbody;
                if (body == null || body.isKinematic) continue;
                var box = body.GetComponent<LogisticsBox>();
                if (box != null && box.IsHeld) continue;   // 흡착 툴이 들고 있는 박스는 건드리지 않는다
                float along = Vector3.Dot(body.linearVelocity, downhill);
                if (along > maxSpeed) body.linearVelocity -= downhill * (along - maxSpeed);
            }
        }
    }
}
