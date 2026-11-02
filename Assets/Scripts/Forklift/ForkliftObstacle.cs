using UnityEngine;

namespace Logistics
{
    // AMR 장애물 표시. 이 컴포넌트가 붙은 물체(자식 충돌체 포함)가 AMR 진행 방향 감지 상자에 들어오면 AMR이 정지한다. (FR-14)
    // 다른 AMR은 표시 없이도 장애물로 본다. 공정 설비·파레트·박스는 장애물로 보지 않는다.
    // 시험용 장애물(ShippingProcess/AMR_TestObstacle_Lane1)은 비활성으로 두고 Play 중 켜서 정지·재개를 확인한다.
    public class ForkliftObstacle : MonoBehaviour
    {
    }
}
