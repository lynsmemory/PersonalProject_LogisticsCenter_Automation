namespace Logistics
{
    // 박스 종류. 새 박스는 값을 추가한다. (FR-03, 박스 타입 추가 대비)
    // Standard: 현재 적용 박스 180×140×170mm, 2kg (SRS·Design의 360×280×340mm에서 사용자 승인으로 변경됨)
    public enum BoxType
    {
        Standard
    }

    // 목적 레인. 바코드 L01·L02·L03과 대응한다. (FR-07)
    public enum DestinationLane
    {
        Lane1,
        Lane2,
        Lane3
    }

    // 기본(L=3, N=10)·시연(L=2, N=3) 설정 선택 (FR-23)
    public enum SettingsPreset
    {
        Default,
        Demonstration
    }

    // 전체 운전 상태 (FR-21)
    public enum SimulationRunState
    {
        Stopped,
        Running,
        Paused
    }

    // 카메라 뷰. CameraViewSwitcher의 카메라 배열 순서와 같다. (FR-02)
    public enum CameraView
    {
        Overview,
        Sorting,
        Palletizing,
        Dock
    }

    // FR5 설비 상태 (화면 표시) (FR-09, FR-18)
    public enum PalletizingState
    {
        Idle,
        Picking,
        Placing,
        Returning
    }

    // AMR 설비 상태 (화면 표시) (FR-13~15, FR-18)
    // 설계 열거형 밖 추가 값: WaitingForRoute = 경로가 없어 재탐색 대기, Faulted = 도달 불가·상차 재시도 초과·화물 수 불일치로 고장 정지 (이유는 FaultReason)
    public enum ForkliftState
    {
        Idle,
        MovingToPallet,
        Lifting,
        MovingToDock,
        Loading,
        Returning,
        StoppedByObstacle,
        WaitingForRoute,
        Faulted
    }

    // 흡착 결과 (FR-09). 붙지 못한 이유를 구분해 PalletizingSequence가 실패로 처리한다
    public enum VacuumAttachResult
    {
        Attached,         // 흡착 성공
        NoBox,            // 대상 박스 없음 또는 Rigidbody·BoxCollider 없음
        AlreadyHolding,   // 이미 다른 박스를 들고 있음
        TooFar,           // 흡착면과 박스 윗면 사이가 접촉 거리보다 멂
        OffCenter,        // 흡착면 중심이 박스 윗면 안쪽 허용 범위를 벗어남
        Tilted            // 툴 축과 박스 윗면 수직이 허용 각도보다 어긋남
    }

    // 박스 1개 적재 결과 (FR-09, FR-10). 성공일 때만 파레트 적재 수를 늘린다
    public enum PalletizingResult
    {
        Success,          // 지정 자리에 놓고 위치 확인까지 끝남
        NoBox,            // 픽업 구역에 정렬된 박스 없음
        PalletFull,       // 파레트가 6×L 완료라 놓을 자리 없음
        Busy,             // 이미 동작 중
        PickMoveFailed,   // 흡착점 접근·하강 이동 실패 (해 없음·도달 실패)
        AttachFailed,     // 흡착 조건 불충족 (VacuumAttachResult 참고)
        CarryMoveFailed,  // 들어 올림·운반·하강 이동 실패
        Dropped,          // 운반 중 흡착력 초과로 떨어짐
        PlaceCheckFailed  // 해제 후 박스가 지정 자리·수평 허용 범위를 벗어남
    }
}
