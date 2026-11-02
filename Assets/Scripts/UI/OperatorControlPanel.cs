using System.Text;
using UnityEngine;

namespace Logistics
{
    // 운전 조작·설정 선택·뷰 선택을 받아 해당 클래스 함수를 호출하고, ProcessStatusRecorder 값(처리 제품·시간당 생산량·대기 수·적재 단수·설비 상태·차량 상차 수)과
    // 설정 L·N, 정체 알람을 표시한다. 표시값은 기록(CSV)과 같은 함수로 만든다. (FR-02, FR-18, FR-19, FR-21, FR-23)
    // OnStart 등 공개 함수는 uGUI Button OnClick에 연결한다. UI 에셋을 연결하기 전에는 drawFallbackGui로 화면 왼쪽 위에 기본 패널을 그린다.
    // 기본 패널은 기준 해상도(referenceResolution) 좌표로 그리고 현재 화면 크기에 맞춰 같은 비율로 키우거나 줄인다.
    // 고정 픽셀로 그리면 화면 크기를 바꿀 때 패널이 차지하는 위치·비율이 달라진다 (10/30 사용자 보고).
    // 버튼은 운전 상태에 맞을 때만 누를 수 있다 (정지: 시작·설정 선택 / 운전: 일시정지 / 일시정지: 재개 / 초기화는 항상).
    public class OperatorControlPanel : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("운전 명령 대상")]
        [SerializeField] private LogisticsSimulationController simulation;

        [Tooltip("뷰 전환")]
        [SerializeField] private CameraViewSwitcher cameraSwitcher;

        [Tooltip("표시값 (처리 수·생산량·설비 상태 기록)")]
        [SerializeField] private ProcessStatusRecorder statusRecorder;

        [Tooltip("UI 에셋 연결 전 기본 화면 (IMGUI)")]
        [SerializeField] private bool drawFallbackGui = true;

        [Tooltip("기본 패널 기준 해상도 (픽셀). 화면이 이 크기일 때 패널이 아래 위치·크기로 그려진다")]
        [SerializeField] private Vector2 referenceResolution = new Vector2(1920f, 1080f);

        [Tooltip("기준 해상도에서 기본 패널의 위치·크기 (왼쪽 위 기준, 픽셀)")]
        [SerializeField] private Rect panelRect = new Rect(10f, 10f, 460f, 520f);
        #endregion

        #region 2. 상태
        private readonly StringBuilder text = new StringBuilder();
        public string DisplayText => text.ToString();
        #endregion

        #region 3. 조작
        // 운전 버튼
        public void OnStart() { if (simulation != null) simulation.StartSimulation(); }
        public void OnPause() { if (simulation != null) simulation.PauseSimulation(); }
        public void OnResume() { if (simulation != null) simulation.ResumeSimulation(); }
        public void OnReset() { if (simulation != null) simulation.ResetSimulation(); }

        // 생산 속도
        public void OnSpeedChanged(float multiplier) { if (simulation != null) simulation.SetSpeedMultiplier(multiplier); }

        // 설정 선택 (UI 버튼용 번호: 0 기본, 1 시연)
        public void OnPresetSelected(int preset) { OnPresetSelected((SettingsPreset)preset); }
        public void OnPresetSelected(SettingsPreset preset) { if (simulation != null) simulation.SelectPreset(preset); }

        // 뷰 선택 (UI 버튼용 번호: 0 전체, 1 분류, 2 FR5, 3 도크)
        public void OnViewSelected(int view) { if (cameraSwitcher != null) cameraSwitcher.SwitchToIndex(view); }
        #endregion

        #region 4. 표시
        private void Update()
        {
            RefreshDisplay();
        }

        // 표시값·정체 알람 갱신
        public void RefreshDisplay()
        {
            text.Clear();
            if (simulation == null)
            {
                return;
            }
            SimulationSettings settings = simulation.ActiveSettings;
            text.AppendLine($"운전 상태: {StateName(simulation.State)}   속도 x{simulation.SpeedMultiplier:F1}");
            if (settings != null)
            {
                text.AppendLine($"설정: {(simulation.ActivePreset == SettingsPreset.Default ? "기본" : "시연")}  L={settings.PalletLayerCount}  N={settings.TruckPalletCapacity}  (층당 {SimulationSettings.BoxesPerLayer}개)");
            }
            if (simulation.BoxSpawner != null)
            {
                text.AppendLine($"투입 {simulation.BoxSpawner.SpawnedCount}개{(simulation.BoxSpawner.IsFeedBlocked ? "  [합류 대기·투입선 정지]" : "")}");
            }
            if (simulation.BarcodeScanner != null)
            {
                text.AppendLine($"판독 {simulation.BarcodeScanner.ReadCount}개  실패 {simulation.BarcodeScanner.FailCount}개");
            }
            if (statusRecorder != null)
            {
                text.AppendLine($"처리 {statusRecorder.ProcessedBoxCount}개  시간당 {statusRecorder.BoxesPerHour:F1}개");
            }
            foreach (LaneProcessController lane in simulation.LaneProcesses)
            {
                if (lane == null || lane.OccupancyMonitor == null)
                {
                    continue;
                }
                LaneOccupancyMonitor m = lane.OccupancyMonitor;
                bool waiting = lane.MiddleStopper != null && lane.MiddleStopper.HasWaitingBox;
                bool pickup = lane.PickupZone != null && !lane.PickupZone.IsEmpty;
                text.AppendLine($"레인{(int)lane.Lane + 1}: 순환 {m.CirculatingCount}  스토퍼 {(waiting ? 1 : 0)}  픽업 {(pickup ? 1 : 0)}  처리 {lane.ProcessedCount}{(m.IsCongested ? "  [정체]" : "")}");
                int placed = lane.Pallet != null ? lane.Pallet.PlacedCount : 0;
                int capacity = lane.Pallet != null ? lane.Pallet.Capacity : 0;
                text.AppendLine($"   적재 {placed}/{capacity} ({ProcessStatusRecorder.LayerText(lane.Pallet)}단)  FR5 {ProcessStatusRecorder.PalletizingStateName(lane.Palletizing)}");
                string departures = lane.Dock != null ? $"  출발 {lane.Dock.DepartureCount}대" : "";
                text.AppendLine($"   AMR {ProcessStatusRecorder.ForkliftStateName(lane.Forklift)}  차량 {ProcessStatusRecorder.TruckText(lane.Dock)}{departures}");
                if (lane.Forklift != null && lane.Forklift.FaultReason != null) text.AppendLine($"   [AMR 정지] {lane.Forklift.FaultReason}");
                if (lane.FaultReason != null) text.AppendLine($"   [레인 대기] {lane.FaultReason}");
            }
        }

        private static string StateName(SimulationRunState state)
        {
            switch (state)
            {
                case SimulationRunState.Running: return "운전";
                case SimulationRunState.Paused: return "일시정지";
                default: return "정지";
            }
        }

        // 화면 크기 / 기준 해상도 비율. 가로·세로 중 작은 쪽을 써서 글자·버튼 비율을 유지한다
        public float PanelScale => Mathf.Min(Screen.width / Mathf.Max(1f, referenceResolution.x), Screen.height / Mathf.Max(1f, referenceResolution.y));

        // 현재 화면에서 기본 패널이 차지하는 영역 (픽셀, 왼쪽 위 기준)
        public Rect ScreenPanelRect => new Rect(panelRect.x * PanelScale, panelRect.y * PanelScale, panelRect.width * PanelScale, panelRect.height * PanelScale);

        private void OnGUI()
        {
            if (!drawFallbackGui || simulation == null)
            {
                return;
            }
            //1. 기준 해상도 좌표로 그리도록 화면 크기 비율만큼 확대·축소한다
            Matrix4x4 previousMatrix = GUI.matrix;
            float scale = PanelScale;
            GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1f));

            GUILayout.BeginArea(panelRect, GUI.skin.box);
            SimulationRunState state = simulation.State;

            GUILayout.BeginHorizontal();
            GUI.enabled = state == SimulationRunState.Stopped;
            if (GUILayout.Button("시작")) OnStart();
            GUI.enabled = state == SimulationRunState.Running;
            if (GUILayout.Button("일시정지")) OnPause();
            GUI.enabled = state == SimulationRunState.Paused;
            if (GUILayout.Button("재개")) OnResume();
            GUI.enabled = true;
            if (GUILayout.Button("초기화")) OnReset();
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUI.enabled = state == SimulationRunState.Stopped;
            if (GUILayout.Button("기본 설정")) OnPresetSelected(SettingsPreset.Default);
            if (GUILayout.Button("시연 설정")) OnPresetSelected(SettingsPreset.Demonstration);
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            SimulationSettings settings = simulation.ActiveSettings;
            if (settings != null)
            {
                Vector2 range = settings.SpeedMultiplierRange;
                float speed = GUILayout.HorizontalSlider(simulation.SpeedMultiplier, range.x, range.y);
                if (!Mathf.Approximately(speed, simulation.SpeedMultiplier)) OnSpeedChanged(speed);
            }

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("전체")) OnViewSelected(0);
            if (GUILayout.Button("분류")) OnViewSelected(1);
            if (GUILayout.Button("FR5")) OnViewSelected(2);
            if (GUILayout.Button("도크")) OnViewSelected(3);
            GUILayout.EndHorizontal();

            GUILayout.Label(text.ToString());
            GUILayout.EndArea();

            //2. 다른 IMGUI에 영향이 없도록 원래 행렬로 되돌린다
            GUI.matrix = previousMatrix;
        }
        #endregion
    }
}
