using System;
using UnityEngine;

namespace Logistics
{
    // 전체 운전 상태를 관리하고 공통 설비와 레인 3개에 시작·일시정지·재개·초기화·속도를 전달한다. 씬에 1개. (FR-21, FR-23)
    // 일시정지는 Time.timeScale = 0으로 물리·코루틴을 함께 멈춘다. 무동력 경사 레인의 박스도 같이 멈춰야 하기 때문이다.
    // 생산 속도 배율은 컨베이어 속도와 투입 간격에 적용한다 (범위는 설정값, 측정 후 확정).
    public class LogisticsSimulationController : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("기본 설정 (L=3, N=10)")]
        [SerializeField] private SimulationSettings defaultSettings;

        [Tooltip("시연 설정 (L=2, N=3)")]
        [SerializeField] private SimulationSettings demonstrationSettings;

        [Tooltip("박스 투입")]
        [SerializeField] private BoxSpawner boxSpawner;

        [Tooltip("메인 컨베이어")]
        [SerializeField] private MainConveyorController mainConveyor;

        [Tooltip("바코드 판독")]
        [SerializeField] private BarcodeScanner barcodeScanner;

        [Tooltip("레인 1~3")]
        [SerializeField] private LaneProcessController[] laneProcesses = new LaneProcessController[3];

        [Tooltip("카메라 뷰 (초기화 때 전체 뷰로)")]
        [SerializeField] private CameraViewSwitcher cameraSwitcher;

        [Tooltip("Play 시작과 함께 운전 시작 (시연·시험용). 끄면 조작 화면의 시작 버튼을 기다린다")]
        [SerializeField] private bool startOnPlay = false;
        #endregion

        #region 2. 상태
        public SimulationRunState State { get; private set; } = SimulationRunState.Stopped;
        public SettingsPreset ActivePreset { get; private set; } = SettingsPreset.Default;
        public float SpeedMultiplier { get; private set; } = 1f;
        public SimulationSettings ActiveSettings => ActivePreset == SettingsPreset.Demonstration ? demonstrationSettings : defaultSettings;
        public LaneProcessController[] LaneProcesses => laneProcesses;
        public BoxSpawner BoxSpawner => boxSpawner;
        public BarcodeScanner BarcodeScanner => barcodeScanner;

        public event Action<SimulationRunState> StateChanged;
        public event Action<SettingsPreset> PresetChanged;
        #endregion

        #region 3. 운전 명령
        private void Start()
        {
            if (startOnPlay)
            {
                StartSimulation();
            }
        }

        // 운전 시작 (정지 상태에서만)
        public void StartSimulation()
        {
            if (State != SimulationRunState.Stopped)
            {
                return;
            }
            SimulationSettings settings = ActiveSettings;
            if (settings == null)
            {
                Debug.LogWarning("[LogisticsSimulationController] 적용할 설정 에셋이 없음", this);
                return;
            }

            Time.timeScale = 1f;
            if (mainConveyor != null)
            {
                mainConveyor.SetSpeedMultiplier(SpeedMultiplier);
                mainConveyor.Run();
            }
            foreach (LaneProcessController lane in laneProcesses)
            {
                if (lane != null) lane.Begin(settings);
            }
            if (boxSpawner != null)
            {
                boxSpawner.Begin(settings.SpawnIntervalSeconds / SpeedMultiplier);
            }
            SetState(SimulationRunState.Running);
        }

        // 일시정지
        public void PauseSimulation()
        {
            if (State != SimulationRunState.Running)
            {
                return;
            }
            Time.timeScale = 0f;
            SetState(SimulationRunState.Paused);
        }

        // 재개
        public void ResumeSimulation()
        {
            if (State != SimulationRunState.Paused)
            {
                return;
            }
            Time.timeScale = 1f;
            SetState(SimulationRunState.Running);
        }

        // 초기화 버튼: 운전 정지 후 모든 설비의 ResetToInitialState() 호출
        public void ResetSimulation()
        {
            Time.timeScale = 1f;
            if (boxSpawner != null) boxSpawner.ResetToInitialState();
            if (mainConveyor != null) mainConveyor.ResetToInitialState();
            if (barcodeScanner != null) barcodeScanner.ResetToInitialState();
            foreach (LaneProcessController lane in laneProcesses)
            {
                if (lane != null) lane.ResetToInitialState();
            }
            if (cameraSwitcher != null) cameraSwitcher.ResetToInitialState();
            SpeedMultiplier = 1f;
            SetState(SimulationRunState.Stopped);
        }

        // 생산 속도 조절 (설정의 범위 안)
        public void SetSpeedMultiplier(float multiplier)
        {
            SimulationSettings settings = ActiveSettings;
            Vector2 range = settings != null ? settings.SpeedMultiplierRange : new Vector2(0.5f, 2f);
            SpeedMultiplier = Mathf.Clamp(multiplier, range.x, range.y);
            if (mainConveyor != null) mainConveyor.SetSpeedMultiplier(SpeedMultiplier);
            if (boxSpawner != null && settings != null) boxSpawner.SetInterval(settings.SpawnIntervalSeconds / SpeedMultiplier);
        }

        // 기본/시연 설정 선택 (정지 상태에서만 바꾼다: 진행 중 파레트의 완료 기준이 바뀌지 않게)
        public bool SelectPreset(SettingsPreset preset)
        {
            if (State != SimulationRunState.Stopped)
            {
                return false;
            }
            ActivePreset = preset;
            PresetChanged?.Invoke(preset);
            return true;
        }

        private void SetState(SimulationRunState state)
        {
            State = state;
            StateChanged?.Invoke(state);
        }

        private void OnDestroy()
        {
            Time.timeScale = 1f;
        }
        #endregion
    }
}
