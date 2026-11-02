using System.Collections.Generic;
using RoboLacyTech.Conveyors;
using UnityEngine;

namespace Logistics
{
    // 순환 컨베이어를 운전 중 상시 구동한다. 레인 정체로는 멈추지 않는다. (FR-05, FR-21)
    // loopRoot(MainLoop)와 extraRoots(EntryLine 등) 아래의 LinearConveyor·CurvedConveyor와 분기 장치를 모아
    // 시작·정지·속도 배율을 한꺼번에 적용한다. 각 컨베이어의 원래 속도·방향은 처음 수집할 때 저장한다.
    public class MainConveyorController : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("MainLoop 루트 (컨베이어 수집)")]
        [SerializeField] private Transform loopRoot;

        [Tooltip("같이 구동할 다른 컨베이어 루트 (투입 라인 EntryLine)")]
        [SerializeField] private Transform[] extraRoots = new Transform[0];

        [Tooltip("신규 투입 전용 루트. 이 루트만 재순환 대기 중 정지하며 메인 합류 구동면은 포함하지 않는다")]
        [SerializeField] private Transform inputRoot;

        [Tooltip("기준 속도 배율 1에서의 벨트 속도 (m/s). 0이면 각 컨베이어에 저장된 속도를 그대로 쓴다")]
        [SerializeField, Min(0f)] private float baseSpeed = 0f;
        #endregion

        #region 2. 상태
        private struct Drive
        {
            public LinearConveyor linear;
            public CurvedConveyor curved;
            public float signedSpeed;
            public bool input;
        }

        private readonly List<Drive> drives = new List<Drive>();
        private readonly HashSet<Behaviour> restingInputDrives = new HashSet<Behaviour>();
        private LaneDiverter[] diverters = new LaneDiverter[0];

        public bool IsRunning { get; private set; }
        public float SpeedMultiplier { get; private set; } = 1f;
        public int ConveyorCount => drives.Count;
        public bool IsInputBlocked { get; private set; }
        #endregion

        #region 3. 수집
        private void Awake()
        {
            Collect();
        }

        // 운전 시작 전에는 멈춰 둔다. 패키지 기본값은 시작하자마자 구동하고, 그 초기화가 각 컨베이어 Awake에서 일어나므로 Start에서 끈다.
        // 이미 운전 명령을 받았으면(같은 프레임 Start 순서) 건드리지 않는다.
        private void Start()
        {
            if (!IsRunning)
            {
                Stop();
            }
        }

        private void FixedUpdate()
        {
            if (!IsInputBlocked) return;
            // 감속이 끝나면 정지 모터의 0거리 MovePosition 호출을 쉬게 한다.
            // 충돌체/Rigidbody는 유지해 정지 벨트 위 박스가 자연스럽게 쉴 수 있게 한다.
            foreach (Drive d in drives)
            {
                if (!d.input) continue;
                if (d.linear != null && d.linear.enabled && Mathf.Abs(d.linear.CurrentSpeed) < 0.001f)
                {
                    restingInputDrives.Add(d.linear);
                    d.linear.enabled = false;
                }
                if (d.curved != null && d.curved.enabled && Mathf.Abs(d.curved.CurrentSpeed) < 0.001f)
                {
                    restingInputDrives.Add(d.curved);
                    d.curved.enabled = false;
                }
            }
        }

        private void Collect()
        {
            drives.Clear();
            var roots = new List<Transform> { loopRoot };
            roots.AddRange(extraRoots);
            var diverterList = new List<LaneDiverter>();
            foreach (Transform root in roots)
            {
                if (root == null)
                {
                    continue;
                }
                foreach (LinearConveyor c in root.GetComponentsInChildren<LinearConveyor>(true))
                {
                    drives.Add(new Drive { linear = c, signedSpeed = Signed(c.Motor), input = inputRoot != null && c.transform.IsChildOf(inputRoot) });
                }
                foreach (CurvedConveyor c in root.GetComponentsInChildren<CurvedConveyor>(true))
                {
                    drives.Add(new Drive { curved = c, signedSpeed = Signed(c.Motor), input = inputRoot != null && c.transform.IsChildOf(inputRoot) });
                }
            }

            // 분기 장치는 레인 그룹(LaneN_Gate) 아래에 있으므로 루트의 부모(ConveyorSystem) 기준으로 찾는다
            Transform system = loopRoot != null ? loopRoot.parent : null;
            if (system != null)
            {
                diverterList.AddRange(system.GetComponentsInChildren<LaneDiverter>(true));
            }
            diverters = diverterList.ToArray();
        }

        private float Signed(ConveyorMotor motor)
        {
            float speed = baseSpeed > 0f ? baseSpeed : motor.BaseSpeed;
            return motor.Direction == ConveyorDirection.Backward ? -speed : speed;
        }
        #endregion

        #region 4. 운전
        // 구동 (일시정지 후 재개 포함)
        public void Run()
        {
            ApplySpeed();
            foreach (Drive d in drives)
            {
                bool run = !d.input || !IsInputBlocked;
                if (d.linear != null) { if (run) d.linear.Play(); else d.linear.Stop(); }
                if (d.curved != null) { if (run) d.curved.Play(); else d.curved.Stop(); }
            }
            foreach (LaneDiverter diverter in diverters)
            {
                diverter.Play();
            }
            IsRunning = true;
        }

        // 정지 (일시정지·초기화 전용)
        public void Stop()
        {
            foreach (Drive d in drives)
            {
                if (d.linear != null) d.linear.Stop();
                if (d.curved != null) d.curved.Stop();
            }
            foreach (LaneDiverter diverter in diverters)
            {
                diverter.Stop();
            }
            IsRunning = false;
        }

        // 메인은 계속 운전하고 투입선 모터만 감속 정지/재개한다.
        public void SetInputBlocked(bool blocked)
        {
            IsInputBlocked = blocked;
            if (!blocked)
            {
                foreach (var drive in restingInputDrives) if (drive != null) drive.enabled = true;
                restingInputDrives.Clear();
            }
            foreach (Drive d in drives)
            {
                if (!d.input) continue;
                bool run = IsRunning && !blocked;
                if (d.linear != null) { if (run) d.linear.Play(); else d.linear.Stop(); }
                if (d.curved != null) { if (run) d.curved.Play(); else d.curved.Stop(); }
            }
        }

        // 속도 배율 적용
        public void SetSpeedMultiplier(float multiplier)
        {
            SpeedMultiplier = Mathf.Max(0.01f, multiplier);
            ApplySpeed();
        }

        private void ApplySpeed()
        {
            foreach (Drive d in drives)
            {
                float speed = d.signedSpeed * SpeedMultiplier;
                if (d.linear != null) d.linear.SetSpeed(speed);
                if (d.curved != null) d.curved.SetSpeed(speed);
            }
            foreach (LaneDiverter diverter in diverters)
            {
                diverter.SpeedMultiplier = SpeedMultiplier;
            }
        }

        // 초기화 버튼: 정지, 속도 배율 1
        public void ResetToInitialState()
        {
            Stop();
            SetInputBlocked(false);
            foreach (var d in diverters) d.ResetToInitialState();
            SetSpeedMultiplier(1f);
        }
        #endregion
    }
}
