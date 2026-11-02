using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Logistics
{
    // 4개 카메라 중 선택한 것만 켠다. (FR-02, UI-04, TC-02)
    // 씬에 1개. ViewCameras 루트 등 항상 켜져 있는 오브젝트에 붙인다.
    // 뷰 카메라 4대는 씬에서 비활성으로 대기하고, 전환하면 선택한 카메라 오브젝트만 켠다.
    // 뷰가 켜져 있는 동안 기존 Main Camera는 Camera 컴포넌트만 끈다 (AudioListener는 Main Camera에 그대로 둔다).
    [DisallowMultipleComponent]
    public class CameraViewSwitcher : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("CameraView 순서의 카메라 (Overview, Sorting, Palletizing(FR5), Dock)")]
        [SerializeField] private Camera[] viewCameras = new Camera[4];

        [Tooltip("뷰를 켜는 동안 끌 기존 카메라 (Main Camera). 비워 두면 건드리지 않는다")]
        [SerializeField] private Camera mainCamera;

        [Tooltip("시작할 때 이 뷰로 전환한다. 끄면 시작 때는 Main Camera 화면을 유지한다")]
        [SerializeField] private bool switchOnStart = true;

        [Tooltip("시작·초기화 때 보여 줄 뷰")]
        [SerializeField] private CameraView initialView = CameraView.Overview;

        [Tooltip("숫자키 1~4로 뷰 전환 (운전 화면 UI 연결 전 TC-02 시험용)")]
        [SerializeField] private bool enableNumberKeys = true;
        #endregion

        #region 2. 상태
        // 지금 뷰. 아직 전환하지 않았으면 HasActiveView가 false다.
        public CameraView CurrentView { get; private set; }
        public bool HasActiveView { get; private set; }

        // 뷰 전환 알림 (운전 화면 표시용)
        public event Action<CameraView> ViewChanged;
        #endregion

        #region 3. 시작과 입력
        private void Start()
        {
            if (switchOnStart)
            {
                SwitchTo(initialView);
            }
        }

        private void Update()
        {
            if (!enableNumberKeys)
            {
                return;
            }

            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
            {
                return;
            }

            if (keyboard.digit1Key.wasPressedThisFrame) SwitchTo(CameraView.Overview);
            else if (keyboard.digit2Key.wasPressedThisFrame) SwitchTo(CameraView.Sorting);
            else if (keyboard.digit3Key.wasPressedThisFrame) SwitchTo(CameraView.Palletizing);
            else if (keyboard.digit4Key.wasPressedThisFrame) SwitchTo(CameraView.Dock);
        }
        #endregion

        #region 4. 뷰 전환
        // 뷰 전환: 선택한 카메라 오브젝트만 켜고 나머지 뷰와 Main Camera는 끈다.
        public void SwitchTo(CameraView view)
        {
            int index = (int)view;
            if (index < 0 || index >= viewCameras.Length || viewCameras[index] == null)
            {
                Debug.LogWarning($"[CameraViewSwitcher] {view} 카메라가 연결되지 않아 전환하지 않음", this);
                return;
            }

            //1. 선택한 뷰만 켠다 (Camera 컴포넌트도 켜진 상태로 맞춘다)
            for (int i = 0; i < viewCameras.Length; i++)
            {
                Camera viewCamera = viewCameras[i];
                if (viewCamera == null)
                {
                    continue;
                }

                bool selected = i == index;
                viewCamera.enabled = selected;
                viewCamera.gameObject.SetActive(selected);
            }

            //2. 같은 깊이의 Main Camera가 화면을 덮지 않게 끈다
            if (mainCamera != null)
            {
                mainCamera.enabled = false;
            }

            CurrentView = view;
            HasActiveView = true;
            ViewChanged?.Invoke(view);
        }

        // UI Button OnClick용: CameraView 순서의 번호 (0 전체, 1 분류, 2 FR5, 3 도크)
        public void SwitchToIndex(int viewIndex)
        {
            if (!Enum.IsDefined(typeof(CameraView), viewIndex))
            {
                Debug.LogWarning($"[CameraViewSwitcher] 없는 뷰 번호 {viewIndex}", this);
                return;
            }

            SwitchTo((CameraView)viewIndex);
        }

        // 초기화 버튼: 전체 뷰로
        public void ResetToInitialState()
        {
            SwitchTo(initialView);
        }
        #endregion
    }
}
