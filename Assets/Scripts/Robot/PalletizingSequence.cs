using System.Collections;
using UnityEngine;

namespace Logistics
{
    // 픽업 구역에 정렬된 박스 1개를 흡착해 파레트의 다음 지정 자리에 놓는 동작 순서를 진행한다. (FR-09, FR-10)
    // 순서: 흡착점 위 접근 → 직선 하강 → 흡착 → 정렬기 구역 비움 → 직선 상승(운반 높이) → 자리 위 이동 → 직선 하강 → 해제
    //       → 놓인 위치 확인 → (성공 시) 파레트 적재 수 +1 → 자리 위로 상승 → 홈 복귀.
    // 놓인 박스의 위치·수평을 확인한 뒤에만 PalletLoadState.ConfirmPlaced를 부른다. 실패하면 결과(LastResult)를 남기고 세지 않는다.
    // 운반 중 실패하면 박스를 든 채 멈춘다(실패 후 처리 규칙은 SRS상 시험 후 결정). 로봇 루트(FR5_LaneN)에 레인마다 1개 붙인다.
    // State가 Picking에서 Placing으로 바뀌는 시점 = 흡착한 박스를 운반 높이까지 들어 올려 픽업 구역을 벗어난 시점이다.
    // 레인(LaneProcessController)은 이 시점부터 다음 박스를 방출한다(적재·복귀와 병행).
    [DisallowMultipleComponent]
    public class PalletizingSequence : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("이 레인 FR5 팔")]
        [SerializeField] private RobotArmInverseKinematics arm;

        [Tooltip("흡착 툴")]
        [SerializeField] private VacuumGripper gripper;

        [Tooltip("흡착점·적재점 위 접근 높이 (m)")]
        [SerializeField, Min(0.02f)] private float approachHeight = 0.10f;

        [Tooltip("흡착 시 흡착면을 박스 윗면 위 이 높이까지 내린다 (m). 흡착 허용 간격(VacuumGripper.contactDistance) 안이어야 한다")]
        [SerializeField, Min(0f)] private float contactGap = 0.003f;

        [Tooltip("운반 중 박스 바닥과 쌓인 박스·픽업 박스 윗면 사이 여유 (m)")]
        [SerializeField, Min(0.02f)] private float transitClearance = 0.08f;

        [Tooltip("해제 높이: 박스 바닥이 지정 자리보다 이만큼 위에서 놓는다 (m)")]
        [SerializeField, Min(0f)] private float releaseGap = 0.005f;

        [Tooltip("해제 후 박스가 안정될 때까지 기다리는 시간 (초)")]
        [SerializeField, Min(0f)] private float settleSeconds = 0.5f;

        [Tooltip("놓인 박스 바닥 중심과 지정 자리의 허용 거리 (m)")]
        [SerializeField, Min(0.001f)] private float placeTolerance = 0.02f;

        [Tooltip("놓인 박스의 허용 기울기 (도)")]
        [SerializeField, Range(0f, 30f)] private float tiltToleranceDegrees = 5f;
        #endregion

        #region 2. 상태
        public PalletizingState State { get; private set; } = PalletizingState.Idle;   // 설비 상태 (화면 표시)
        public PalletizingResult LastResult { get; private set; } = PalletizingResult.Success;
        public float LastPlaceError { get; private set; }   // 마지막으로 놓은 박스 바닥 중심과 지정 자리의 거리 (m)
        public bool IsBusy => State != PalletizingState.Idle;
        public RobotArmInverseKinematics Arm => arm;
        public VacuumGripper Gripper => gripper;

        private int runVersion;   // 초기화 때 진행 중인 순서를 끊기 위한 번호
        #endregion

        #region 3. 적재 순서
        // 박스 1개 적재. 끝나면 LastResult로 결과를 확인한다 (Success일 때만 파레트 적재 수가 늘어난다)
        public IEnumerator PickAndPlace(PickupZoneAligner pickup, PalletLoadState pallet)
        {
            if (IsBusy)
            {
                LastResult = PalletizingResult.Busy;
                yield break;
            }
            LogisticsBox box = pickup != null ? pickup.CurrentBox : null;
            BoxCollider shape = box != null ? box.GetComponentInChildren<BoxCollider>() : null;
            if (box == null || shape == null)
            {
                LastResult = PalletizingResult.NoBox;
                yield break;
            }
            if (pallet == null || !pallet.GetNextSlotPose(out Pose slot))
            {
                LastResult = PalletizingResult.PalletFull;
                yield break;
            }
            int version = ++runVersion;

            //1. 흡착: 박스 윗면 중심 위 접근 → 직선 하강 → 흡착 → 정렬기 구역 비움
            State = PalletizingState.Picking;
            Bounds bounds = shape.bounds;
            float boxHeight = bounds.size.y;
            Vector3 top = new Vector3(bounds.center.x, bounds.max.y, bounds.center.z);
            float pickYaw = box.transform.eulerAngles.y;
            yield return arm.MoveTo(top + Vector3.up * approachHeight, pickYaw);
            if (Aborted(version) || !Check(arm.LastMoveSucceeded, PalletizingResult.PickMoveFailed)) yield break;
            yield return arm.MoveLinear(top + Vector3.up * contactGap, pickYaw);
            if (Aborted(version) || !Check(arm.LastMoveSucceeded, PalletizingResult.PickMoveFailed)) yield break;
            if (gripper.TryAttach(box) != VacuumAttachResult.Attached)
            {
                yield return arm.MoveLinear(top + Vector3.up * approachHeight, pickYaw);   // 박스는 정렬기에 그대로 두고 물러난다
                Finish(PalletizingResult.AttachFailed);
                yield break;
            }
            pickup.ClearZone();

            //2. 운반: 박스 바닥이 쌓인 박스(L층 전부)와 픽업 박스 윗면보다 여유만큼 높게 오도록 상승 → 자리 위로 이동
            float transitHeight = Mathf.Max(pallet.StackTopHeight, top.y) + boxHeight + transitClearance;
            yield return arm.MoveLinear(new Vector3(top.x, transitHeight, top.z), pickYaw);
            if (Aborted(version) || !CheckCarry()) yield break;

            //   운반 높이 도달: 박스 바닥이 픽업 박스 윗면보다 transitClearance 이상 위 → Placing (레인은 이때부터 다음 박스 방출)
            State = PalletizingState.Placing;
            float placeYaw = slot.rotation.eulerAngles.y;
            Vector3 release = slot.position + Vector3.up * (boxHeight + releaseGap);
            yield return arm.MoveTo(new Vector3(release.x, transitHeight, release.z), placeYaw);
            if (Aborted(version) || !CheckCarry()) yield break;

            //3. 적재: 직선 하강 → 해제 → 안정 대기 → 놓인 위치 확인
            yield return arm.MoveLinear(release, placeYaw);
            if (Aborted(version) || !CheckCarry()) yield break;
            gripper.Release();
            box.SetPlacedOnPallet();   // 파레트 위에서는 잠들 수 있게 해 쌓인 박스가 미끄러지지 않게 한다
            for (float t = 0f; t < settleSeconds; t += Time.fixedDeltaTime)
            {
                yield return new WaitForFixedUpdate();
            }
            if (Aborted(version)) yield break;
            Bounds placed = shape.bounds;
            Vector3 bottom = new Vector3(placed.center.x, placed.min.y, placed.center.z);
            LastPlaceError = Vector3.Distance(bottom, slot.position);
            bool upright = Vector3.Angle(box.transform.up, Vector3.up) <= tiltToleranceDegrees;
            PalletizingResult placeResult = LastPlaceError <= placeTolerance && upright ? PalletizingResult.Success : PalletizingResult.PlaceCheckFailed;
            if (placeResult == PalletizingResult.Success)
            {
                pallet.ConfirmPlaced();
            }

            //4. 복귀: 자리 위로 상승 → 홈
            State = PalletizingState.Returning;
            yield return arm.MoveLinear(new Vector3(release.x, transitHeight, release.z), placeYaw);
            if (Aborted(version)) yield break;
            yield return arm.MoveHome();
            if (Aborted(version)) yield break;
            Finish(placeResult);
        }

        // 이동 결과 확인. 실패면 결과를 남기고 Idle로 (든 박스는 그대로)
        private bool Check(bool succeeded, PalletizingResult failure)
        {
            if (!succeeded)
            {
                Finish(failure);
            }
            return succeeded;
        }

        // 운반 단계 확인: 떨어졌으면 Dropped, 이동 실패면 CarryMoveFailed
        private bool CheckCarry()
        {
            if (!gripper.IsHolding)
            {
                Finish(PalletizingResult.Dropped);
                return false;
            }
            return Check(arm.LastMoveSucceeded, PalletizingResult.CarryMoveFailed);
        }

        private bool Aborted(int version) => version != runVersion;

        private void Finish(PalletizingResult result)
        {
            LastResult = result;
            State = PalletizingState.Idle;
        }

        // 초기화 버튼: 진행 중인 순서를 끊고 팔을 홈에 즉시 배치, 흡착 해제
        public void ResetToInitialState()
        {
            runVersion++;
            if (arm != null) arm.ResetToInitialState();
            if (gripper != null) gripper.ResetToInitialState();
            State = PalletizingState.Idle;
            LastResult = PalletizingResult.Success;
            LastPlaceError = 0f;
        }
        #endregion
    }
}
