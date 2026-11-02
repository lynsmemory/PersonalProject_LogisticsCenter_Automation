using UnityEngine;

namespace Logistics
{
    // 박스 Prefab에 붙어 박스 타입, 바코드 라벨 표시, 바코드 판독 결과를 가진다. (FR-03, FR-06, 박스 타입 추가 대비)
    // 판독 결과는 BarcodeScanner만 쓴다. 투입 때 붙인 라벨 값을 판독 결과로 복사하지 않는다.
    // 기존 BoxRouting을 대체한다 (같은 스크립트 GUID 유지).
    [DisallowMultipleComponent]
    public class LogisticsBox : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("박스 종류")]
        [SerializeField] private BoxType boxType = BoxType.Standard;

        [Tooltip("바코드 라벨을 표시하는 박스 면 (BarcodeLabel)")]
        [SerializeField] private Renderer labelRenderer;
        #endregion

        #region 2. 상태
        private static readonly int BaseMapId = Shader.PropertyToID("_BaseMap");
        private static readonly int MainTexId = Shader.PropertyToID("_MainTex");
        private MaterialPropertyBlock labelBlock;

        public BoxType BoxType => boxType;
        public Renderer LabelRenderer => labelRenderer;

        // 판독한 목적 레인. HasReadResult가 false이면 의미 없음
        public DestinationLane ReadLane { get; private set; }
        public bool HasReadResult { get; private set; }

        public Rigidbody Body { get; private set; }

        // 흡착 툴이 들고 있음 (VacuumGripper가 설정). 픽업 정렬기·레인 감속 구역이 이 박스를 건드리지 않게 한다
        public bool IsHeld { get; private set; }
        #endregion

        #region 3. 라벨과 판독 결과
        private void Awake()
        {
            Body = GetComponent<Rigidbody>();
            // 레인 입구 완경사(약 2~4°)에서 느려진 박스가 잠들면 저마찰 면에서도 다시 움직이지 않는다 (10/30 시험에서 확인).
            // 공정 중인 박스는 잠들지 않게 한다
            if (Body != null)
            {
                Body.sleepThreshold = 0f;
            }
        }

        // 라벨 이미지를 박스 면에 표시한다 (재질 공유를 깨지 않도록 PropertyBlock 사용)
        public void ApplyLabel(Texture2D label)
        {
            if (labelRenderer == null || label == null)
            {
                return;
            }

            labelBlock ??= new MaterialPropertyBlock();
            labelRenderer.GetPropertyBlock(labelBlock);
            labelBlock.SetTexture(BaseMapId, label);
            labelBlock.SetTexture(MainTexId, label);
            labelRenderer.SetPropertyBlock(labelBlock);
        }

        // 판독 결과 기록 (BarcodeScanner 전용)
        public void SetReadResult(DestinationLane lane)
        {
            ReadLane = lane;
            HasReadResult = true;
        }

        public void ClearReadResult()
        {
            HasReadResult = false;
        }

        // 파레트에 놓인 박스: 공정이 끝났으므로 다시 잠들 수 있게 하고(기본 sleepThreshold) 쌓임 해석 반복을 높인다.
        // 잠들지 않는 박스를 여러 층 쌓으면 미세 떨림으로 조금씩 미끄러진다 (10/30 적재 시험: 로봇 접촉 없이 2단 박스 8~13 mm 밀림)
        public void SetPlacedOnPallet()
        {
            if (Body == null)
            {
                return;
            }
            Body.sleepThreshold = Physics.sleepThreshold;
            Body.solverIterations = Mathf.Max(Body.solverIterations, placedSolverIterations);
            Body.solverVelocityIterations = Mathf.Max(Body.solverVelocityIterations, placedSolverIterations);
        }

        private const int placedSolverIterations = 16;   // 파레트 위 박스 해석 반복 횟수

        // 흡착 상태 기록 (VacuumGripper 전용)
        public void SetHeld(bool held)
        {
            IsHeld = held;
        }
        #endregion
    }
}
