using System;
using System.Collections.Generic;
using UnityEngine;
using ZXing;
using ZXing.Common;

namespace Logistics
{
    // 판독점에 박스가 들어오면 판독 카메라 화면을 읽어 ZXing.Net으로 Code 128을 해독하고 결과를 LogisticsBox에 기록한다. (FR-06, NFR-03)
    // 판독점(BarcodeReadPoint)에 붙인다. readTrigger는 자식 ScanSensor의 트리거여서 이 오브젝트로 OnTriggerEnter가 오지 않으므로
    // 매 프레임 그 트리거 영역을 직접 조회한다. 박스 중심이 카메라 바로 아래 촬영 구간에 들어오면 촬영해 판독하고,
    // 실패하면 구간 안에서 다시 시도한다. 이미 판독 결과가 있는 순환 박스는 다시 읽지 않는다.
    public class BarcodeScanner : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("판독 카메라 (평소 비활성, 수동 촬영)")]
        [SerializeField] private Camera readCamera;

        [Tooltip("판독점 감지 트리거 (ScanSensor)")]
        [SerializeField] private Collider readTrigger;

        [Tooltip("판독 카메라 출력 (512×256)")]
        [SerializeField] private RenderTexture captureTexture;

        [Tooltip("벨트 흐름 방향으로 박스 중심이 카메라 중심선에서 이 거리 이내일 때 촬영 (m)")]
        [SerializeField, Min(0.005f)] private float captureWindow = 0.05f;

        [Tooltip("박스 하나당 최대 촬영 횟수")]
        [SerializeField, Min(1)] private int maxAttemptsPerBox = 4;
        #endregion

        #region 2. 상태
        private BarcodeReader decoder;
        private Texture2D readBuffer;
        private readonly Collider[] hits = new Collider[16];
        private readonly Dictionary<LogisticsBox, int> attempts = new Dictionary<LogisticsBox, int>();
        private readonly HashSet<LogisticsBox> inside = new HashSet<LogisticsBox>();
        private readonly List<LogisticsBox> leaving = new List<LogisticsBox>();

        // 판독 완료 알림 (레인 집계용)
        public event Action<LogisticsBox> BoxRead;
        // 촬영 구간을 지날 때까지 판독하지 못함 (미판독 처리는 MVP 제외, 기록용)
        public event Action<LogisticsBox> ReadFailed;

        public int ReadCount { get; private set; }
        public int FailCount { get; private set; }
        // 마지막 판독 문자열과 소요 시간 (NFR-03 측정용)
        public string LastText { get; private set; }
        public float LastDecodeMilliseconds { get; private set; }
        #endregion

        #region 3. 시작
        private void Awake()
        {
            decoder = new BarcodeReader
            {
                AutoRotate = true,
                Options = new DecodingOptions { PossibleFormats = new[] { BarcodeFormat.CODE_128 }, TryHarder = true }
            };
            if (readCamera != null)
            {
                readCamera.enabled = false;
            }
        }
        #endregion

        #region 4. 감지와 판독
        private void Update()
        {
            if (readTrigger == null || readCamera == null)
            {
                return;
            }

            //1. 트리거 영역 안의 박스를 찾는다
            Bounds b = readTrigger.bounds;
            int n = Physics.OverlapBoxNonAlloc(b.center, b.extents, hits, Quaternion.identity, ~0, QueryTriggerInteraction.Ignore);
            leaving.Clear();
            leaving.AddRange(inside);
            for (int i = 0; i < n; i++)
            {
                LogisticsBox box = hits[i].attachedRigidbody != null ? hits[i].attachedRigidbody.GetComponent<LogisticsBox>() : null;
                if (box == null)
                {
                    continue;
                }
                leaving.Remove(box);
                inside.Add(box);
                if (!box.HasReadResult)
                {
                    TryRead(box);
                }
            }

            //2. 영역을 벗어난 박스: 끝내 못 읽었으면 실패로 기록
            foreach (LogisticsBox box in leaving)
            {
                inside.Remove(box);
                if (box != null && !box.HasReadResult && attempts.ContainsKey(box))
                {
                    FailCount++;
                    Debug.LogWarning($"[BarcodeScanner] 판독 실패 {box.name}", box);
                    ReadFailed?.Invoke(box);
                }
                if (box != null)
                {
                    attempts.Remove(box);
                }
            }
        }

        private void TryRead(LogisticsBox box)
        {
            //1. 박스 중심이 흐름 방향으로 카메라 중심선 근처에 있고, 화면 가로 안쪽에 들어왔을 때만
            //   (카메라는 아래를 보며 화면 위쪽이 벨트 흐름 방향. 합류 후 박스가 벨트 한쪽으로 치우쳐 가므로 가로 위치는 화면 안이면 된다)
            Vector3 offset = box.transform.position - readCamera.transform.position;
            float along = Vector3.Dot(offset, readCamera.transform.up);
            Vector3 viewport = readCamera.WorldToViewportPoint(box.transform.position);
            if (Mathf.Abs(along) > captureWindow || viewport.x < 0.15f || viewport.x > 0.85f)
            {
                return;
            }
            attempts.TryGetValue(box, out int count);
            if (count >= maxAttemptsPerBox)
            {
                return;
            }
            attempts[box] = count + 1;

            //2. 촬영 → 해독 → 레인 변환
            float start = Time.realtimeSinceStartup;
            Color32[] pixels = CapturePixels();
            bool ok = TryDecode(pixels, out string text);
            LastDecodeMilliseconds = (Time.realtimeSinceStartup - start) * 1000f;
            if (ok && TryParseLane(text, out DestinationLane lane))
            {
                LastText = text;
                ReadCount++;
                box.SetReadResult(lane);
                attempts.Remove(box);
                BoxRead?.Invoke(box);
            }
        }

        // 판독 카메라 화면을 픽셀로 읽기
        public Color32[] CapturePixels()
        {
            int w = captureTexture != null ? captureTexture.width : 512;
            int h = captureTexture != null ? captureTexture.height : 256;
            RenderTexture target = captureTexture;
            bool temporary = target == null;
            if (temporary)
            {
                target = RenderTexture.GetTemporary(w, h, 24);
            }

            RenderTexture previousTarget = readCamera.targetTexture;
            RenderTexture previousActive = RenderTexture.active;
            readCamera.targetTexture = target;
            readCamera.Render();
            RenderTexture.active = target;
            if (readBuffer == null || readBuffer.width != w || readBuffer.height != h)
            {
                readBuffer = new Texture2D(w, h, TextureFormat.RGBA32, false);
            }
            readBuffer.ReadPixels(new Rect(0, 0, w, h), 0, 0, false);
            readBuffer.Apply(false);
            RenderTexture.active = previousActive;
            readCamera.targetTexture = previousTarget;
            if (temporary)
            {
                RenderTexture.ReleaseTemporary(target);
            }
            return readBuffer.GetPixels32();
        }

        // ZXing으로 해독
        public bool TryDecode(Color32[] pixels, out string text)
        {
            text = null;
            if (pixels == null || readBuffer == null)
            {
                return false;
            }
            Result result = decoder.Decode(pixels, readBuffer.width, readBuffer.height);
            if (result == null)
            {
                return false;
            }
            text = result.Text;
            return true;
        }

        // 판독 문자열 L01·L02·L03을 레인으로 변환
        public static bool TryParseLane(string text, out DestinationLane lane)
        {
            switch (text)
            {
                case "L01": lane = DestinationLane.Lane1; return true;
                case "L02": lane = DestinationLane.Lane2; return true;
                case "L03": lane = DestinationLane.Lane3; return true;
                default: lane = DestinationLane.Lane1; return false;
            }
        }

        // 초기화 버튼: 감지 기록과 집계 초기화
        public void ResetToInitialState()
        {
            inside.Clear();
            attempts.Clear();
            ReadCount = 0;
            FailCount = 0;
            LastText = null;
        }
        #endregion
    }
}
