using System.Collections.Generic;
using UnityEngine;
using ZXing;
using ZXing.Common;

namespace Logistics
{
    // ZXing.Net으로 L01·L02·L03 Code 128 라벨 이미지를 만들어 둔다. 레인마다 한 번만 만들고 같은 이미지를 재사용한다. (FR-06)
    // 프로젝트의 ZXing 0.15.0 Unity 빌드는 BarcodeWriter.Write(Color32 출력)에 투명 픽셀이 섞여 판독에 실패하므로
    // Encode로 받은 BitMatrix를 직접 검정·흰색으로 칠한다 (10/29 확인).
    public class BarcodeLabelPrinter : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("라벨 이미지 가로 픽셀 (바코드 샘플 PNG와 같은 512)")]
        [SerializeField, Min(64)] private int labelWidth = 512;

        [Tooltip("라벨 이미지 세로 픽셀")]
        [SerializeField, Min(32)] private int labelHeight = 256;

        [Tooltip("바코드 좌우 여백 (모듈 수)")]
        [SerializeField, Min(0)] private int quietZone = 10;
        #endregion

        #region 2. 라벨 생성
        private readonly Dictionary<DestinationLane, Texture2D> labels = new Dictionary<DestinationLane, Texture2D>();

        // 레인 라벨 반환 (없으면 생성)
        public Texture2D GetLabel(DestinationLane lane)
        {
            if (!labels.TryGetValue(lane, out Texture2D label) || label == null)
            {
                label = CreateLabel(lane);
                labels[lane] = label;
            }
            return label;
        }

        // Encode 결과 BitMatrix로 텍스처 생성
        public Texture2D CreateLabel(DestinationLane lane)
        {
            string text = ToBarcodeText(lane);
            var writer = new BarcodeWriter
            {
                Format = BarcodeFormat.CODE_128,
                Options = new EncodingOptions { Width = labelWidth, Height = labelHeight, Margin = quietZone, PureBarcode = true }
            };
            BitMatrix matrix = writer.Encode(text);

            //1. BitMatrix 위쪽 행이 텍스처 아래쪽 행이 되도록 뒤집어 칠한다 (1차원 바코드라 판독에는 영향 없음)
            var pixels = new Color32[matrix.Width * matrix.Height];
            var black = new Color32(0, 0, 0, 255);
            var white = new Color32(255, 255, 255, 255);
            for (int y = 0; y < matrix.Height; y++)
            {
                int row = (matrix.Height - 1 - y) * matrix.Width;
                for (int x = 0; x < matrix.Width; x++)
                {
                    pixels[row + x] = matrix[x, y] ? black : white;
                }
            }

            //2. 막대 경계가 흐려지지 않게 점 필터, 밉맵 없음
            var texture = new Texture2D(matrix.Width, matrix.Height, TextureFormat.RGBA32, false)
            {
                name = $"Barcode_{text}",
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp
            };
            texture.SetPixels32(pixels);
            texture.Apply(false, true);
            return texture;
        }

        // 레인 → L01·L02·L03
        public static string ToBarcodeText(DestinationLane lane)
        {
            return $"L{(int)lane + 1:00}";
        }
        #endregion
    }
}
