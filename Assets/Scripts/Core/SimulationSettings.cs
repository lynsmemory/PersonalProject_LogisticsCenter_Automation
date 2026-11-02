using UnityEngine;

namespace Logistics
{
    // 공정 설정값. 기본(L=3, N=10)·시연(L=2, N=3) 에셋 2개를 만든다. (FR-04, FR-12, FR-16, FR-21, FR-23)
    [CreateAssetMenu(menuName = "Logistics/Simulation Settings", fileName = "SimulationSettings")]
    public class SimulationSettings : ScriptableObject
    {
        // 층당 박스 수 (고정, 파레트 460×400mm에 3×2)
        public const int BoxesPerLayer = 6;

        [Tooltip("기본/시연 구분")]
        [SerializeField] private SettingsPreset preset = SettingsPreset.Default;

        [Tooltip("적재 층 수 L")]
        [SerializeField, Min(1)] private int palletLayerCount = 3;

        [Tooltip("만재 파레트 수 N")]
        [SerializeField, Min(1)] private int truckPalletCapacity = 10;

        [Tooltip("박스 투입 간격 (초). 측정 후 확정: 10/28 물리 시험에서 6초·10초 간격 정상 분기")]
        [SerializeField, Min(0.5f)] private float spawnIntervalSeconds = 6f;

        [Tooltip("생산 속도 조절 범위 (배율). 측정 후 확정")]
        [SerializeField] private Vector2 speedMultiplierRange = new Vector2(0.5f, 2f);

        public SettingsPreset Preset => preset;
        public int PalletLayerCount => palletLayerCount;
        public int TruckPalletCapacity => truckPalletCapacity;
        public float SpawnIntervalSeconds => spawnIntervalSeconds;
        public Vector2 SpeedMultiplierRange => speedMultiplierRange;

        // 파레트 1개 완료 박스 수 (6×L)
        public int BoxesPerPallet => BoxesPerLayer * palletLayerCount;

        // 에셋 생성 도구에서 값을 채울 때 쓴다
        public void Initialize(SettingsPreset newPreset, int layers, int capacity, float interval, Vector2 speedRange)
        {
            preset = newPreset;
            palletLayerCount = layers;
            truckPalletCapacity = capacity;
            spawnIntervalSeconds = interval;
            speedMultiplierRange = speedRange;
        }
    }
}
