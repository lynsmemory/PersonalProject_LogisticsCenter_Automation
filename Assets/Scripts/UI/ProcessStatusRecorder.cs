using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace Logistics
{
    // 처리 제품, 시간당 생산량, 레인별 대기 수, 적재 단수, 설비 상태, 차량 상차 수를 모아 화면에 주고,
    // 같은 값을 박스 1개를 처리(파레트 적재 완료)할 때마다 날짜별 CSV에 한 줄씩 기록한다. 기록은 1달 보관한다. (FR-18)
    // 상차·차량 출발·초기화도 같은 형식의 줄로 남긴다 (이벤트 열로 구분). 파일 형식·주기·보관 기간은 SRS상 확인 필요 항목이며 설계 제안값을 쓴다.
    // 시간당 생산량 = 처리 수 ÷ 운전 시간(일시정지 제외). 초기화 버튼을 누르면 처리 수·생산량을 0으로 하고 CSV에는 초기화 줄을 남긴다.
    public class ProcessStatusRecorder : MonoBehaviour
    {
        #region 1. 연결과 설정
        [Tooltip("값 수집 대상")]
        [SerializeField] private LogisticsSimulationController simulation;

        [Tooltip("기록 폴더. 상대 경로면 Application.persistentDataPath 아래")]
        [SerializeField] private string recordFolder = "ProcessRecords";

        [Tooltip("기록 보관 기간 (일, 1달)")]
        [SerializeField, Min(1)] private int retentionDays = 30;

        [Tooltip("CSV 기록 사용")]
        [SerializeField] private bool writeFile = true;
        #endregion

        #region 2. 상태
        private const string FilePrefix = "ProcessRecord_";
        private readonly StringBuilder line = new StringBuilder();
        private bool writeFailed;

        public int ProcessedBoxCount { get; private set; }   // 총 처리 제품 수 (차량 교체 때도 유지, 초기화 때 0)
        public float RunningSeconds { get; private set; }     // 운전 시간 (일시정지·정지 제외)
        public float BoxesPerHour => RunningSeconds > 1f ? ProcessedBoxCount * 3600f / RunningSeconds : 0f;   // 시간당 생산량
        public int RecordCount { get; private set; }          // 이번 실행에서 쓴 줄 수
        public string RecordFolderPath => Path.IsPathRooted(recordFolder) ? recordFolder : Path.Combine(Application.persistentDataPath, recordFolder);
        public string TodayFilePath => Path.Combine(RecordFolderPath, FilePrefix + DateTime.Now.ToString("yyyyMMdd", CultureInfo.InvariantCulture) + ".csv");
        #endregion

        #region 3. 수집
        private void OnEnable()
        {
            if (simulation == null)
            {
                return;
            }
            simulation.StateChanged += OnStateChanged;
            foreach (LaneProcessController lane in simulation.LaneProcesses)
            {
                if (lane == null) continue;
                lane.BoxProcessed += OnBoxProcessed;
                if (lane.Dock != null)
                {
                    lane.Dock.PalletLoaded += OnPalletLoaded;
                    lane.Dock.Departed += OnTruckDeparted;
                }
            }
        }

        private void OnDisable()
        {
            if (simulation == null)
            {
                return;
            }
            simulation.StateChanged -= OnStateChanged;
            foreach (LaneProcessController lane in simulation.LaneProcesses)
            {
                if (lane == null) continue;
                lane.BoxProcessed -= OnBoxProcessed;
                if (lane.Dock != null)
                {
                    lane.Dock.PalletLoaded -= OnPalletLoaded;
                    lane.Dock.Departed -= OnTruckDeparted;
                }
            }
        }

        private void Start()
        {
            DeleteExpiredFiles();
        }

        private void Update()
        {
            if (simulation != null && simulation.State == SimulationRunState.Running)
            {
                RunningSeconds += Time.deltaTime;
            }
        }

        // 박스 1개 적재 완료 시 처리 수 +1, 값 수집 후 기록
        private void OnBoxProcessed(DestinationLane lane)
        {
            ProcessedBoxCount++;
            WriteRecord("박스처리", lane);
        }

        private void OnPalletLoaded(int loaded)
        {
            WriteRecord("상차", null);
        }

        private void OnTruckDeparted()
        {
            WriteRecord("차량출발", null);
        }

        // 초기화 버튼(정지 상태로 전환)에서 처리 수·생산량 0, 초기화 줄 기록
        private void OnStateChanged(SimulationRunState state)
        {
            if (state == SimulationRunState.Stopped)
            {
                ResetToInitialState();
            }
        }

        // 설비 값 수집: 레인별 순환/스토퍼/픽업 대기, 적재 수·단수, FR5·AMR 상태, 차량 상차 수·출발 수
        public string CollectStatus()
        {
            var values = new StringBuilder();
            values.Append(ProcessedBoxCount).Append(',').Append(BoxesPerHour.ToString("F1", CultureInfo.InvariantCulture));
            if (simulation == null)
            {
                return values.ToString();
            }
            foreach (LaneProcessController lane in simulation.LaneProcesses)
            {
                if (lane == null)
                {
                    values.Append(",,,,,,,,,");
                    continue;
                }
                int circulating = lane.OccupancyMonitor != null ? lane.OccupancyMonitor.CirculatingCount : 0;
                int stopper = lane.MiddleStopper != null && lane.MiddleStopper.HasWaitingBox ? 1 : 0;
                int pickup = lane.PickupZone != null && !lane.PickupZone.IsEmpty ? 1 : 0;
                int placed = lane.Pallet != null ? lane.Pallet.PlacedCount : 0;
                values.Append(',').Append(circulating).Append(',').Append(stopper).Append(',').Append(pickup)
                      .Append(',').Append(placed).Append(',').Append(LayerText(lane.Pallet))
                      .Append(',').Append(PalletizingStateName(lane.Palletizing))
                      .Append(',').Append(ForkliftStateName(lane.Forklift))
                      .Append(',').Append(TruckText(lane.Dock))
                      .Append(',').Append(lane.Dock != null ? lane.Dock.DepartureCount : 0);
            }
            return values.ToString();
        }

        // 수집한 값을 CSV 한 줄로 기록
        public void WriteRecord(string eventName, DestinationLane? lane)
        {
            if (!writeFile || writeFailed)
            {
                return;
            }
            line.Clear();
            line.Append(DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff", CultureInfo.InvariantCulture))
                .Append(',').Append(eventName)
                .Append(',').Append(lane.HasValue ? $"레인{(int)lane.Value + 1}" : "")
                .Append(',').Append(CollectStatus());
            try
            {
                string path = OpenTodayFile();
                File.AppendAllText(path, line.Append("\r\n").ToString(), new UTF8Encoding(false));
                RecordCount++;
            }
            catch (Exception e)
            {
                writeFailed = true;
                Debug.LogWarning($"[ProcessStatusRecorder] 기록 실패, 이번 실행에서는 기록을 멈춤: {e.Message}", this);
            }
        }

        // 날짜별 CSV 열기. 같은 날 다시 실행하면 그 날 파일에 이어 쓴다. 새 파일이면 머리줄(엑셀용 BOM 포함)을 쓴다
        public string OpenTodayFile()
        {
            Directory.CreateDirectory(RecordFolderPath);
            string path = TodayFilePath;
            if (!File.Exists(path))
            {
                var header = new StringBuilder("시각,이벤트,레인,총처리,시간당생산량");
                for (int i = 1; i <= 3; i++)
                {
                    header.Append($",레인{i}순환,레인{i}스토퍼,레인{i}픽업,레인{i}적재수,레인{i}적재단,레인{i}FR5,레인{i}AMR,레인{i}차량상차,레인{i}차량출발");
                }
                File.WriteAllText(path, header.Append("\r\n").ToString(), new UTF8Encoding(true));
            }
            return path;
        }

        // 보관 기간이 지난 CSV 삭제 (파일 이름의 날짜 기준)
        public void DeleteExpiredFiles()
        {
            if (!writeFile)
            {
                return;
            }
            try
            {
                if (!Directory.Exists(RecordFolderPath)) return;
                DateTime limit = DateTime.Today.AddDays(-retentionDays);
                foreach (string file in Directory.GetFiles(RecordFolderPath, FilePrefix + "*.csv"))
                {
                    string stamp = Path.GetFileNameWithoutExtension(file).Substring(FilePrefix.Length);
                    if (DateTime.TryParseExact(stamp, "yyyyMMdd", CultureInfo.InvariantCulture, DateTimeStyles.None, out DateTime day) && day < limit)
                    {
                        File.Delete(file);
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[ProcessStatusRecorder] 오래된 기록 정리 실패: {e.Message}", this);
            }
        }

        // 초기화 버튼: 총 처리 수·생산량 0. CSV 파일은 지우지 않고 초기화 줄을 남긴다
        public void ResetToInitialState()
        {
            ProcessedBoxCount = 0;
            RunningSeconds = 0f;
            WriteRecord("초기화", null);
        }
        #endregion

        #region 4. 표시 문자열
        // 적재 단수: 완성한 층 / L
        public static string LayerText(PalletLoadState pallet)
        {
            return pallet != null ? $"{pallet.PlacedCount / SimulationSettings.BoxesPerLayer}/{pallet.LayerCount}" : "-";
        }

        public static string TruckText(DockVehicleManager dock)
        {
            if (dock == null) return "-";
            return $"{dock.LoadedPalletCount}/{dock.Capacity}{(dock.IsDeparting ? " 교체중" : "")}";
        }

        public static string PalletizingStateName(PalletizingSequence palletizing)
        {
            if (palletizing == null) return "-";
            switch (palletizing.State)
            {
                case PalletizingState.Picking: return "집기";
                case PalletizingState.Placing: return "적재";
                case PalletizingState.Returning: return "복귀";
                default: return "대기";
            }
        }

        public static string ForkliftStateName(AutonomousForkliftController forklift)
        {
            if (forklift == null) return "-";
            switch (forklift.State)
            {
                case ForkliftState.MovingToPallet: return "파레트이동";
                case ForkliftState.Lifting: return "인양";
                case ForkliftState.MovingToDock: return "도크이동";
                case ForkliftState.Loading: return forklift.LoadRetryCount > 0 ? $"상차재시도{forklift.LoadRetryCount}" : "상차";
                case ForkliftState.Returning: return "복귀";
                case ForkliftState.StoppedByObstacle: return "장애물정지";
                case ForkliftState.WaitingForRoute: return "경로재탐색대기";
                case ForkliftState.Faulted: return "고장정지";
                default: return "대기";
            }
        }
        #endregion
    }
}
