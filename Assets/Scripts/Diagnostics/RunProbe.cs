using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Logistics
{
    // 실행 인자 -runprobe로만 켜지는 실시간 운전 측정기. 인자가 없으면 아무 것도 만들지 않는다. (NFR 성능·연속 운전 시험용)
    // 빌드 실행 예: LogisticsCenter_Automation.exe -runprobe -probeSeconds 600 -probePreset demo -probeSpeed 1 -probeOut <폴더> [-probeShots 1 -probeViewSeconds 20]
    // 운전 화면과 같은 함수(OperatorControlPanel)로 설정 선택·시작을 하고, 실제 시간 기준으로 FPS·프레임 시간·처리 수·표시 갱신 간격·오류를 1초마다 기록한다.
    // -probeShots가 0보다 크면 그 간격(초)으로 화면을 JPEG로 저장하고 -probeViewSeconds마다 4개 뷰를 차례로 전환한다 (시연 영상용 프레임).
    // -probeRecordFolder를 주면 공정 기록 CSV를 그 폴더에 쓴다 (사용자 기록 폴더 대신 시험 전용 폴더).
    // 표시 지연: 처리 이벤트·상차 이벤트 시각부터 운전 화면 문자열에 새 값이 처음 들어간 프레임, 그 프레임의 화면 그리기 끝까지의 실제 시간을 잰다.
    // 판독 시간: 바코드 판독 성공마다 촬영·해독 시간(BarcodeScanner.LastDecodeMilliseconds)을 모은다.
    // 지정 시간이 지나면 요약을 쓰고 종료한다.
    public class RunProbe : MonoBehaviour
    {
        private float seconds = 600f, speed = 1f, shotInterval = 0f, viewSeconds = 20f;
        private string outDir = "RunProbe", preset = "demo";
        private OperatorControlPanel panel;
        private LogisticsSimulationController simulation;
        private ProcessStatusRecorder recorder;
        private StreamWriter perSecond;
        private readonly List<float> frameTimes = new List<float>();
        private readonly List<float> secondFrames = new List<float>();
        private readonly List<float> displayGaps = new List<float>();
        private float startTime, lastSecond, lastShot = -999f, lastDisplayChange;
        private string lastDisplay;
        private int errors, warnings, shots, view;
        private readonly List<string> firstErrors = new List<string>();
        private readonly List<(float at, int frame, string expect, string kind)> pendingShow = new List<(float, int, string, string)>();
        private readonly List<float> showStringMs = new List<float>(), showRenderMs = new List<float>(), showFrames = new List<float>();
        private readonly List<float> showStringMsLoad = new List<float>();
        private readonly List<float> decodeMs = new List<float>();
        private int readFails, missedShow;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Boot()
        {
            if (!Environment.GetCommandLineArgs().Contains("-runprobe")) return;
            new GameObject("RunProbe").AddComponent<RunProbe>();
        }

        private static string Arg(string name, string fallback)
        {
            string[] args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : fallback;
        }

        private static float ArgF(string name, float fallback) => float.TryParse(Arg(name, ""), NumberStyles.Float, CultureInfo.InvariantCulture, out float v) ? v : fallback;

        private void Start()
        {
            seconds = ArgF("-probeSeconds", seconds);
            speed = ArgF("-probeSpeed", speed);
            shotInterval = ArgF("-probeShots", shotInterval);
            viewSeconds = ArgF("-probeViewSeconds", viewSeconds);
            preset = Arg("-probePreset", preset);
            string recordFolder = Arg("-probeRecordFolder", null);
            outDir = Arg("-probeOut", Path.Combine(Application.persistentDataPath, outDir));
            Directory.CreateDirectory(outDir);
            if (shotInterval > 0f) Directory.CreateDirectory(Path.Combine(outDir, "frames"));
            Application.runInBackground = true;
            Application.logMessageReceived += OnLog;
            panel = FindAnyObjectByType<OperatorControlPanel>();
            simulation = FindAnyObjectByType<LogisticsSimulationController>();
            recorder = FindAnyObjectByType<ProcessStatusRecorder>();
            if (!string.IsNullOrEmpty(recordFolder))
            {
                typeof(ProcessStatusRecorder).GetField("recordFolder", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic).SetValue(recorder, recordFolder);
                Directory.CreateDirectory(recorder.RecordFolderPath);
            }
            foreach (LaneProcessController lane in simulation.LaneProcesses)
            {
                lane.BoxProcessed += _ => pendingShow.Add((Time.realtimeSinceStartup, Time.frameCount, $"처리 {recorder.ProcessedBoxCount}개", "처리"));
                DockVehicleManager dock = lane.Dock;
                int laneNumber = (int)lane.Lane + 1;
                if (dock != null) dock.PalletLoaded += n => pendingShow.Add((Time.realtimeSinceStartup, Time.frameCount, $"차량 {ProcessStatusRecorder.TruckText(dock)}", $"상차{laneNumber}"));
            }
            simulation.BarcodeScanner.BoxRead += _ => decodeMs.Add(simulation.BarcodeScanner.LastDecodeMilliseconds);
            simulation.BarcodeScanner.ReadFailed += _ => readFails++;
            perSecond = new StreamWriter(Path.Combine(outDir, "probe_seconds.csv"), false, new UTF8Encoding(true)) { AutoFlush = true };
            perSecond.WriteLine("실시간초,FPS,최대프레임ms,처리,시간당생산량,투입,판독,판독실패,차량출발합,오류,운전상태");
            panel.OnPresetSelected(preset == "default" ? SettingsPreset.Default : SettingsPreset.Demonstration);
            panel.OnStart();
            if (!Mathf.Approximately(speed, 1f)) panel.OnSpeedChanged(speed);
            startTime = Time.realtimeSinceStartup;
            lastSecond = startTime;
            lastDisplayChange = startTime;
            Debug.Log($"[RunProbe] 시작 preset={preset} speed={speed} seconds={seconds} shots={shotInterval} screen={Screen.width}x{Screen.height} record={recorder.RecordFolderPath}");
        }

        private void OnLog(string message, string trace, LogType type)
        {
            if (type == LogType.Error || type == LogType.Exception || type == LogType.Assert)
            {
                errors++;
                if (firstErrors.Count < 10) firstErrors.Add($"{type}: {message}");
            }
            else if (type == LogType.Warning) warnings++;
        }

        private void Update()
        {
            float now = Time.realtimeSinceStartup;
            float dt = Time.unscaledDeltaTime;
            frameTimes.Add(dt);
            secondFrames.Add(dt);
            string display = panel != null ? panel.DisplayText : null;
            if (display != lastDisplay)
            {
                displayGaps.Add(now - lastDisplayChange);
                lastDisplayChange = now;
                lastDisplay = display;
            }
            if (now - lastSecond >= 1f)
            {
                float fps = secondFrames.Count / (now - lastSecond);
                perSecond.WriteLine(string.Join(",", (now - startTime).ToString("F0", CultureInfo.InvariantCulture), fps.ToString("F1", CultureInfo.InvariantCulture),
                    (secondFrames.Max() * 1000f).ToString("F1", CultureInfo.InvariantCulture), recorder.ProcessedBoxCount, recorder.BoxesPerHour.ToString("F1", CultureInfo.InvariantCulture),
                    simulation.BoxSpawner.SpawnedCount, simulation.BarcodeScanner.ReadCount, simulation.BarcodeScanner.FailCount,
                    simulation.LaneProcesses.Sum(l => l.Dock != null ? l.Dock.DepartureCount : 0), errors, simulation.State));
                secondFrames.Clear();
                lastSecond = now;
            }
            if (shotInterval > 0f)
            {
                int wantView = (int)((now - startTime) / Mathf.Max(1f, viewSeconds)) % 4;
                if (wantView != view)
                {
                    view = wantView;
                    panel.OnViewSelected(view);
                }
            }
            if (now - startTime >= seconds) Finish();
        }

        private void LateUpdate()
        {
            // 기대 문자열은 이벤트 시점 값 (기록기가 같은 이벤트에서 먼저 처리 수를 올린다). 5초 안에 반영되지 않으면 미반영으로 센다
            if (pendingShow.Count > 0)
            {
                string display = panel.DisplayText;
                for (int i = pendingShow.Count - 1; i >= 0; i--)
                {
                    var p = pendingShow[i];
                    if (Time.realtimeSinceStartup - p.at > 5f) { missedShow++; pendingShow.RemoveAt(i); continue; }
                    if (!display.Contains(p.expect)) continue;
                    float ms = (Time.realtimeSinceStartup - p.at) * 1000f;
                    if (p.kind == "처리") { showStringMs.Add(ms); showFrames.Add(Time.frameCount - p.frame); }
                    else showStringMsLoad.Add(ms);
                    StartCoroutine(RenderedAt(p.at));
                    pendingShow.RemoveAt(i);
                }
            }
            if (shotInterval <= 0f || Time.realtimeSinceStartup - lastShot < shotInterval) return;
            lastShot = Time.realtimeSinceStartup;
            StartCoroutine(Shot());
        }

        // 문자열에 들어간 프레임의 화면 그리기(OnGUI 포함)가 끝난 시각
        private System.Collections.IEnumerator RenderedAt(float at)
        {
            yield return new WaitForEndOfFrame();
            showRenderMs.Add((Time.realtimeSinceStartup - at) * 1000f);
        }

        private static string Stat(List<float> v, string unit) => v.Count == 0 ? "표본 없음" : $"평균 {v.Average():F1}{unit}, 중앙 {v.OrderBy(x => x).ElementAt(v.Count / 2):F1}{unit}, 95% {v.OrderBy(x => x).ElementAt(Mathf.Min(v.Count - 1, (int)(v.Count * 0.95f))):F1}{unit}, 최대 {v.Max():F1}{unit}, 표본 {v.Count}";

        private System.Collections.IEnumerator Shot()
        {
            yield return new WaitForEndOfFrame();
            Texture2D image = ScreenCapture.CaptureScreenshotAsTexture();
            File.WriteAllBytes(Path.Combine(outDir, "frames", $"frame_{shots++:00000}.jpg"), image.EncodeToJPG(80));
            Destroy(image);
        }

        private void Finish()
        {
            enabled = false;
            float elapsed = Time.realtimeSinceStartup - startTime;
            var sorted = frameTimes.Skip(Mathf.Min(frameTimes.Count - 1, 30)).OrderByDescending(x => x).ToList();
            float avgFps = frameTimes.Count / Mathf.Max(0.001f, frameTimes.Sum());
            float low1 = sorted.Count > 0 ? 1f / sorted.Take(Mathf.Max(1, sorted.Count / 100)).Average() : 0f;
            float worst = sorted.Count > 0 ? sorted[0] * 1000f : 0f;
            int processed = recorder.ProcessedBoxCount;
            var s = new StringBuilder();
            s.AppendLine($"실시간 운전 측정 요약 (실제 시간 기준, captureFramerate 없음)");
            s.AppendLine($"설정 {preset}, 생산속도 x{speed}, 화면 {Screen.width}x{Screen.height}, 실행 {elapsed:F0}s, 프레임 {frameTimes.Count}");
            s.AppendLine($"평균 FPS {avgFps:F1}, 하위 1% FPS {low1:F1}, 최대 프레임 시간 {worst:F1}ms (첫 30프레임 제외)");
            s.AppendLine($"처리 {processed}개, 실제 시간당 {processed * 3600f / Mathf.Max(1f, elapsed):F1}개, 기록기 시간당 {recorder.BoxesPerHour:F1}개, 투입 {simulation.BoxSpawner.SpawnedCount}, 판독 {simulation.BarcodeScanner.ReadCount}/실패 {simulation.BarcodeScanner.FailCount}");
            s.AppendLine("레인 " + string.Join(" | ", simulation.LaneProcesses.Select(l => $"{l.Lane} 처리 {l.ProcessedCount} 적재 {l.Pallet.PlacedCount} AMR {l.Forklift?.State} 상차 {l.Dock?.TotalLoadedPallets} 출발 {l.Dock?.DepartureCount} 실패정지 {l.IsHaltedByFailure}")));
            s.AppendLine($"표시 지연(처리 이벤트→화면 문자열 반영): {Stat(showStringMs, "ms")}, 프레임 차 {Stat(showFrames, "")}");
            s.AppendLine($"표시 지연(처리·상차 이벤트→반영 프레임 그리기 끝): {Stat(showRenderMs, "ms")}");
            s.AppendLine($"표시 지연(상차 이벤트→화면 문자열 반영): {Stat(showStringMsLoad, "ms")}, 5초 안에 반영 못 한 이벤트 {missedShow + pendingShow.Count}");
            s.AppendLine($"바코드 판독 시간(촬영+해독, 성공 판독마다): {Stat(decodeMs, "ms")}, 판독 실패 {readFails}");
            s.AppendLine($"기록 폴더 {recorder.RecordFolderPath}, 이번 실행 기록 줄 {recorder.RecordCount}");
            s.AppendLine($"표시 문자열 값 변화 간격(지연 아님): 평균 {(displayGaps.Count > 0 ? displayGaps.Average() * 1000f : 0f):F1}ms, 최대 {(displayGaps.Count > 0 ? displayGaps.Max() * 1000f : 0f):F1}ms, 변경 {displayGaps.Count}회");
            s.AppendLine($"오류 {errors}건, 경고 {warnings}건, 화면 저장 {shots}장");
            foreach (string e in firstErrors) s.AppendLine($"  {e}");
            File.WriteAllText(Path.Combine(outDir, "probe_summary.txt"), s.ToString(), new UTF8Encoding(true));
            perSecond.Dispose();
            Debug.Log("[RunProbe] 종료\n" + s);
            Application.Quit(errors == 0 ? 0 : 1);
        }
    }
}
