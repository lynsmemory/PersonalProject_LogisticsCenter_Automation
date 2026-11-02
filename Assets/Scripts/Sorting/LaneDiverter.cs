using System;
using UnityEngine;

namespace Logistics
{
    // Swing-arm (flipper) diverter at one lane mouth of the main loop (FR-07).
    // The hinge is on the lane-side guide. Folded, the arm closes the lane opening;
    // extended, it acts as an oblique fence. In passive mode the belt always moves
    // straight and contact with that fence redirects the selected box into the lane.
    // flowSign and laneSideSign describe the hinge's local coordinate convention.
    public class LaneDiverter : MonoBehaviour
    {
        [Tooltip("Destination lane served by this diverter (was the string laneId L01/L02/L03).")]
        [SerializeField] private DestinationLane lane = DestinationLane.Lane1;
        [Tooltip("Kinematic arm body; its pivot is the hinge and the arm extends along its local +Z.")]
        [SerializeField] private Rigidbody arm;
        [Tooltip("Kinematic belt section under the arm sweep; its colliders are the belt there.")]
        [SerializeField] private Rigidbody section;
        [Tooltip("Keep the belt moving straight; the angled arm alone deflects the box.")]
        [SerializeField] private bool passiveDeflection = true;
        [Tooltip("Main flow along local Z: +1 or -1.")]
        [SerializeField] private float flowSign = 1f;
        [Tooltip("Lane side along local X: +1 or -1.")]
        [SerializeField] private float laneSideSign = 1f;
        [SerializeField, Min(0f)] private float speed = 0.5f;
        [SerializeField, Min(0.1f)] private float armLength = 1.4f;
        [SerializeField, Range(5f, 80f)] private float extendedAngle = 35f;
        [SerializeField, Min(0.05f)] private float swingTime = 0.3f;
        [Tooltip("Belt centre line and half width in this component's local X (m).")]
        [SerializeField] private float beltCentreX = 0.4f;
        [SerializeField, Min(0.1f)] private float beltHalfWidth = 0.437f;
        [Tooltip("Local Z range upstream of the hinge in which a matching box fires the arm (m).")]
        [SerializeField] private Vector2 triggerWindow = new Vector2(-1.0f, -0.25f);
        [Tooltip("A box closer than this behind the candidate would ride the section and be pushed or struck by the arm, so the candidate is let past to circulate (m).")]
        [SerializeField, Min(0f)] private float followerClearance = 1.5f;
        [Tooltip("Box centre beyond the belt edge at which the hand-off counts as done (m).")]
        [SerializeField, Min(0f)] private float handOffMargin = 0.1f;
        [SerializeField, Min(0.1f)] private float divertTimeout = 4f;
        [Tooltip("While not diverting, the section steers boxes this many degrees away from the open lane mouth (toward the far guide), " +
                 "so small boxes riding the lane-side edge do not slip into the lane undiverted. Tried at 10 deg on 10/29: worse, left at 0; the mouth guard handles this.")]
        [SerializeField, Range(0f, 30f)] private float passSteerAngle = 0f;
        [Tooltip("Kinematic guard wall across the lane mouth in the side guide. It stays up so passing boxes cannot slip into the lane " +
                 "by themselves (180 mm boxes did, 1-2 per 600 s on 10/29), and drops below the lane floor only while this diverter hands a box off.")]
        [SerializeField] private Rigidbody mouthGuard;
        [Tooltip("How far the guard drops when open (m, world down).")]
        [SerializeField, Min(0f)] private float guardDrop = 0.2f;
        [SerializeField, Min(0.05f)] private float guardMoveTime = 0.2f;
        [Tooltip("Time the guard stays down after the hand-off so the box's trailing edge is clear before it rises (s).")]
        [SerializeField, Min(0f)] private float guardCloseDelay = 0.6f;
        [SerializeField] private bool running = true;

        [Header("직선 푸셔")]
        [Tooltip("회전 유도판 대신 벨트 반대쪽의 밀판을 직선 왕복시킨다")]
        [SerializeField] private bool linearPush;
        [SerializeField] private Vector3 pushHomeLocal = new Vector3(1.31f, 0.11f, 0.19f);
        [SerializeField] private Vector3 pushEndLocal = new Vector3(-0.20f, 0.11f, 0.19f);
        [SerializeField, Min(0.1f)] private float pushSeconds = 0.5f;
        [SerializeField, Min(0.1f)] private float returnSeconds = 0.45f;
        [Tooltip("밀판의 벨트 진행 방향 폭. 실제 메시·충돌체 폭과 일치시킨다")]
        [SerializeField, Min(0.1f)] private float pushPlateLength = 0.80f;
        private int pushPhase;
        private float pushElapsed;
        private bool pushReported;
        // 밀기 후보 구간에 들어왔던 이 레인 박스별 마지막 보류 이유 (분기 없이 지나갈 때 로그로 남긴다)
        private readonly System.Collections.Generic.Dictionary<Rigidbody, string> pushBlockReasons = new System.Collections.Generic.Dictionary<Rigidbody, string>();
        private readonly System.Collections.Generic.List<Rigidbody> pushCandidates = new System.Collections.Generic.List<Rigidbody>();

        // 분기 보류 이유를 구분해 시험과 운전 진단에서 확인한다.
        public string LastBlockReason { get; private set; }

        private float angle;
        private float elapsed;
        private float guardOpen;          // 0 up (closed), 1 down (open)
        private float sinceHandOff = float.MaxValue;
        private Vector3 guardClosedPosition;
        private Rigidbody current;
        private readonly Collider[] hits = new Collider[128];
        private readonly float[] bodies = new float[128];
        private bool overlapCapacityWarned;

        /// <summary>Set by the lane monitor; when false matching boxes pass and keep circulating.</summary>
        public bool AcceptsBoxes { get; set; } = true;
        public DestinationLane Lane => lane;
        /// <summary>Belt speed scale applied by MainConveyorController (production speed).</summary>
        public float SpeedMultiplier { get; set; } = 1f;
        public bool IsDiverting => current != null;
        public bool IsArmRetracted => angle <= 0.01f;
        public bool IsGuardClosed => mouthGuard != null ? guardOpen <= 0.01f : IsArmRetracted;
        public bool IsRunning => running;

        /// <summary>The arm started swinging out for this box.</summary>
        public event Action<Rigidbody> DivertStarted;
        /// <summary>The box left the belt into the lane; raised once per box.</summary>
        public event Action<Rigidbody> Diverted;
        /// <summary>The box passed the arm or the hand-off timed out; the arm retracts.</summary>
        public event Action<Rigidbody> DivertFailed;
        /// <summary>A box read for this lane went past the arm without being diverted (lane full or spacing); it circulates.</summary>
        public event Action<Rigidbody> BoxPassed;

        // Matching boxes seen in the trigger window, watched until they are diverted or go past the arm tip.
        private readonly System.Collections.Generic.HashSet<Rigidbody> watched = new System.Collections.Generic.HashSet<Rigidbody>();
        private readonly System.Collections.Generic.List<Rigidbody> passedNow = new System.Collections.Generic.List<Rigidbody>();

        private float FlowSign => flowSign < 0f ? -1f : 1f;
        private float LaneSideSign => laneSideSign < 0f ? -1f : 1f;
        private float Reach => Mathf.Max(0f, armLength * Mathf.Cos(extendedAngle * Mathf.Deg2Rad) * FlowSign);
        private float LaneDistance(Vector3 localPosition) => (localPosition.x - (beltCentreX + LaneSideSign * beltHalfWidth)) * LaneSideSign;

        private void Awake()
        {
            foreach (var body in new[] { arm, section, mouthGuard })
                if (body != null)
                {
                    body.isKinematic = true;
                    body.useGravity = false;
                }
            if (mouthGuard != null) guardClosedPosition = mouthGuard.position;
        }

        private void FixedUpdate()
        {
            if (!running || arm == null) return;
            float dt = Time.fixedDeltaTime;
            WatchPassing();
            if (linearPush)
            {
                UpdatePush(dt);
                return;
            }
            if (current != null) Track(dt);
            else TryStart();

            float target = current != null ? extendedAngle : 0f;
            angle = Mathf.MoveTowards(angle, target, extendedAngle / swingTime * dt);
            arm.MovePosition(transform.position);
            arm.MoveRotation(transform.rotation * Quaternion.Euler(0f, angle, 0f));

            if (mouthGuard != null)
            {
                // Down from the moment a divert starts until guardCloseDelay after the box has left the belt.
                sinceHandOff = current != null ? 0f : sinceHandOff + dt;
                float guardTarget = current != null || sinceHandOff < guardCloseDelay ? 1f : 0f;
                // Never rise into a box sitting over the mouth: a rising kinematic wall launches it off the line (seen 10/29).
                if (guardTarget < guardOpen && GuardSpaceOccupied()) guardTarget = guardOpen;
                guardOpen = Mathf.MoveTowards(guardOpen, guardTarget, dt / guardMoveTime);
                mouthGuard.MovePosition(guardClosedPosition + Vector3.down * guardDrop * guardOpen);
            }

            if (section != null)
            {
                // Kinematic belt: step back, then MovePosition forward so contacts get the belt velocity
                // while the body itself stays in place.
                bool transfer = current != null && angle >= extendedAngle - 0.01f;
                Vector3 local = Vector3.forward * FlowSign;
                if (!passiveDeflection)
                    local = (transfer ? Quaternion.Euler(0f, extendedAngle, 0f) : Quaternion.Euler(0f, -passSteerAngle, 0f)) * local;
                Vector3 v = transform.TransformDirection(local) * speed * SpeedMultiplier;
                Vector3 p = section.position;
                section.position = p - v * dt;
                section.MovePosition(p);
            }
        }

        private void UpdatePush(float dt)
        {
            Vector3 home = transform.TransformPoint(pushHomeLocal);
            Vector3 end = transform.TransformPoint(pushEndLocal);
            if (section != null)
            {
                Vector3 p = section.position;
                Vector3 v = transform.forward * FlowSign * speed * SpeedMultiplier;
                section.position = p - v * dt;
                section.MovePosition(p);
            }

            if (pushPhase == 0)
            {
                arm.MovePosition(home);
                angle = 0f;
                if (mouthGuard == null || guardOpen <= 0.01f) TryStartPush();
            }
            else
            {
                // 밀판이 움직이는 동안 후보 구간에 들어온 박스는 "밀판 동작 중"으로 기록한다 (복귀가 끝나면 다시 판정)
                FindPushCandidates();
                foreach (Rigidbody body in pushCandidates)
                    if (body != current) pushBlockReasons[body] = pushPhase == 3 ? "밀판 복귀 중" : "밀판 전진 중";
                pushElapsed += dt;
                angle = extendedAngle;
                if (pushPhase == 1 && (mouthGuard == null || guardOpen >= 0.99f))
                {
                    pushPhase = 2;
                    pushElapsed = 0f;
                }
                if (pushPhase == 2)
                {
                    arm.MovePosition(Vector3.Lerp(home, end, Mathf.Clamp01(pushElapsed / pushSeconds)));
                    if (current != null && !pushReported && HasFullyEnteredLane(current))
                    {
                        pushReported = true;
                        Diverted?.Invoke(current);
                    }
                    if (pushElapsed >= pushSeconds)
                    {
                        if (!pushReported && current != null)
                        {
                            Debug.LogWarning($"[LaneDiverter {lane}] push failed for {current.name}", this);
                            DivertFailed?.Invoke(current);
                        }
                        current = null;
                        pushPhase = 3;
                        pushElapsed = 0f;
                    }
                }
                else if (pushPhase == 3)
                {
                    arm.MovePosition(Vector3.Lerp(end, home, Mathf.Clamp01(pushElapsed / returnSeconds)));
                    if (pushElapsed >= returnSeconds)
                    {
                        pushPhase = 0;
                        angle = 0f;
                    }
                }
            }

            if (mouthGuard != null)
            {
                float target = pushPhase != 0 ? 1f : 0f;
                if (target < guardOpen && GuardSpaceOccupied()) target = guardOpen;
                guardOpen = Mathf.MoveTowards(guardOpen, target, dt / guardMoveTime);
                mouthGuard.MovePosition(guardClosedPosition + Vector3.down * guardDrop * guardOpen);
            }
        }

        private bool HasFullyEnteredLane(Rigidbody body)
        {
            // 박스 중심이 아니라 벨트 쪽 마지막 외곽까지 입구를 넘어야 성공으로 센다.
            var shape = body.GetComponentInChildren<BoxCollider>();
            if (shape == null) return false;
            Bounds b = shape.bounds;
            Vector3 side = transform.right * LaneSideSign;
            float half = Vector3.Dot(b.extents, new Vector3(Mathf.Abs(side.x), Mathf.Abs(side.y), Mathf.Abs(side.z)));
            return LaneDistance(transform.InverseTransformPoint(b.center)) > half + 0.01f;
        }

        // 진행 방향 위치 (분기 장치 로컬 Z × 진행 방향 부호)
        private float Progress(Vector3 world) => transform.InverseTransformPoint(world).z * FlowSign;

        // 충돌체의 진행 방향 뒤끝·앞끝 (월드 AABB를 진행 방향에 투영)
        private void FlowExtent(Collider collider, out float rear, out float front)
        {
            Bounds b = collider.bounds;
            Vector3 f = transform.forward * FlowSign;
            float half = Mathf.Abs(b.extents.x * f.x) + Mathf.Abs(b.extents.y * f.y) + Mathf.Abs(b.extents.z * f.z);
            float c = Progress(b.center);
            rear = c - half;
            front = c + half;
        }

        // 밀판이 벨트 위를 덮는 진행 방향 구간 (뒤끝, 앞끝)
        private float PlateRear => pushHomeLocal.z * FlowSign - pushPlateLength * 0.5f;
        private float PlateFront => pushHomeLocal.z * FlowSign + pushPlateLength * 0.5f;
        private float BeltSpeed => speed * SpeedMultiplier;

        // 밀기 후보 구간 안에 있는 이 레인 판독 박스 (앞선 박스부터). 후보 구간 = 밀기 시작 시 박스 전체가 밀판 폭 안에 있고,
        // 전진하는 동안(pushSeconds) 벨트를 따라 움직여도 밀판 앞끝을 넘지 않는 위치
        private void FindPushCandidates()
        {
            pushCandidates.Clear();
            Vector3 centre = transform.TransformPoint(new Vector3(beltCentreX, 0.12f, (PlateRear + PlateFront) * 0.5f * FlowSign));
            int n = Physics.OverlapBoxNonAlloc(centre, new Vector3(beltHalfWidth + 0.1f, 0.2f, pushPlateLength * 0.5f), hits, transform.rotation, ~0, QueryTriggerInteraction.Ignore);
            float frontLimit = PlateFront - BeltSpeed * pushSeconds - 0.02f;
            for (int i = 0; i < n; i++)
            {
                Rigidbody body = hits[i].attachedRigidbody;
                if (body == null || body.isKinematic || pushCandidates.Contains(body)) continue;
                var box = body.GetComponent<LogisticsBox>();
                if (box == null || !box.HasReadResult || box.ReadLane != lane) continue;
                if (LaneDistance(transform.InverseTransformPoint(body.position)) > 0.2f) continue;
                var shape = body.GetComponentInChildren<BoxCollider>();
                if (shape == null) continue;
                FlowExtent(shape, out float rear, out float front);
                if (rear < PlateRear + 0.02f || front > frontLimit) continue;
                pushCandidates.Add(body);
            }
            pushCandidates.Sort((a, b) => Progress(b.position).CompareTo(Progress(a.position)));
        }

        // 밀기 시작 판정. 후보 구간에 있는 동안 매 물리 단계 다시 판정한다.
        // 간섭 = 다른 박스가 밀기·복귀 동안(pushSeconds + returnSeconds) 벨트를 따라 움직인 범위가 밀판 구간과 겹침.
        // 밀판 앞끝을 이미 지난 앞 박스는 계속 멀어지므로 간섭이 아니다. 밀판 폭 안에 두 박스가 함께 있으면 계속 보류한다(붙은 박스 자동 분리 없음).
        private void TryStartPush()
        {
            LastBlockReason = null;
            FindPushCandidates();
            if (pushCandidates.Count == 0) return;
            if (!AcceptsBoxes)
            {
                LastBlockReason = "레인 대기 (중간 스토퍼 앞 1개)";
                foreach (Rigidbody body in pushCandidates) pushBlockReasons[body] = LastBlockReason;
                return;
            }
            float sweep = BeltSpeed * (pushSeconds + returnSeconds);
            float lo = PlateRear - 0.02f, hi = PlateFront + 0.02f;
            float xHalf = Mathf.Abs(pushHomeLocal.x - pushEndLocal.x) * 0.5f + 0.15f;
            Vector3 mid = new Vector3((pushHomeLocal.x + pushEndLocal.x) * 0.5f, pushHomeLocal.y, ((lo - sweep - 0.3f) + hi) * 0.5f * FlowSign);
            Vector3 half = new Vector3(xHalf, 0.2f, (hi - (lo - sweep - 0.3f)) * 0.5f);
            int n = Physics.OverlapBoxNonAlloc(transform.TransformPoint(mid), half, hits, transform.rotation, ~0, QueryTriggerInteraction.Ignore);
            if (n == hits.Length)
            {
                LastBlockReason = "감지 포화";
                foreach (Rigidbody body in pushCandidates) pushBlockReasons[body] = LastBlockReason;
                return;
            }
            Rigidbody candidate = pushCandidates[0];
            for (int i = 0; i < n; i++)
            {
                Rigidbody body = hits[i].attachedRigidbody;
                if (body == null || body == candidate || body.isKinematic || body.GetComponent<LogisticsBox>() == null) continue;
                if (LaneDistance(transform.InverseTransformPoint(body.position)) > 0.2f) continue;
                FlowExtent(hits[i], out float rear, out float front);
                if (front + sweep < lo || rear > hi) continue;
                LastBlockReason = $"앞뒤 박스 간섭 ({body.name})";
                foreach (Rigidbody c in pushCandidates) pushBlockReasons[c] = LastBlockReason;
                return;
            }
            current = candidate;
            pushBlockReasons.Remove(candidate);
            pushPhase = 1;
            pushElapsed = 0f;
            pushReported = false;
            DivertStarted?.Invoke(current);
        }

        private void TryStart()
        {
            if (!AcceptsBoxes) return;
            // Region over the belt from the follower clearance behind the trigger window to past the arm tip.
            // The retracted arm lies along the belt for its full length, so its tip sweeps that far downstream.
            float zMin = triggerWindow.x - followerClearance, zMax = Mathf.Max(0f, armLength * FlowSign) + 0.3f;
            Vector3 centre = transform.TransformPoint(new Vector3(beltCentreX, 0.25f, (zMin + zMax) * 0.5f * FlowSign));
            int n = Physics.OverlapBoxNonAlloc(centre, new Vector3(beltHalfWidth, 0.25f, (zMax - zMin) * 0.5f), hits, transform.rotation, ~0, QueryTriggerInteraction.Ignore);
            // A truncated query cannot prove that the swept region is clear.
            if (n == hits.Length)
            {
                if (!overlapCapacityWarned)
                {
                    overlapCapacityWarned = true;
                    Debug.LogWarning($"[LaneDiverter {lane}] clearance query full; divert deferred", this);
                }
                return;
            }
            Rigidbody candidate = null;
            bool blocked = false;
            int others = 0;
            for (int i = 0; i < n; i++)
            {
                var body = hits[i].attachedRigidbody;
                if (body == null || body.isKinematic || body == candidate) continue;
                Vector3 p = transform.InverseTransformPoint(body.position);
                var box = body.GetComponent<LogisticsBox>();
                bool matches = box != null && box.HasReadResult && box.ReadLane == lane;
                float progress = p.z * FlowSign;
                if (matches && candidate == null && progress >= triggerWindow.x && progress <= triggerWindow.y) candidate = body;
                else if (others < bodies.Length) bodies[others++] = progress;
            }
            if (candidate == null) return;
            float candidateZ = transform.InverseTransformPoint(candidate.position).z * FlowSign;
            for (int i = 0; i < others; i++)
            {
                // Ahead of the candidate in the sweep, or close behind it: either would be pushed into the lane too.
                // Small boxes can both fit inside the trigger window. Do not leave an
                // unchecked interval between the candidate and the end of that window.
                if (bodies[i] > candidateZ) blocked = true;
                if (bodies[i] <= candidateZ && candidateZ - bodies[i] < followerClearance) blocked = true;
            }
            if (blocked) return;
            if (passiveDeflection && HasCloseUpstreamBody(candidate, candidateZ)) return;
            current = candidate;
            elapsed = 0f;
            DivertStarted?.Invoke(candidate);
        }

        // Near the north turn, following boxes are still outside the straight belt's
        // X range. Include that curved approach before reserving an arm cycle.
        private bool HasCloseUpstreamBody(Rigidbody candidate, float candidateProgress)
        {
            int n = Physics.OverlapSphereNonAlloc(candidate.position, followerClearance, hits, ~0, QueryTriggerInteraction.Ignore);
            if (n == hits.Length) return true;
            float distanceSquared = followerClearance * followerClearance;
            for (int i = 0; i < n; i++)
            {
                var body = hits[i].attachedRigidbody;
                if (body == null || body == candidate || body.isKinematic || body.GetComponent<LogisticsBox>() == null) continue;
                Vector3 p = transform.InverseTransformPoint(body.position);
                if (p.z * FlowSign >= candidateProgress || LaneDistance(p) > 0.15f) continue;
                if ((body.position - candidate.position).sqrMagnitude < distanceSquared) return true;
            }
            return false;
        }

        // True when a loose body is inside the space the raised guard occupies (plus 2 cm).
        private bool GuardSpaceOccupied()
        {
            var guardCollider = mouthGuard.GetComponent<BoxCollider>();
            if (guardCollider == null) return false;
            Transform t = mouthGuard.transform;
            Vector3 centre = guardClosedPosition + t.rotation * Vector3.Scale(guardCollider.center, t.lossyScale);
            Vector3 half = Vector3.Scale(guardCollider.size, t.lossyScale) * 0.5f + Vector3.one * 0.02f;
            int n = Physics.OverlapBoxNonAlloc(centre, half, hits, t.rotation, ~0, QueryTriggerInteraction.Ignore);
            if (n == hits.Length) return true;
            for (int i = 0; i < n; i++)
            {
                var body = hits[i].attachedRigidbody;
                if (body != null && !body.isKinematic) return true;
            }
            return false;
        }

        // Watches matching boxes from the trigger window on; one that ends up past the retracted arm tip while still on
        // the belt was not diverted and is reported once as passed (the lane monitor counts it as circulating).
        private void WatchPassing()
        {
            Vector3 centre = transform.TransformPoint(new Vector3(beltCentreX, 0.25f, (triggerWindow.x + triggerWindow.y) * 0.5f * FlowSign));
            int n = Physics.OverlapBoxNonAlloc(centre, new Vector3(beltHalfWidth, 0.25f, (triggerWindow.y - triggerWindow.x) * 0.5f), hits, transform.rotation, ~0, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < n; i++)
            {
                var body = hits[i].attachedRigidbody;
                if (body == null || body.isKinematic) continue;
                var box = body.GetComponent<LogisticsBox>();
                if (box != null && box.HasReadResult && box.ReadLane == lane) watched.Add(body);
            }

            passedNow.Clear();
            watched.RemoveWhere(b => b == null);
            foreach (var body in watched)
            {
                if (body == current) continue;
                Vector3 p = transform.InverseTransformPoint(body.position);
                if (LaneDistance(p) > 0f) passedNow.Add(body);
                else if (p.z * FlowSign > Mathf.Max(0f, armLength * FlowSign) + 0.3f) passedNow.Add(body);
            }
            foreach (var body in passedNow)
            {
                watched.Remove(body);
                Vector3 p = transform.InverseTransformPoint(body.position);
                if (LaneDistance(p) <= 0f)
                {
                    if (linearPush)
                    {
                        string reason = pushBlockReasons.TryGetValue(body, out string r) ? r : "후보 구간 판정 없음";
                        Debug.Log($"[LaneDiverter {lane}] {body.name} 분기 못하고 통과: {reason}", this);
                    }
                    BoxPassed?.Invoke(body);
                }
                pushBlockReasons.Remove(body);
            }
            if (pushBlockReasons.Count > 64) pushBlockReasons.Clear();
        }

        private void Track(float dt)
        {
            elapsed += dt;
            Vector3 p = transform.InverseTransformPoint(current.position);
            if (LaneDistance(p) > handOffMargin)
            {
                var done = current;
                current = null;
                Diverted?.Invoke(done);
            }
            else if (p.z * FlowSign > Reach + 0.4f || elapsed > divertTimeout)
            {
                var failed = current;
                current = null;
                Debug.LogWarning($"[LaneDiverter {lane}] divert failed for {failed.name}", this);
                DivertFailed?.Invoke(failed);
            }
        }

        public void Play() => running = true;
        public void Stop() => running = false;

        public void ResetToInitialState()
        {
            current = null;
            pushPhase = 0;
            pushElapsed = 0f;
            pushReported = false;
            angle = 0f;
            guardOpen = 0f;
            sinceHandOff = float.MaxValue;
            watched.Clear();
            passedNow.Clear();
            pushBlockReasons.Clear();
            LastBlockReason = null;
            if (mouthGuard != null) mouthGuard.position = guardClosedPosition;
            if (arm != null && linearPush) arm.position = transform.TransformPoint(pushHomeLocal);
        }

#if UNITY_EDITOR
        private void OnDrawGizmosSelected()
        {
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.color = new Color(1f, 0.6f, 0.1f, 0.8f);
            Gizmos.DrawLine(Vector3.zero, Quaternion.Euler(0f, extendedAngle, 0f) * Vector3.forward * armLength);
            Gizmos.color = new Color(0.2f, 0.8f, 1f, 0.6f);
            Gizmos.DrawWireCube(new Vector3(beltCentreX, 0.1f, (triggerWindow.x + triggerWindow.y) * 0.5f * FlowSign),
                new Vector3(beltHalfWidth * 2f, 0.2f, triggerWindow.y - triggerWindow.x));
        }
#endif
    }
}
