using System;
using System.Collections.Generic;
using UnityEngine;

namespace Logistics
{
    // AMR 경로 탐색: 바닥 평면 격자(기본 0.1 m)에서 A*로 출발→목표 경로를 찾는다. (FR-14)
    // 칸 점유는 칸을 차체 반폭만큼 넓힌 상자를 차체 높이 범위(바닥면 위)에서 Physics로 조회해 판단하고, 처음 조회할 때 한 번 계산해 둔다.
    // 무엇을 장애물로 볼지는 호출한 쪽 판정 함수(isObstacle)가 정한다 (자기 차체·운반물·바닥면 제외).
    // 찾은 칸 경로는 격자와 차체 휩쓸기 검사(segmentClear)가 모두 통과하는 구간을 묶어 경유점으로 줄인다.
    // 경로가 없거나 탐색 범위를 넘으면 실패 이유를 돌려준다 (도달 불가). 설계의 NavMeshAgent 대신 쓰는 확장 클래스다.
    public sealed class ForkliftPathPlanner
    {
        public struct Settings
        {
            public float HalfWidth;     // 차체·화물 반폭 + 여유 (칸 넓힘)
            public float BandBottom;    // 점유 검사 높이 범위 (월드 Y)
            public float BandTop;
            public float SearchMargin;  // 출발·목표를 감싼 사각형 바깥 탐색 여유 (m)
            public int MaxCells;        // 격자 칸 수 상한
        }

        private const sbyte Unknown = 0, Free = 1, Blocked = 2;
        private readonly float cell;
        private readonly Func<Collider, bool> isObstacle;
        private Collider[] buffer = new Collider[64];

        private Vector2 origin;
        private int width, height;
        private sbyte[] grid;
        private Settings settings;

        public int CheckedCells { get; private set; }   // 마지막 탐색에서 점유를 조회한 칸 수
        public int ExpandedCells { get; private set; }  // 마지막 탐색에서 펼친 칸 수

        public ForkliftPathPlanner(float cellSize, Func<Collider, bool> isObstacle)
        {
            cell = Mathf.Max(0.02f, cellSize);
            this.isObstacle = isObstacle;
        }

        // 출발→목표 경유점(출발점 제외, 목표 포함). blockedPoints는 이전 시도에서 회전 공간이 없던 지점 (그 칸을 막는다)
        public bool TryPlan(Vector3 start, Vector3 goal, Settings s, Func<Vector3, Vector3, bool> segmentClear,
            IList<Vector3> blockedPoints, List<Vector3> waypoints, out string reason)
        {
            waypoints.Clear();
            settings = s;
            CheckedCells = 0;
            ExpandedCells = 0;
            Vector2 min = Vector2.Min(Flat(start), Flat(goal)) - Vector2.one * s.SearchMargin;
            Vector2 max = Vector2.Max(Flat(start), Flat(goal)) + Vector2.one * s.SearchMargin;
            width = Mathf.CeilToInt((max.x - min.x) / cell);
            height = Mathf.CeilToInt((max.y - min.y) / cell);
            if (width * height > s.MaxCells)
            {
                reason = $"탐색 범위 초과 ({width}×{height}칸)";
                return false;
            }
            origin = min;
            grid = new sbyte[width * height];
            int startIndex = Index(start), goalIndex = Index(goal);
            if (blockedPoints != null)
            {
                foreach (Vector3 p in blockedPoints)
                {
                    int i = Index(p);
                    if (i >= 0) grid[i] = Blocked;
                }
            }
            grid[startIndex] = Free;   // 지금 서 있는 칸은 자기 차체 자리
            if (IsBlocked(goalIndex, out string goalHit))
            {
                reason = $"목표 지점 점유 ({goalHit})";
                return false;
            }

            //1. A* (8방향, 모서리 가로지르기 금지)
            var cameFrom = new int[grid.Length];
            var cost = new float[grid.Length];
            var closed = new bool[grid.Length];
            for (int i = 0; i < cost.Length; i++) { cost[i] = float.MaxValue; cameFrom[i] = -1; }
            var open = new MinHeap();
            cost[startIndex] = 0f;
            open.Push(startIndex, Heuristic(startIndex, goalIndex));
            bool found = false;
            while (open.Count > 0)
            {
                int current = open.Pop();
                if (closed[current]) continue;
                if (current == goalIndex) { found = true; break; }
                closed[current] = true;
                ExpandedCells++;
                int cx = current % width, cy = current / width;
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (dx == 0 && dy == 0) continue;
                        int nx = cx + dx, ny = cy + dy;
                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                        int next = ny * width + nx;
                        if (closed[next] || IsBlocked(next, out _)) continue;
                        if (dx != 0 && dy != 0 && (IsBlocked(cy * width + nx, out _) || IsBlocked(ny * width + cx, out _))) continue;
                        float step = (dx != 0 && dy != 0) ? 1.41421356f : 1f;
                        float g = cost[current] + step;
                        if (g >= cost[next]) continue;
                        cost[next] = g;
                        cameFrom[next] = current;
                        open.Push(next, g + Heuristic(next, goalIndex));
                    }
                }
            }
            if (!found)
            {
                reason = $"목표까지 이어진 통로 없음 (조회 {CheckedCells}칸)";
                return false;
            }

            //2. 칸 경로 → 경유점 (격자·휩쓸기 검사를 통과하는 가장 먼 칸까지 묶음)
            var cells = new List<int>();
            for (int i = goalIndex; i != -1; i = cameFrom[i]) cells.Add(i);
            cells.Reverse();
            Vector3 anchor = start;
            for (int k = 1; k < cells.Count; k++)
            {
                Vector3 candidate = k == cells.Count - 1 ? goal : Centre(cells[k], start.y);
                if (LineFree(anchor, candidate) && (segmentClear == null || segmentClear(anchor, candidate)))
                {
                    continue;
                }
                Vector3 previous = Centre(cells[k - 1], start.y);
                if (k - 1 == 0 || (previous - anchor).sqrMagnitude < 1e-6f)
                {
                    // 바로 다음 칸도 휩쓸기 검사를 통과하지 못함: 차체가 지나갈 수 없는 칸 경로
                    reason = $"차체·화물이 지나갈 폭 부족 ({Flat(candidate)})";
                    return false;
                }
                waypoints.Add(previous);
                anchor = previous;
                k--;
            }
            waypoints.Add(goal);
            reason = null;
            return true;
        }

        // 격자 칸들로만 본 직선 통과 여부 (반 칸 간격 표본)
        private bool LineFree(Vector3 a, Vector3 b)
        {
            float length = Vector2.Distance(Flat(a), Flat(b));
            int steps = Mathf.Max(1, Mathf.CeilToInt(length / (cell * 0.5f)));
            for (int i = 0; i <= steps; i++)
            {
                int index = Index(Vector3.Lerp(a, b, (float)i / steps));
                if (index < 0 || (index != Index(a) && IsBlocked(index, out _))) return false;
            }
            return true;
        }

        private bool IsBlocked(int index, out string hit)
        {
            hit = null;
            if (grid[index] == Unknown)
            {
                CheckedCells++;
                Vector3 centre = Centre(index, (settings.BandBottom + settings.BandTop) * 0.5f);
                Vector3 half = new Vector3(cell * 0.5f + settings.HalfWidth, (settings.BandTop - settings.BandBottom) * 0.5f, cell * 0.5f + settings.HalfWidth);
                int n;
                while ((n = Physics.OverlapBoxNonAlloc(centre, half, buffer, Quaternion.identity, ~0, QueryTriggerInteraction.Ignore)) == buffer.Length)
                {
                    buffer = new Collider[buffer.Length * 2];
                }
                grid[index] = Free;
                for (int i = 0; i < n; i++)
                {
                    if (isObstacle(buffer[i]))
                    {
                        grid[index] = Blocked;
                        hit = buffer[i].name;
                        break;
                    }
                }
            }
            return grid[index] == Blocked;
        }

        private int Index(Vector3 p)
        {
            int x = Mathf.FloorToInt((p.x - origin.x) / cell), y = Mathf.FloorToInt((p.z - origin.y) / cell);
            return x < 0 || y < 0 || x >= width || y >= height ? -1 : y * width + x;
        }

        private Vector3 Centre(int index, float y) => new Vector3(origin.x + (index % width + 0.5f) * cell, y, origin.y + (index / width + 0.5f) * cell);

        private float Heuristic(int a, int b)
        {
            int dx = Mathf.Abs(a % width - b % width), dy = Mathf.Abs(a / width - b / width);
            return Mathf.Max(dx, dy) + 0.41421356f * Mathf.Min(dx, dy);
        }

        private static Vector2 Flat(Vector3 v) => new Vector2(v.x, v.z);

        // 우선순위 큐 (작은 값 먼저)
        private sealed class MinHeap
        {
            private readonly List<(float priority, int item)> items = new List<(float, int)>();
            public int Count => items.Count;

            public void Push(int item, float priority)
            {
                items.Add((priority, item));
                int i = items.Count - 1;
                while (i > 0)
                {
                    int parent = (i - 1) / 2;
                    if (items[parent].priority <= items[i].priority) break;
                    (items[parent], items[i]) = (items[i], items[parent]);
                    i = parent;
                }
            }

            public int Pop()
            {
                int top = items[0].item;
                int last = items.Count - 1;
                items[0] = items[last];
                items.RemoveAt(last);
                int i = 0;
                while (true)
                {
                    int l = i * 2 + 1, r = l + 1, smallest = i;
                    if (l < items.Count && items[l].priority < items[smallest].priority) smallest = l;
                    if (r < items.Count && items[r].priority < items[smallest].priority) smallest = r;
                    if (smallest == i) break;
                    (items[smallest], items[i]) = (items[i], items[smallest]);
                    i = smallest;
                }
                return top;
            }
        }
    }
}
