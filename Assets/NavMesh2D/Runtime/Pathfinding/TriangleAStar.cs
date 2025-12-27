using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using FixedMathSharp;
using NavMesh2D.Geometry;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// 삼각형 그래프에서의 A* 경로 찾기
    /// 삼각형 중심점을 노드로, 인접 삼각형을 에지로 사용
    ///
    /// 최적화: Fixed64 대신 raw long 연산 + 사전계산 거리 사용
    /// </summary>
    public class TriangleAStar
    {
        private NavMesh2DData _navMesh;

        // 경로 찾기 결과
        public struct PathResult
        {
            public bool Success;
            public List<int> TrianglePath;
            public List<Portal> Portals;
            public Vector2Fixed Start;
            public Vector2Fixed End;
            public int ExploredCount;

#if NAVMESH_PROFILING
            // 디버그용 타이밍 (ticks)
            public long TicksExtractMin;
            public long TicksGetTriangle;
            public long TicksMoveCost;
            public long TicksHeuristic;
            public long TicksHeapOps;
            public long TicksFindTriangle;
#endif
        }

        public struct Portal
        {
            public Vector2Fixed Left;
            public Vector2Fixed Right;

            public Portal(Vector2Fixed left, Vector2Fixed right)
            {
                Left = left;
                Right = right;
            }
        }

        public TriangleAStar(NavMesh2DData navMesh)
        {
            _navMesh = navMesh;
            int triCount = navMesh.TriangleCount;

            _openSet = new IndexedMinHeap(triCount);
            _gScore = new long[triCount];
            _cameFrom = new int[triCount];
            _lastEntryEdge = new int[triCount];  // -1 = start point, 0-2 = entry edge
            _nodeGeneration = new int[triCount];
            _inClosedSet = new int[triCount];
        }

        private IndexedMinHeap _openSet;
        private readonly List<int> _pathBuffer = new List<int>();

        private int _generation = 0;
        private long[] _gScore;
        private int[] _cameFrom;
        private int[] _lastEntryEdge;  // 진입한 에지 인덱스 (-1 = 시작점)
        private int[] _nodeGeneration;
        private int[] _inClosedSet;

        public PathResult FindPath(Vector2Fixed start, Vector2Fixed end)
        {
            PathResult result = new PathResult
            {
                Success = false,
                TrianglePath = new List<int>(),
                Portals = new List<Portal>(),
                Start = start,
                End = end
            };

#if NAVMESH_PROFILING
            long ticksExtractMin = 0, ticksGetTriangle = 0, ticksMoveCost = 0;
            long ticksHeuristic = 0, ticksHeapOps = 0, ticksFindTriangle = 0;
            long t0, t1;
            t0 = Stopwatch.GetTimestamp();
#endif

            int startTri = _navMesh.FindTriangleContainingPoint(start);
            int endTri = _navMesh.FindTriangleContainingPoint(end);

            if (startTri < 0)
                startTri = _navMesh.FindNearestTriangle(start);
            if (endTri < 0)
                endTri = _navMesh.FindNearestTriangle(end);

#if NAVMESH_PROFILING
            t1 = Stopwatch.GetTimestamp();
            ticksFindTriangle += (t1 - t0);
#endif

            if (startTri < 0 || endTri < 0)
                return result;

            if (startTri == endTri)
            {
                result.Success = true;
                result.TrianglePath.Add(startTri);
                return result;
            }

            long startX = start.x.m_rawValue;
            long startY = start.y.m_rawValue;
            long endX = end.x.m_rawValue;
            long endY = end.y.m_rawValue;

            _generation++;
            _openSet.Clear();
            int exploredCount = 0;

#if NAVMESH_PROFILING
            t0 = Stopwatch.GetTimestamp();
#endif
            long hStartRaw = ChebyshevRaw(startX, startY, endX, endY);
#if NAVMESH_PROFILING
            t1 = Stopwatch.GetTimestamp();
            ticksHeuristic += (t1 - t0);
            t0 = Stopwatch.GetTimestamp();
#endif
            _openSet.Insert(startTri, 0L, hStartRaw);
#if NAVMESH_PROFILING
            t1 = Stopwatch.GetTimestamp();
            ticksHeapOps += (t1 - t0);
#endif

            _gScore[startTri] = 0L;
            _lastEntryEdge[startTri] = -1;
            _nodeGeneration[startTri] = _generation;
            _cameFrom[startTri] = startTri;

            while (_openSet.Count > 0)
            {
#if NAVMESH_PROFILING
                t0 = Stopwatch.GetTimestamp();
#endif
                _openSet.ExtractMin(out int currentTri);
#if NAVMESH_PROFILING
                t1 = Stopwatch.GetTimestamp();
                ticksExtractMin += (t1 - t0);
#endif
                exploredCount++;

                if (currentTri == endTri)
                {
                    result.Success = true;
                    result.ExploredCount = exploredCount;
#if NAVMESH_PROFILING
                    result.TicksExtractMin = ticksExtractMin;
                    result.TicksGetTriangle = ticksGetTriangle;
                    result.TicksMoveCost = ticksMoveCost;
                    result.TicksHeuristic = ticksHeuristic;
                    result.TicksHeapOps = ticksHeapOps;
                    result.TicksFindTriangle = ticksFindTriangle;
#endif
                    ReconstructPath(result, currentTri);
                    CheckGenerationOverflow();
                    return result;
                }

                _inClosedSet[currentTri] = _generation;
                int entryEdge = _lastEntryEdge[currentTri];

#if NAVMESH_PROFILING
                t0 = Stopwatch.GetTimestamp();
#endif
                var triangle = _navMesh.GetTriangle(currentTri);
#if NAVMESH_PROFILING
                t1 = Stopwatch.GetTimestamp();
                ticksGetTriangle += (t1 - t0);
#endif

                for (int exitEdge = 0; exitEdge < 3; exitEdge++)
                {
                    int neighbor = triangle.GetNeighbor(exitEdge);
                    if (neighbor < 0 || _inClosedSet[neighbor] == _generation)
                        continue;

                    // 에지 중심 한 번만 계산 (Move/Heuristic 공용)
                    _navMesh.GetEdgeCenterRaw(currentTri, exitEdge, out long ecX, out long ecY);

                    // 이동 비용 계산
#if NAVMESH_PROFILING
                    t0 = Stopwatch.GetTimestamp();
#endif
                    long moveCostRaw;
                    if (entryEdge < 0)
                    {
                        moveCostRaw = DistanceRaw(startX, startY, ecX, ecY);
                    }
                    else
                    {
                        moveCostRaw = _navMesh.GetEdgePairDistanceRaw(currentTri, entryEdge, exitEdge);
                    }
                    long tentativeGRaw = _gScore[currentTri] + moveCostRaw;

                    if (neighbor == endTri)
                    {
                        long finalLegRaw = DistanceRaw(ecX, ecY, endX, endY);
                        tentativeGRaw += finalLegRaw;
                    }
#if NAVMESH_PROFILING
                    t1 = Stopwatch.GetTimestamp();
                    ticksMoveCost += (t1 - t0);
#endif

                    bool isFirstVisit = _nodeGeneration[neighbor] != _generation;
                    if (isFirstVisit || tentativeGRaw < _gScore[neighbor])
                    {
                        _gScore[neighbor] = tentativeGRaw;
                        _cameFrom[neighbor] = currentTri;
                        _lastEntryEdge[neighbor] = _navMesh.GetNeighborEntryEdge(currentTri, exitEdge);
                        _nodeGeneration[neighbor] = _generation;

                        // 휴리스틱 (Chebyshev - sqrt 없음) - ecX, ecY 재사용
#if NAVMESH_PROFILING
                        t0 = Stopwatch.GetTimestamp();
#endif
                        long hRaw = (neighbor == endTri) ? 0L : ChebyshevRaw(ecX, ecY, endX, endY);
#if NAVMESH_PROFILING
                        t1 = Stopwatch.GetTimestamp();
                        ticksHeuristic += (t1 - t0);
                        t0 = Stopwatch.GetTimestamp();
#endif
                        if (_openSet.Contains(neighbor))
                        {
                            _openSet.UpdatePriority(neighbor, tentativeGRaw, hRaw);
                        }
                        else
                        {
                            _openSet.Insert(neighbor, tentativeGRaw, hRaw);
                        }
#if NAVMESH_PROFILING
                        t1 = Stopwatch.GetTimestamp();
                        ticksHeapOps += (t1 - t0);
#endif
                    }
                }
            }

            result.ExploredCount = exploredCount;
#if NAVMESH_PROFILING
            result.TicksExtractMin = ticksExtractMin;
            result.TicksGetTriangle = ticksGetTriangle;
            result.TicksMoveCost = ticksMoveCost;
            result.TicksHeuristic = ticksHeuristic;
            result.TicksHeapOps = ticksHeapOps;
            result.TicksFindTriangle = ticksFindTriangle;
#endif
            CheckGenerationOverflow();
            return result;
        }

        /// <summary>
        /// Octagonal Distance 근사 (sqrt 없음, ~97% 정확도)
        /// dist ≈ max(|dx|, |dy|) + 0.41421356 * min(|dx|, |dy|)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long DistanceRaw(long x1, long y1, long x2, long y2)
        {
            long dx = x2 - x1;
            long dy = y2 - y1;
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;

            long minD, maxD;
            if (dx < dy) { minD = dx; maxD = dy; }
            else { minD = dy; maxD = dx; }

            // 0.41421356 ≈ 27/64 = 0.421875 (98.3% accurate)
            return maxD + ((minD * 27) >> 6);
        }

        /// <summary>
        /// Chebyshev 거리 (sqrt 없음) - 휴리스틱용
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long ChebyshevRaw(long x1, long y1, long x2, long y2)
        {
            long dx = x2 - x1;
            long dy = y2 - y1;
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            return (dx > dy) ? dx : dy;
        }

        private void CheckGenerationOverflow()
        {
            if (_generation == int.MaxValue)
            {
                _generation = 0;
                System.Array.Clear(_nodeGeneration, 0, _nodeGeneration.Length);
                System.Array.Clear(_inClosedSet, 0, _inClosedSet.Length);
            }
        }

        private void ReconstructPath(PathResult result, int endTri)
        {
            _pathBuffer.Clear();
            int current = endTri;

            while (_cameFrom[current] != current)
            {
                _pathBuffer.Add(current);
                current = _cameFrom[current];
            }
            _pathBuffer.Add(current);

            for (int i = _pathBuffer.Count - 1; i >= 0; i--)
            {
                result.TrianglePath.Add(_pathBuffer[i]);
            }

            for (int i = 0; i < result.TrianglePath.Count - 1; i++)
            {
                int triA = result.TrianglePath[i];
                int triB = result.TrianglePath[i + 1];

                var portal = _navMesh.GetPortalBetween(triA, triB);
                if (portal.HasValue)
                {
                    result.Portals.Add(new Portal(portal.Value.Left, portal.Value.Right));
                }
            }
        }
    }
}
