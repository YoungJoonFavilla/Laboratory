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
        // Fixed64 Q31.32 포맷 상수
        private const int SHIFT = 32;
        private const long ONE_L = 4294967296L; // 1L << 32

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

            // 디버그용 타이밍 (ticks)
            public long TicksExtractMin;
            public long TicksGetTriangle;
            public long TicksMoveCost;
            public long TicksHeuristic;
            public long TicksHeapOps;
            public long TicksFindTriangle;
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

            _gScore = new long[triCount];
            _cameFrom = new int[triCount];
            _lastEntryEdge = new int[triCount];  // -1 = start point, 0-2 = entry edge
            _nodeGeneration = new int[triCount];
            _inClosedSet = new int[triCount];
        }

        private readonly IndexedMinHeap _openSet = new IndexedMinHeap();
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

            // 타이밍 변수
            long ticksExtractMin = 0, ticksGetTriangle = 0, ticksMoveCost = 0;
            long ticksHeuristic = 0, ticksHeapOps = 0, ticksFindTriangle = 0;
            long t0, t1;

            t0 = Stopwatch.GetTimestamp();
            int startTri = _navMesh.FindTriangleContainingPoint(start);
            int endTri = _navMesh.FindTriangleContainingPoint(end);

            if (startTri < 0)
                startTri = _navMesh.FindNearestTriangle(start);
            if (endTri < 0)
                endTri = _navMesh.FindNearestTriangle(end);
            t1 = Stopwatch.GetTimestamp();
            ticksFindTriangle += (t1 - t0);

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

            t0 = Stopwatch.GetTimestamp();
            long hStartRaw = DistanceRaw(startX, startY, endX, endY);
            t1 = Stopwatch.GetTimestamp();
            ticksHeuristic += (t1 - t0);

            t0 = Stopwatch.GetTimestamp();
            _openSet.Insert(startTri, 0L, hStartRaw);
            t1 = Stopwatch.GetTimestamp();
            ticksHeapOps += (t1 - t0);

            _gScore[startTri] = 0L;
            _lastEntryEdge[startTri] = -1;
            _nodeGeneration[startTri] = _generation;
            _cameFrom[startTri] = startTri;

            while (_openSet.Count > 0)
            {
                t0 = Stopwatch.GetTimestamp();
                var (currentTri, _, _) = _openSet.ExtractMin();
                t1 = Stopwatch.GetTimestamp();
                ticksExtractMin += (t1 - t0);
                exploredCount++;

                if (currentTri == endTri)
                {
                    result.Success = true;
                    result.ExploredCount = exploredCount;
                    result.TicksExtractMin = ticksExtractMin;
                    result.TicksGetTriangle = ticksGetTriangle;
                    result.TicksMoveCost = ticksMoveCost;
                    result.TicksHeuristic = ticksHeuristic;
                    result.TicksHeapOps = ticksHeapOps;
                    result.TicksFindTriangle = ticksFindTriangle;
                    ReconstructPath(result, currentTri);
                    CheckGenerationOverflow();
                    return result;
                }

                _inClosedSet[currentTri] = _generation;
                int entryEdge = _lastEntryEdge[currentTri];

                t0 = Stopwatch.GetTimestamp();
                var triangle = _navMesh.GetTriangle(currentTri);
                t1 = Stopwatch.GetTimestamp();
                ticksGetTriangle += (t1 - t0);

                for (int exitEdge = 0; exitEdge < 3; exitEdge++)
                {
                    int neighbor = triangle.GetNeighbor(exitEdge);
                    if (neighbor < 0 || _inClosedSet[neighbor] == _generation)
                        continue;

                    // 이동 비용 계산
                    t0 = Stopwatch.GetTimestamp();
                    long moveCostRaw;
                    if (entryEdge < 0)
                    {
                        _navMesh.GetEdgeCenterRaw(currentTri, exitEdge, out long ecX, out long ecY);
                        moveCostRaw = DistanceRaw(startX, startY, ecX, ecY);
                    }
                    else
                    {
                        moveCostRaw = _navMesh.GetEdgePairDistanceRaw(currentTri, entryEdge, exitEdge);
                    }
                    long tentativeGRaw = _gScore[currentTri] + moveCostRaw;

                    if (neighbor == endTri)
                    {
                        _navMesh.GetEdgeCenterRaw(currentTri, exitEdge, out long ecX, out long ecY);
                        long finalLegRaw = DistanceRaw(ecX, ecY, endX, endY);
                        tentativeGRaw += finalLegRaw;
                    }
                    t1 = Stopwatch.GetTimestamp();
                    ticksMoveCost += (t1 - t0);

                    bool isFirstVisit = _nodeGeneration[neighbor] != _generation;
                    if (isFirstVisit || tentativeGRaw < _gScore[neighbor])
                    {
                        _gScore[neighbor] = tentativeGRaw;
                        _cameFrom[neighbor] = currentTri;
                        _lastEntryEdge[neighbor] = _navMesh.GetNeighborEntryEdge(currentTri, exitEdge);
                        _nodeGeneration[neighbor] = _generation;

                        // 휴리스틱
                        t0 = Stopwatch.GetTimestamp();
                        long hRaw;
                        if (neighbor == endTri)
                        {
                            hRaw = 0L;
                        }
                        else
                        {
                            _navMesh.GetEdgeCenterRaw(currentTri, exitEdge, out long ecX, out long ecY);
                            hRaw = DistanceRaw(ecX, ecY, endX, endY);
                        }
                        t1 = Stopwatch.GetTimestamp();
                        ticksHeuristic += (t1 - t0);

                        t0 = Stopwatch.GetTimestamp();
                        if (_openSet.Contains(neighbor))
                        {
                            _openSet.UpdatePriority(neighbor, tentativeGRaw, hRaw);
                        }
                        else
                        {
                            _openSet.Insert(neighbor, tentativeGRaw, hRaw);
                        }
                        t1 = Stopwatch.GetTimestamp();
                        ticksHeapOps += (t1 - t0);
                    }
                }
            }

            result.ExploredCount = exploredCount;
            result.TicksExtractMin = ticksExtractMin;
            result.TicksGetTriangle = ticksGetTriangle;
            result.TicksMoveCost = ticksMoveCost;
            result.TicksHeuristic = ticksHeuristic;
            result.TicksHeapOps = ticksHeapOps;
            result.TicksFindTriangle = ticksFindTriangle;
            CheckGenerationOverflow();
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long DistanceRaw(long x1, long y1, long x2, long y2)
        {
            long dx = x2 - x1;
            long dy = y2 - y1;
            long sqrDistRaw = MulRaw(dx, dx) + MulRaw(dy, dy);
            return SqrtRaw(sqrDistRaw);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long MulRaw(long a, long b)
        {
            bool negA = a < 0;
            bool negB = b < 0;
            if (negA) a = -a;
            if (negB) b = -b;

            ulong aLo = (ulong)a & 0xFFFFFFFF;
            ulong aHi = (ulong)a >> 32;
            ulong bLo = (ulong)b & 0xFFFFFFFF;
            ulong bHi = (ulong)b >> 32;

            ulong loLo = aLo * bLo;
            ulong loHi = aLo * bHi;
            ulong hiLo = aHi * bLo;
            ulong hiHi = aHi * bHi;

            ulong mid = loHi + hiLo + (loLo >> 32);
            ulong result = hiHi + (mid >> 32);
            result = (result << 32) | (mid & 0xFFFFFFFF);

            if ((loLo & 0x80000000) != 0)
                result++;

            long signedResult = (long)result;
            return (negA != negB) ? -signedResult : signedResult;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long SqrtRaw(long x)
        {
            if (x <= 0) return 0;

            ulong num = (ulong)x;
            ulong result = 0UL;
            ulong bit = 1UL << 62;

            while (bit > num)
                bit >>= 2;

            while (bit != 0)
            {
                if (num >= result + bit)
                {
                    num -= result + bit;
                    result = (result >> 1) + bit;
                }
                else
                {
                    result >>= 1;
                }
                bit >>= 2;
            }

            if (num > ((1UL << SHIFT) - 1))
            {
                num -= result;
                num = (num << SHIFT) - 0x80000000UL;
                result = (result << SHIFT) + 0x80000000UL;
            }
            else
            {
                num <<= SHIFT;
                result <<= SHIFT;
            }

            bit = 1UL << (SHIFT - 2);
            while (bit != 0)
            {
                if (num >= result + bit)
                {
                    num -= result + bit;
                    result = (result >> 1) + bit;
                }
                else
                {
                    result >>= 1;
                }
                bit >>= 2;
            }

            if (num > result && (num - result) > (result >> 1))
                result++;

            return (long)result;
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
