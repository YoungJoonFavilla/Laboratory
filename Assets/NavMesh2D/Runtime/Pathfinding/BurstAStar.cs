using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// Burst 호환 A* 패스파인딩
    /// - NativeArray 기반 데이터
    /// - FunctionPointer로 managed 코드에서 호출 가능
    /// </summary>
    [BurstCompile]
    public static class BurstAStar
    {
        /// <summary>
        /// Burst 컴파일된 A* 경로 찾기
        /// </summary>
        /// <returns>경로 길이 (실패 시 -1)</returns>
        [BurstCompile]
        public static int FindPath(
            // NavMesh 데이터 (읽기 전용)
            in NativeArray<int> neighbors,           // [triIndex * 3 + edge]
            in NativeArray<long> edgeCentersX,       // [triIndex * 3 + edge]
            in NativeArray<long> edgeCentersY,       // [triIndex * 3 + edge]
            in NativeArray<long> edgePairDistances,  // [triIndex * 3 + pairIndex]
            in NativeArray<int> neighborEntryEdge,   // [triIndex * 3 + edge]
            int triangleCount,
            // 입력
            int startTri,
            int endTri,
            long startX, long startY,
            long endX, long endY,
            // 작업 배열 (재사용)
            ref NativeArray<long> gScore,
            ref NativeArray<int> cameFrom,
            ref NativeArray<int> lastEntryEdge,
            ref NativeArray<int> nodeGeneration,
            ref NativeArray<int> inClosedSet,
            ref int generation,
            // 힙 (재사용)
            ref NativeMinHeap heap,
            // 출력
            ref NativeArray<int> outPath,
            int maxPathLength)
        {
            if (startTri < 0 || endTri < 0)
                return -1;

            if (startTri == endTri)
            {
                outPath[0] = startTri;
                return 1;
            }

            // Generation 증가
            generation++;
            if (generation == int.MaxValue)
            {
                generation = 1;
                for (int i = 0; i < triangleCount; i++)
                {
                    nodeGeneration[i] = 0;
                    inClosedSet[i] = 0;
                }
            }

            heap.Clear();

            // 시작 노드 초기화
            long hStart = ChebyshevRaw(startX, startY, endX, endY);
            heap.Insert(startTri, 0L, hStart);
            gScore[startTri] = 0L;
            lastEntryEdge[startTri] = -1;
            nodeGeneration[startTri] = generation;
            cameFrom[startTri] = startTri;

            while (heap.Count > 0)
            {
                int currentTri = heap.ExtractMin();

                if (currentTri == endTri)
                {
                    // 경로 재구성
                    return ReconstructPath(ref cameFrom, startTri, endTri, ref outPath, maxPathLength);
                }

                inClosedSet[currentTri] = generation;
                int entryEdge = lastEntryEdge[currentTri];

                for (int exitEdge = 0; exitEdge < 3; exitEdge++)
                {
                    int idx = currentTri * 3 + exitEdge;
                    int neighbor = neighbors[idx];
                    if (neighbor < 0 || inClosedSet[neighbor] == generation)
                        continue;

                    // 에지 중심
                    long ecX = edgeCentersX[idx];
                    long ecY = edgeCentersY[idx];

                    // 이동 비용 계산
                    long moveCostRaw;
                    if (entryEdge < 0)
                    {
                        moveCostRaw = OctagonalDistanceRaw(startX, startY, ecX, ecY);
                    }
                    else
                    {
                        moveCostRaw = GetEdgePairDistanceRaw(in edgePairDistances, currentTri, entryEdge, exitEdge);
                    }

                    long tentativeG = gScore[currentTri] + moveCostRaw;

                    if (neighbor == endTri)
                    {
                        long finalLeg = OctagonalDistanceRaw(ecX, ecY, endX, endY);
                        tentativeG += finalLeg;
                    }

                    bool isFirstVisit = nodeGeneration[neighbor] != generation;
                    if (isFirstVisit || tentativeG < gScore[neighbor])
                    {
                        gScore[neighbor] = tentativeG;
                        cameFrom[neighbor] = currentTri;
                        lastEntryEdge[neighbor] = neighborEntryEdge[idx];
                        nodeGeneration[neighbor] = generation;

                        long hRaw = (neighbor == endTri) ? 0L : ChebyshevRaw(ecX, ecY, endX, endY);

                        if (heap.Contains(neighbor))
                        {
                            heap.UpdatePriority(neighbor, tentativeG, hRaw);
                        }
                        else
                        {
                            heap.Insert(neighbor, tentativeG, hRaw);
                        }
                    }
                }
            }

            return -1; // 경로 없음
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static long GetEdgePairDistanceRaw(in NativeArray<long> edgePairDistances, int triIndex, int edge1, int edge2)
        {
            int e1 = edge1 < edge2 ? edge1 : edge2;
            int e2 = edge1 < edge2 ? edge2 : edge1;

            int pairIndex;
            if (e1 == 0 && e2 == 1) pairIndex = 0;
            else if (e1 == 0 && e2 == 2) pairIndex = 1;
            else pairIndex = 2;

            return edgePairDistances[triIndex * 3 + pairIndex];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static long OctagonalDistanceRaw(long x1, long y1, long x2, long y2)
        {
            long dx = x2 - x1;
            long dy = y2 - y1;
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;

            long minD, maxD;
            if (dx < dy) { minD = dx; maxD = dy; }
            else { minD = dy; maxD = dx; }

            return maxD + ((minD * 27) >> 6);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static long ChebyshevRaw(long x1, long y1, long x2, long y2)
        {
            long dx = x2 - x1;
            long dy = y2 - y1;
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            return (dx > dy) ? dx : dy;
        }

        private static int ReconstructPath(ref NativeArray<int> cameFrom, int startTri, int endTri, ref NativeArray<int> outPath, int maxPathLength)
        {
            // 역순으로 경로 수집 (임시 스택 사용)
            int pathLength = 0;
            int current = endTri;

            // 먼저 길이 계산
            while (cameFrom[current] != current)
            {
                pathLength++;
                current = cameFrom[current];
                if (pathLength > maxPathLength) return -1; // 오버플로우 방지
            }
            pathLength++; // 시작 노드 포함

            if (pathLength > maxPathLength) return -1;

            // 역순으로 채우기
            current = endTri;
            int idx = pathLength - 1;
            while (cameFrom[current] != current)
            {
                outPath[idx--] = current;
                current = cameFrom[current];
            }
            outPath[0] = current; // 시작 노드

            return pathLength;
        }
    }

    /// <summary>
    /// Burst 호환 Min-Heap
    /// </summary>
    public struct NativeMinHeap : IDisposable
    {
        private struct HeapNode
        {
            public int TriangleIndex;
            public long FScore;
        }

        private NativeArray<HeapNode> _heap;
        private NativeArray<int> _heapPosition;
        private NativeArray<int> _positionGeneration;
        private int _heapCount;
        private int _generation;
        private int _capacity;

        public int Count => _heapCount;

        public NativeMinHeap(int capacity, Allocator allocator)
        {
            _capacity = capacity;
            _heap = new NativeArray<HeapNode>(capacity, allocator);
            _heapPosition = new NativeArray<int>(capacity, allocator);
            _positionGeneration = new NativeArray<int>(capacity, allocator);
            _heapCount = 0;
            _generation = 0;
        }

        public void Dispose()
        {
            if (_heap.IsCreated) _heap.Dispose();
            if (_heapPosition.IsCreated) _heapPosition.Dispose();
            if (_positionGeneration.IsCreated) _positionGeneration.Dispose();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            _heapCount = 0;
            _generation++;

            if (_generation == int.MaxValue)
            {
                _generation = 1;
                for (int i = 0; i < _capacity; i++)
                    _positionGeneration[i] = 0;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(int triangleIndex)
        {
            return _positionGeneration[triangleIndex] == _generation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Insert(int triangleIndex, long g, long h)
        {
            int index = _heapCount;
            _heap[index] = new HeapNode { TriangleIndex = triangleIndex, FScore = g + h };
            _heapPosition[triangleIndex] = index;
            _positionGeneration[triangleIndex] = _generation;
            _heapCount++;
            SiftUp(index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int ExtractMin()
        {
            int result = _heap[0].TriangleIndex;
            _heapCount--;

            if (_heapCount > 0)
            {
                _heap[0] = _heap[_heapCount];
                _heapPosition[_heap[0].TriangleIndex] = 0;
                SiftDown(0);
            }

            _positionGeneration[result] = 0;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdatePriority(int triangleIndex, long newG, long newH)
        {
            if (_positionGeneration[triangleIndex] != _generation)
                return;

            int index = _heapPosition[triangleIndex];
            long oldF = _heap[index].FScore;
            long newF = newG + newH;
            _heap[index] = new HeapNode { TriangleIndex = triangleIndex, FScore = newF };

            if (newF < oldF)
                SiftUp(index);
            else if (newF > oldF)
                SiftDown(index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SiftUp(int index)
        {
            HeapNode node = _heap[index];
            while (index > 0)
            {
                int parent = (index - 1) >> 1;
                if (node.FScore >= _heap[parent].FScore)
                    break;

                _heap[index] = _heap[parent];
                _heapPosition[_heap[index].TriangleIndex] = index;
                index = parent;
            }
            _heap[index] = node;
            _heapPosition[node.TriangleIndex] = index;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SiftDown(int index)
        {
            HeapNode node = _heap[index];

            while (true)
            {
                int smallest = index;
                long smallestF = node.FScore;
                int left = (index << 1) + 1;
                int right = (index << 1) + 2;

                if (left < _heapCount && _heap[left].FScore < smallestF)
                {
                    smallest = left;
                    smallestF = _heap[left].FScore;
                }

                if (right < _heapCount && _heap[right].FScore < smallestF)
                {
                    smallest = right;
                }

                if (smallest == index)
                    break;

                _heap[index] = _heap[smallest];
                _heapPosition[_heap[index].TriangleIndex] = index;
                index = smallest;
            }

            _heap[index] = node;
            _heapPosition[node.TriangleIndex] = index;
        }
    }

    /// <summary>
    /// Burst 컴파일된 Spatial Query
    /// </summary>
    [BurstCompile]
    public static class BurstSpatialQuery
    {
        /// <summary>
        /// 점이 포함된 삼각형 인덱스 찾기
        /// </summary>
        [BurstCompile]
        public static int FindTriangleContainingPoint(
            long pointX, long pointY,
            in NativeArray<int> gridData,
            in NativeArray<int> gridOffsets,
            in NativeArray<int> gridCounts,
            long gridMinX, long gridMinY,
            long cellWidth, long cellHeight,
            int gridResolution,
            in NativeArray<long> verticesX,
            in NativeArray<long> verticesY,
            in NativeArray<int> triangleVertexIndices,
            int triangleCount)
        {
            // 셀 좌표 계산
            long relX = pointX - gridMinX;
            long relY = pointY - gridMinY;

            if (relX < 0 || relY < 0)
                return -1;

            int cellX = (int)(relX / cellWidth);
            int cellY = (int)(relY / cellHeight);

            if (cellX < 0) cellX = 0;
            if (cellX >= gridResolution) cellX = gridResolution - 1;
            if (cellY < 0) cellY = 0;
            if (cellY >= gridResolution) cellY = gridResolution - 1;

            int cellIndex = cellY * gridResolution + cellX;

            // 해당 셀의 삼각형만 체크
            int offset = gridOffsets[cellIndex];
            int count = gridCounts[cellIndex];

            for (int i = 0; i < count; i++)
            {
                int triIndex = gridData[offset + i];
                if (TriangleContainsPoint(pointX, pointY, triIndex,
                    in verticesX, in verticesY, in triangleVertexIndices))
                {
                    return triIndex;
                }
            }

            return -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TriangleContainsPoint(
            long px, long py, int triIndex,
            in NativeArray<long> verticesX,
            in NativeArray<long> verticesY,
            in NativeArray<int> triangleVertexIndices)
        {
            int baseIdx = triIndex * 3;
            int v0 = triangleVertexIndices[baseIdx];
            int v1 = triangleVertexIndices[baseIdx + 1];
            int v2 = triangleVertexIndices[baseIdx + 2];

            long v0x = verticesX[v0], v0y = verticesY[v0];
            long v1x = verticesX[v1], v1y = verticesY[v1];
            long v2x = verticesX[v2], v2y = verticesY[v2];

            long d1 = SignRaw(px, py, v0x, v0y, v1x, v1y);
            long d2 = SignRaw(px, py, v1x, v1y, v2x, v2y);
            long d3 = SignRaw(px, py, v2x, v2y, v0x, v0y);

            bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

            return !(hasNeg && hasPos);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static long SignRaw(long p1x, long p1y, long p2x, long p2y, long p3x, long p3y)
        {
            // (p1 - p3) × (p2 - p3)
            // 오버플로우 방지를 위해 중간 결과 시프트
            long ax = p1x - p3x;
            long ay = p1y - p3y;
            long bx = p2x - p3x;
            long by = p2y - p3y;

            // 결과의 부호만 필요하므로 오버플로우 가능성이 있어도 부호는 유지됨
            return (ax >> 16) * (by >> 16) - (bx >> 16) * (ay >> 16);
        }
    }

    /// <summary>
    /// Burst 컴파일된 Funnel Algorithm
    /// </summary>
    [BurstCompile]
    public static class BurstFunnel
    {
        /// <summary>
        /// String Pull 알고리즘
        /// </summary>
        /// <returns>경로 점 개수</returns>
        [BurstCompile]
        public static int StringPull(
            long startX, long startY,
            long endX, long endY,
            in NativeArray<long> portalLeftX,
            in NativeArray<long> portalLeftY,
            in NativeArray<long> portalRightX,
            in NativeArray<long> portalRightY,
            int portalCount,
            ref NativeArray<long> outPathX,
            ref NativeArray<long> outPathY,
            int maxPathLength)
        {
            if (portalCount == 0)
            {
                outPathX[0] = startX;
                outPathY[0] = startY;
                outPathX[1] = endX;
                outPathY[1] = endY;
                return 2;
            }

            int pathCount = 0;

            // Funnel 초기화
            long apexX = startX, apexY = startY;
            long leftX = startX, leftY = startY;
            long rightX = startX, rightY = startY;
            int apexIndex = 0;
            int leftIndex = 0;
            int rightIndex = 0;

            // 시작점 추가
            outPathX[pathCount] = startX;
            outPathY[pathCount] = startY;
            pathCount++;

            for (int i = 1; i < portalCount && pathCount < maxPathLength - 1; i++)
            {
                long pLeftX = portalLeftX[i];
                long pLeftY = portalLeftY[i];
                long pRightX = portalRightX[i];
                long pRightY = portalRightY[i];

                // 오른쪽 경계 업데이트 시도
                if (TriArea2Raw(apexX, apexY, rightX, rightY, pRightX, pRightY) <= 0)
                {
                    if ((apexX == rightX && apexY == rightY) ||
                        TriArea2Raw(apexX, apexY, leftX, leftY, pRightX, pRightY) > 0)
                    {
                        rightX = pRightX;
                        rightY = pRightY;
                        rightIndex = i;
                    }
                    else
                    {
                        // 왼쪽을 새 apex로
                        if (pathCount == 0 || outPathX[pathCount - 1] != leftX || outPathY[pathCount - 1] != leftY)
                        {
                            outPathX[pathCount] = leftX;
                            outPathY[pathCount] = leftY;
                            pathCount++;
                        }

                        apexX = leftX;
                        apexY = leftY;
                        apexIndex = leftIndex;

                        leftX = apexX;
                        leftY = apexY;
                        rightX = apexX;
                        rightY = apexY;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        i = apexIndex;
                        continue;
                    }
                }

                // 왼쪽 경계 업데이트 시도
                if (TriArea2Raw(apexX, apexY, leftX, leftY, pLeftX, pLeftY) >= 0)
                {
                    if ((apexX == leftX && apexY == leftY) ||
                        TriArea2Raw(apexX, apexY, rightX, rightY, pLeftX, pLeftY) < 0)
                    {
                        leftX = pLeftX;
                        leftY = pLeftY;
                        leftIndex = i;
                    }
                    else
                    {
                        // 오른쪽을 새 apex로
                        if (pathCount == 0 || outPathX[pathCount - 1] != rightX || outPathY[pathCount - 1] != rightY)
                        {
                            outPathX[pathCount] = rightX;
                            outPathY[pathCount] = rightY;
                            pathCount++;
                        }

                        apexX = rightX;
                        apexY = rightY;
                        apexIndex = rightIndex;

                        leftX = apexX;
                        leftY = apexY;
                        rightX = apexX;
                        rightY = apexY;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        i = apexIndex;
                        continue;
                    }
                }
            }

            // 끝점 추가
            if (pathCount == 0 || outPathX[pathCount - 1] != endX || outPathY[pathCount - 1] != endY)
            {
                outPathX[pathCount] = endX;
                outPathY[pathCount] = endY;
                pathCount++;
            }

            return pathCount;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static long TriArea2Raw(long ax, long ay, long bx, long by, long cx, long cy)
        {
            long abx = bx - ax;
            long aby = by - ay;
            long acx = cx - ax;
            long acy = cy - ay;

            // 오버플로우 방지: 시프트 후 곱셈
            return ((acx >> 16) * (aby >> 16)) - ((abx >> 16) * (acy >> 16));
        }
    }
}
