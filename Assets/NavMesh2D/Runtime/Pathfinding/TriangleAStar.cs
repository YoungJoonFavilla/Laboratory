using System.Collections.Generic;
using FixedMathSharp;
using NavMesh2D.Geometry;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// 삼각형 그래프에서의 A* 경로 찾기
    /// 삼각형 중심점을 노드로, 인접 삼각형을 에지로 사용
    /// </summary>
    public class TriangleAStar
    {
        private NavMesh2DData _navMesh;

        // 경로 찾기 결과
        public struct PathResult
        {
            /// <summary>
            /// 경로 찾기 성공 여부
            /// </summary>
            public bool Success;

            /// <summary>
            /// 경로상의 삼각형 인덱스들 (시작 → 끝)
            /// </summary>
            public List<int> TrianglePath;

            /// <summary>
            /// 경로상의 포탈들 (에지 양 끝점)
            /// </summary>
            public List<Portal> Portals;

            /// <summary>
            /// 시작점
            /// </summary>
            public Vector2Fixed Start;

            /// <summary>
            /// 끝점
            /// </summary>
            public Vector2Fixed End;
        }

        /// <summary>
        /// 포탈 (두 삼각형 사이의 공유 에지)
        /// </summary>
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

            // 배열 초기화
            _gScore = new Fixed64[triCount];
            _cameFrom = new int[triCount];
            _lastEdgeCenter = new Vector2Fixed[triCount];
            _nodeGeneration = new int[triCount];
            _inClosedSet = new int[triCount];
        }

        // 재사용 컬렉션
        private readonly IndexedMinHeap _openSet = new IndexedMinHeap();
        private readonly List<int> _pathBuffer = new List<int>();

        // Generation 기반 배열 (Dictionary 대체)
        private int _generation = 0;
        private Fixed64[] _gScore;
        private int[] _cameFrom;
        private Vector2Fixed[] _lastEdgeCenter;
        private int[] _nodeGeneration;  // 노드가 마지막으로 방문된 generation
        private int[] _inClosedSet;     // closedSet에 추가된 generation

        /// <summary>
        /// A* 경로 찾기 수행
        /// </summary>
        /// <param name="start">시작점</param>
        /// <param name="end">끝점</param>
        /// <returns>경로 찾기 결과</returns>
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

            // 1. 시작점과 끝점이 포함된 삼각형 찾기
            int startTri = _navMesh.FindTriangleContainingPoint(start);
            int endTri = _navMesh.FindTriangleContainingPoint(end);

            // 포인트가 NavMesh 내에 없으면 가장 가까운 삼각형 사용
            if (startTri < 0)
                startTri = _navMesh.FindNearestTriangle(start);
            if (endTri < 0)
                endTri = _navMesh.FindNearestTriangle(end);

            // 여전히 찾을 수 없으면 실패
            if (startTri < 0 || endTri < 0)
                return result;

            // 같은 삼각형 내에 있으면 바로 성공
            if (startTri == endTri)
            {
                result.Success = true;
                result.TrianglePath.Add(startTri);
                return result;
            }

            // Generation 증가 (모든 이전 데이터 무효화)
            _generation++;
            _openSet.Clear();

            // 2. A* 알고리즘 실행
            // 시작점에서 끝점까지의 휴리스틱 (실제 점 사용)
            Fixed64 hStart = Vector2Fixed.Distance(start, end);
            _openSet.Insert(startTri, Fixed64.Zero, hStart);

            // 시작 노드 초기화
            _gScore[startTri] = Fixed64.Zero;
            _lastEdgeCenter[startTri] = start;
            _nodeGeneration[startTri] = _generation;
            _cameFrom[startTri] = startTri; // 자기 자신을 가리켜서 시작점임을 표시

            while (_openSet.Count > 0)
            {
                // F가 가장 낮은 노드 추출 - O(log n)
                var (currentTri, _, _) = _openSet.ExtractMin();

                // 목표 도달
                if (currentTri == endTri)
                {
                    result.Success = true;
                    ReconstructPath(result, currentTri);
                    CheckGenerationOverflow();
                    return result;
                }

                // ClosedSet에 추가 (generation 기반)
                _inClosedSet[currentTri] = _generation;

                // 현재 위치 (시작점이면 실제 시작점, 아니면 진입한 에지 중심점)
                Vector2Fixed currentPos = _lastEdgeCenter[currentTri];

                // 인접 삼각형들 탐색
                var triangle = _navMesh.GetTriangle(currentTri);
                for (int edge = 0; edge < 3; edge++)
                {
                    int neighbor = triangle.GetNeighbor(edge);
                    if (neighbor < 0 || _inClosedSet[neighbor] == _generation)
                        continue;

                    // 에지 중심점으로 이동 비용 계산 (실제 현재 위치에서)
                    Vector2Fixed edgeCenter = _navMesh.GetEdgeCenter(currentTri, edge);
                    Fixed64 moveCost = Vector2Fixed.Distance(currentPos, edgeCenter);
                    Fixed64 tentativeG = _gScore[currentTri] + moveCost;

                    // 도착 삼각형이면 끝점까지의 최종 비용도 G에 포함
                    if (neighbor == endTri)
                    {
                        Fixed64 finalLeg = Vector2Fixed.Distance(edgeCenter, end);
                        tentativeG += finalLeg;
                    }

                    // 이번 generation에서 처음 방문하거나, 더 좋은 경로인 경우
                    bool isFirstVisit = _nodeGeneration[neighbor] != _generation;
                    if (isFirstVisit || tentativeG < _gScore[neighbor])
                    {
                        _gScore[neighbor] = tentativeG;
                        _cameFrom[neighbor] = currentTri;
                        _lastEdgeCenter[neighbor] = edgeCenter;
                        _nodeGeneration[neighbor] = _generation;

                        // 휴리스틱: 에지 중심점에서 실제 끝점까지 (도착 삼각형이면 이미 G에 포함했으므로 0)
                        Fixed64 h = (neighbor == endTri) ? Fixed64.Zero : Vector2Fixed.Distance(edgeCenter, end);

                        // openSet에 이미 있는지 확인하고 업데이트 또는 추가 - O(1) + O(log n)
                        if (_openSet.Contains(neighbor))
                        {
                            _openSet.UpdatePriority(neighbor, tentativeG, h);
                        }
                        else
                        {
                            _openSet.Insert(neighbor, tentativeG, h);
                        }
                    }
                }
            }

            // 경로를 찾지 못함
            CheckGenerationOverflow();
            return result;
        }

        /// <summary>
        /// Generation 오버플로우 체크 및 리셋
        /// </summary>
        private void CheckGenerationOverflow()
        {
            if (_generation == int.MaxValue)
            {
                _generation = 0;
                System.Array.Clear(_nodeGeneration, 0, _nodeGeneration.Length);
                System.Array.Clear(_inClosedSet, 0, _inClosedSet.Length);
            }
        }

        /// <summary>
        /// 경로 재구성 (역추적)
        /// </summary>
        private void ReconstructPath(PathResult result, int endTri)
        {
            _pathBuffer.Clear();
            int current = endTri;

            // _cameFrom이 유효한 동안 역추적 (시작 노드는 자기 자신을 가리킴)
            while (_cameFrom[current] != current)
            {
                _pathBuffer.Add(current);
                current = _cameFrom[current];
            }
            _pathBuffer.Add(current); // 시작 삼각형

            // 역순을 정순으로 result에 추가
            for (int i = _pathBuffer.Count - 1; i >= 0; i--)
            {
                result.TrianglePath.Add(_pathBuffer[i]);
            }

            // 포탈 생성 (연속된 삼각형들 사이의 공유 에지)
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
