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

        // A* 노드 (struct로 GC 최소화)
        private struct AStarNode
        {
            public int TriangleIndex;
            public Fixed64 GScore;
            public Fixed64 HScore;
            public Fixed64 FScore => GScore + HScore;

            public AStarNode(int triangleIndex, Fixed64 g, Fixed64 h)
            {
                TriangleIndex = triangleIndex;
                GScore = g;
                HScore = h;
            }
        }

        public TriangleAStar(NavMesh2DData navMesh)
        {
            _navMesh = navMesh;
        }

        // 재사용 컬렉션
        private readonly List<AStarNode> _openSet = new List<AStarNode>();
        private readonly HashSet<int> _closedSet = new HashSet<int>();
        private readonly Dictionary<int, Fixed64> _gScore = new Dictionary<int, Fixed64>();
        private readonly Dictionary<int, int> _cameFrom = new Dictionary<int, int>();
        private readonly List<int> _pathBuffer = new List<int>();

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

            // 시작/도착 삼각형의 이웃 정보 출력
            var startTriData = _navMesh.GetTriangle(startTri);
            var endTriData = _navMesh.GetTriangle(endTri);
            UnityEngine.Debug.Log($"[TriangleAStar] startTri={startTri} (neighbors: {startTriData.GetNeighbor(0)}, {startTriData.GetNeighbor(1)}, {startTriData.GetNeighbor(2)})");
            UnityEngine.Debug.Log($"[TriangleAStar] endTri={endTri} (neighbors: {endTriData.GetNeighbor(0)}, {endTriData.GetNeighbor(1)}, {endTriData.GetNeighbor(2)})");

            // 같은 삼각형 내에 있으면 바로 성공
            if (startTri == endTri)
            {
                result.Success = true;
                result.TrianglePath.Add(startTri);
                return result;
            }

            // 컬렉션 초기화
            _openSet.Clear();
            _closedSet.Clear();
            _gScore.Clear();
            _cameFrom.Clear();

            // 2. A* 알고리즘 실행
            // 시작점에서 끝점까지의 휴리스틱 (실제 점 사용)
            Fixed64 hStart = Vector2Fixed.Distance(start, end);
            _openSet.Add(new AStarNode(startTri, Fixed64.Zero, hStart));
            _gScore[startTri] = Fixed64.Zero;

            // 이전에 통과한 에지 중심점 저장 (실제 시작점부터 계산하기 위해)
            var _lastEdgeCenter = new Dictionary<int, Vector2Fixed>();
            _lastEdgeCenter[startTri] = start; // 시작 삼각형은 실제 시작점에서 출발

            while (_openSet.Count > 0)
            {
                // F가 가장 낮은 노드 선택
                int bestIndex = FindLowestFScore();
                AStarNode current = _openSet[bestIndex];
                int currentTri = current.TriangleIndex;

                // 목표 도달
                if (currentTri == endTri)
                {
                    // 삼각형 17이 탐색됐는지 확인
                    bool tri17Explored = _closedSet.Contains(17);
                    bool tri17InOpen = FindInOpenSet(17) >= 0;
                    UnityEngine.Debug.Log($"[TriangleAStar] Goal reached! Explored {_closedSet.Count} triangles. Tri17: explored={tri17Explored}, inOpenSet={tri17InOpen}");
                    result.Success = true;
                    ReconstructPath(result, currentTri);
                    return result;
                }

                _openSet.RemoveAt(bestIndex);
                _closedSet.Add(currentTri);

                // 현재 위치 (시작점이면 실제 시작점, 아니면 진입한 에지 중심점)
                Vector2Fixed currentPos = _lastEdgeCenter[currentTri];

                // 인접 삼각형들 탐색
                var triangle = _navMesh.GetTriangle(currentTri);
                for (int edge = 0; edge < 3; edge++)
                {
                    int neighbor = triangle.GetNeighbor(edge);
                    if (neighbor < 0 || _closedSet.Contains(neighbor))
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
                        UnityEngine.Debug.Log($"[TriangleAStar] Path to goal via tri {currentTri}, edge {edge}: G={tentativeG} (finalLeg={finalLeg})");
                    }

                    // 더 좋은 경로인지 확인
                    if (!_gScore.TryGetValue(neighbor, out Fixed64 existingG) || tentativeG < existingG)
                    {
                        _gScore[neighbor] = tentativeG;
                        _cameFrom[neighbor] = currentTri;
                        _lastEdgeCenter[neighbor] = edgeCenter; // 이 삼각형에 진입한 에지 중심점 저장

                        // 휴리스틱: 에지 중심점에서 실제 끝점까지 (도착 삼각형이면 이미 G에 포함했으므로 0)
                        Fixed64 h = (neighbor == endTri) ? Fixed64.Zero : Vector2Fixed.Distance(edgeCenter, end);

                        // openSet에 이미 있는지 확인하고 업데이트 또는 추가
                        int existingIndex = FindInOpenSet(neighbor);
                        if (existingIndex >= 0)
                        {
                            _openSet[existingIndex] = new AStarNode(neighbor, tentativeG, h);
                        }
                        else
                        {
                            _openSet.Add(new AStarNode(neighbor, tentativeG, h));
                        }
                    }
                }
            }

            // 경로를 찾지 못함
            return result;
        }

        private int FindInOpenSet(int triangleIndex)
        {
            for (int i = 0; i < _openSet.Count; i++)
            {
                if (_openSet[i].TriangleIndex == triangleIndex)
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// F가 가장 낮은 노드의 인덱스 반환
        /// </summary>
        private int FindLowestFScore()
        {
            int bestIndex = 0;
            Fixed64 bestF = _openSet[0].FScore;

            for (int i = 1; i < _openSet.Count; i++)
            {
                if (_openSet[i].FScore < bestF)
                {
                    bestF = _openSet[i].FScore;
                    bestIndex = i;
                }
            }

            return bestIndex;
        }

        /// <summary>
        /// 경로 재구성 (역추적)
        /// </summary>
        private void ReconstructPath(PathResult result, int endTri)
        {
            _pathBuffer.Clear();
            int current = endTri;

            while (_cameFrom.ContainsKey(current))
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

            // 총 비용 계산
            Fixed64 totalCost = _gScore.ContainsKey(endTri) ? _gScore[endTri] : Fixed64.Zero;
            UnityEngine.Debug.Log($"[TriangleAStar] Path: {result.TrianglePath.Count} triangles, G={totalCost}\n  [{string.Join(" -> ", result.TrianglePath)}]");

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
