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

        // A* 노드
        private class AStarNode
        {
            public int TriangleIndex;
            public Fixed64 G; // 시작부터 현재까지 비용
            public Fixed64 H; // 현재부터 목표까지 추정 비용
            public Fixed64 F => G + H; // 총 비용
            public AStarNode Parent;
            public int EdgeFromParent; // 부모에서 이 노드로 온 에지 인덱스

            public AStarNode(int triangleIndex)
            {
                TriangleIndex = triangleIndex;
                G = Fixed64.Zero;
                H = Fixed64.Zero;
                Parent = null;
                EdgeFromParent = -1;
            }
        }

        public TriangleAStar(NavMesh2DData navMesh)
        {
            _navMesh = navMesh;
        }

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

            // 2. A* 알고리즘 실행
            var openSet = new List<AStarNode>();
            var closedSet = new HashSet<int>();
            var nodeMap = new Dictionary<int, AStarNode>();

            // 시작 노드
            var startNode = new AStarNode(startTri);
            startNode.H = Heuristic(startTri, end);
            openSet.Add(startNode);
            nodeMap[startTri] = startNode;

            while (openSet.Count > 0)
            {
                // F가 가장 낮은 노드 선택
                AStarNode current = GetLowestFNode(openSet);
                openSet.Remove(current);

                // 목표 도달
                if (current.TriangleIndex == endTri)
                {
                    result.Success = true;
                    ReconstructPath(current, result);
                    return result;
                }

                closedSet.Add(current.TriangleIndex);

                // 인접 삼각형들 탐색
                var triangle = _navMesh.GetTriangle(current.TriangleIndex);
                for (int edge = 0; edge < 3; edge++)
                {
                    int neighbor = triangle.GetNeighbor(edge);
                    if (neighbor < 0 || closedSet.Contains(neighbor))
                        continue;

                    // 에지 중심점으로 이동 비용 계산
                    Vector2Fixed edgeCenter = _navMesh.GetEdgeCenter(current.TriangleIndex, edge);
                    Fixed64 moveCost = GetMoveCost(current, edgeCenter);
                    Fixed64 newG = current.G + moveCost;

                    // 이웃 노드가 열린 집합에 있는지 확인
                    if (!nodeMap.TryGetValue(neighbor, out AStarNode neighborNode))
                    {
                        neighborNode = new AStarNode(neighbor);
                        neighborNode.H = Heuristic(neighbor, end);
                        nodeMap[neighbor] = neighborNode;
                        openSet.Add(neighborNode);
                    }
                    else if (!openSet.Contains(neighborNode))
                    {
                        // 이미 닫힌 집합에 있으면 건너뛰기
                        continue;
                    }

                    // 더 좋은 경로면 업데이트
                    if (newG < neighborNode.G || neighborNode.Parent == null)
                    {
                        neighborNode.G = newG;
                        neighborNode.Parent = current;
                        neighborNode.EdgeFromParent = edge;
                    }
                }
            }

            // 경로를 찾지 못함
            return result;
        }

        /// <summary>
        /// F가 가장 낮은 노드 반환
        /// </summary>
        private AStarNode GetLowestFNode(List<AStarNode> openSet)
        {
            AStarNode lowest = openSet[0];
            Fixed64 lowestF = lowest.F;

            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].F < lowestF)
                {
                    lowest = openSet[i];
                    lowestF = lowest.F;
                }
            }

            return lowest;
        }

        /// <summary>
        /// 휴리스틱 계산 (삼각형 중심점에서 목표까지의 거리)
        /// </summary>
        private Fixed64 Heuristic(int triangleIndex, Vector2Fixed target)
        {
            var geo = _navMesh.GetTriangleGeometry(triangleIndex);
            return Vector2Fixed.Distance(geo.Centroid, target);
        }

        /// <summary>
        /// 이동 비용 계산
        /// </summary>
        private Fixed64 GetMoveCost(AStarNode from, Vector2Fixed to)
        {
            var fromGeo = _navMesh.GetTriangleGeometry(from.TriangleIndex);
            return Vector2Fixed.Distance(fromGeo.Centroid, to);
        }

        /// <summary>
        /// 경로 재구성 (역추적)
        /// </summary>
        private void ReconstructPath(AStarNode endNode, PathResult result)
        {
            // 역순으로 경로 수집
            var path = new List<int>();
            var edges = new List<int>();

            AStarNode current = endNode;
            while (current != null)
            {
                path.Add(current.TriangleIndex);
                if (current.Parent != null)
                {
                    edges.Add(current.EdgeFromParent);
                }
                current = current.Parent;
            }

            // 역순을 정순으로
            path.Reverse();
            edges.Reverse();

            result.TrianglePath = path;

            // 포탈 생성 (연속된 삼각형들 사이의 공유 에지)
            for (int i = 0; i < path.Count - 1; i++)
            {
                int triA = path[i];
                int triB = path[i + 1];

                var portal = _navMesh.GetPortalBetween(triA, triB);
                if (portal.HasValue)
                {
                    result.Portals.Add(new Portal(portal.Value.Left, portal.Value.Right));
                }
            }
        }
    }
}
