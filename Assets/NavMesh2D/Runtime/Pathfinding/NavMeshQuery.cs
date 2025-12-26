using System.Collections.Generic;
using FixedMathSharp;
using NavMesh2D.Geometry;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// NavMesh 경로 찾기 통합 API
    /// A* + Funnel Algorithm을 조합하여 최단 경로 제공
    /// </summary>
    public class NavMeshQuery
    {
        private NavMesh2DData _navMesh;
        private TriangleAStar _aStar;
        private FunnelAlgorithm _funnel;

        /// <summary>
        /// 경로 찾기 결과
        /// </summary>
        public struct PathQueryResult
        {
            /// <summary>
            /// 경로 찾기 성공 여부
            /// </summary>
            public bool Success;

            /// <summary>
            /// 경로 웨이포인트 리스트 (시작 → 끝)
            /// </summary>
            public List<Vector2Fixed> Path;

            /// <summary>
            /// 경로 총 길이
            /// </summary>
            public Fixed64 PathLength;

            /// <summary>
            /// 디버그용: 통과한 삼각형 인덱스들
            /// </summary>
            public List<int> TrianglePath;

            /// <summary>
            /// 디버그용: 포탈 리스트
            /// </summary>
            public List<TriangleAStar.Portal> Portals;
        }

        public NavMeshQuery(NavMesh2DData navMesh)
        {
            _navMesh = navMesh;
            _aStar = new TriangleAStar(navMesh);
            _funnel = new FunnelAlgorithm();
        }

        /// <summary>
        /// 경로 찾기 수행
        /// </summary>
        /// <param name="start">시작점</param>
        /// <param name="end">끝점</param>
        /// <returns>경로 찾기 결과</returns>
        public PathQueryResult FindPath(Vector2Fixed start, Vector2Fixed end)
        {
            PathQueryResult result = new PathQueryResult
            {
                Success = false,
                Path = new List<Vector2Fixed>(),
                PathLength = Fixed64.Zero,
                TrianglePath = new List<int>(),
                Portals = new List<TriangleAStar.Portal>()
            };

            // 1. A*로 삼각형 경로 찾기
            var astarResult = _aStar.FindPath(start, end);

            if (!astarResult.Success)
            {
                return result;
            }

            result.TrianglePath = astarResult.TrianglePath;
            result.Portals = astarResult.Portals;

            UnityEngine.Debug.Log($"[NavMeshQuery] A* result: TrianglePath.Count={astarResult.TrianglePath?.Count ?? -1}, Portals.Count={astarResult.Portals?.Count ?? -1}");

            // 2. 같은 삼각형 내라면 직선 경로
            if (astarResult.TrianglePath.Count == 1)
            {
                result.Success = true;
                result.Path.Add(start);
                result.Path.Add(end);
                result.PathLength = Vector2Fixed.Distance(start, end);
                return result;
            }

            // 3. 포탈 방향 정규화
            var normalizedPortals = _funnel.NormalizePortals(astarResult.Portals, start);

            // 4. Funnel Algorithm으로 경로 스무딩
            result.Path = _funnel.StringPull(start, end, normalizedPortals);
            result.Success = true;

            // 5. 경로 길이 계산
            result.PathLength = CalculatePathLength(result.Path);

            return result;
        }

        /// <summary>
        /// 점이 NavMesh 위에 있는지 확인
        /// </summary>
        public bool IsPointOnNavMesh(Vector2Fixed point)
        {
            return _navMesh.FindTriangleContainingPoint(point) >= 0;
        }

        /// <summary>
        /// 점을 NavMesh 위의 가장 가까운 점으로 이동
        /// </summary>
        public Vector2Fixed ClampToNavMesh(Vector2Fixed point)
        {
            int triIndex = _navMesh.FindTriangleContainingPoint(point);
            if (triIndex >= 0)
                return point; // 이미 NavMesh 위

            // 가장 가까운 삼각형 찾기
            triIndex = _navMesh.FindNearestTriangle(point);
            if (triIndex < 0)
                return point; // NavMesh가 비어있음

            // 삼각형 내부의 가장 가까운 점 찾기
            var tri = _navMesh.GetTriangleGeometry(triIndex);
            return ClosestPointOnTriangle(point, tri.V0, tri.V1, tri.V2);
        }

        /// <summary>
        /// 선분이 NavMesh와 교차하는 가장 먼 지점 찾기
        /// (Raycast 용도)
        /// </summary>
        public Vector2Fixed Raycast(Vector2Fixed origin, Vector2Fixed direction, Fixed64 maxDistance)
        {
            Vector2Fixed end = origin + direction * maxDistance;

            // 현재 삼각형 찾기
            int currentTri = _navMesh.FindTriangleContainingPoint(origin);
            if (currentTri < 0)
            {
                currentTri = _navMesh.FindNearestTriangle(origin);
                if (currentTri < 0)
                    return origin;
            }

            Vector2Fixed current = origin;
            int maxIterations = _navMesh.TriangleCount * 2;
            int iterations = 0;

            while (iterations < maxIterations)
            {
                iterations++;

                var tri = _navMesh.GetTriangle(currentTri);
                var geo = _navMesh.GetTriangleGeometry(currentTri);

                // 끝점이 현재 삼각형 내에 있으면 완료
                if (geo.ContainsPoint(end))
                    return end;

                // 현재 삼각형의 에지들과 선분 교차 확인
                int nextTri = -1;
                Vector2Fixed intersection = current;

                for (int i = 0; i < 3; i++)
                {
                    int neighbor = tri.GetNeighbor(i);
                    var edge = geo.GetEdge(i);

                    if (LineSegmentIntersection(current, end, edge.A, edge.B, out Vector2Fixed point))
                    {
                        if (neighbor >= 0)
                        {
                            // 이웃 삼각형으로 이동
                            nextTri = neighbor;
                            intersection = point;
                            break;
                        }
                        else
                        {
                            // NavMesh 경계 - 여기서 정지
                            return point;
                        }
                    }
                }

                if (nextTri < 0)
                {
                    // 더 이상 진행할 수 없음
                    return current;
                }

                current = intersection;
                currentTri = nextTri;
            }

            return current;
        }

        /// <summary>
        /// 경로 길이 계산
        /// </summary>
        private Fixed64 CalculatePathLength(List<Vector2Fixed> path)
        {
            if (path == null || path.Count < 2)
                return Fixed64.Zero;

            Fixed64 length = Fixed64.Zero;
            for (int i = 0; i < path.Count - 1; i++)
            {
                length += Vector2Fixed.Distance(path[i], path[i + 1]);
            }
            return length;
        }

        /// <summary>
        /// 삼각형 위의 가장 가까운 점 찾기
        /// </summary>
        private Vector2Fixed ClosestPointOnTriangle(Vector2Fixed p, Vector2Fixed a, Vector2Fixed b, Vector2Fixed c)
        {
            // 각 에지에서 가장 가까운 점을 찾고 가장 가까운 것 선택
            Vector2Fixed closest = a;
            Fixed64 closestDist = Vector2Fixed.SqrDistance(p, a);

            // 에지 AB
            Vector2Fixed pAB = ClosestPointOnSegment(p, a, b);
            Fixed64 distAB = Vector2Fixed.SqrDistance(p, pAB);
            if (distAB < closestDist)
            {
                closest = pAB;
                closestDist = distAB;
            }

            // 에지 BC
            Vector2Fixed pBC = ClosestPointOnSegment(p, b, c);
            Fixed64 distBC = Vector2Fixed.SqrDistance(p, pBC);
            if (distBC < closestDist)
            {
                closest = pBC;
                closestDist = distBC;
            }

            // 에지 CA
            Vector2Fixed pCA = ClosestPointOnSegment(p, c, a);
            Fixed64 distCA = Vector2Fixed.SqrDistance(p, pCA);
            if (distCA < closestDist)
            {
                closest = pCA;
            }

            return closest;
        }

        /// <summary>
        /// 선분 위의 가장 가까운 점 찾기
        /// </summary>
        private Vector2Fixed ClosestPointOnSegment(Vector2Fixed p, Vector2Fixed a, Vector2Fixed b)
        {
            Vector2Fixed ab = b - a;
            Fixed64 abLenSqr = ab.SqrMagnitude;

            if (abLenSqr == Fixed64.Zero)
                return a;

            Fixed64 t = Vector2Fixed.Dot(p - a, ab) / abLenSqr;

            // Clamp to [0, 1]
            if (t < Fixed64.Zero) t = Fixed64.Zero;
            if (t > Fixed64.One) t = Fixed64.One;

            return a + ab * t;
        }

        /// <summary>
        /// 두 선분의 교차점 계산
        /// </summary>
        private bool LineSegmentIntersection(Vector2Fixed p1, Vector2Fixed p2, Vector2Fixed p3, Vector2Fixed p4, out Vector2Fixed intersection)
        {
            intersection = p1;

            Vector2Fixed d1 = p2 - p1;
            Vector2Fixed d2 = p4 - p3;

            Fixed64 cross = d1.x * d2.y - d1.y * d2.x;

            // 평행
            if (cross == Fixed64.Zero)
                return false;

            Vector2Fixed d3 = p3 - p1;
            Fixed64 t = (d3.x * d2.y - d3.y * d2.x) / cross;
            Fixed64 u = (d3.x * d1.y - d3.y * d1.x) / cross;

            // 교차점이 두 선분 내에 있는지 확인
            if (t >= Fixed64.Zero && t <= Fixed64.One && u >= Fixed64.Zero && u <= Fixed64.One)
            {
                intersection = p1 + d1 * t;
                return true;
            }

            return false;
        }
    }
}
