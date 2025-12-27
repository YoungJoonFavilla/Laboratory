using System.Collections.Generic;
using FixedMathSharp;
using NavMesh2D.Geometry;
using NavMesh2D.Triangulation;
using UnityEngine;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// NavMesh 빌더
    /// 경계 폴리곤과 장애물들을 받아 NavMesh2DData를 생성
    /// </summary>
    public class NavMeshBuilder
    {
        private ConstrainedDelaunayTriangulator _triangulator;

        public NavMeshBuilder()
        {
            _triangulator = new ConstrainedDelaunayTriangulator();
        }

        /// <summary>
        /// NavMesh 베이킹
        /// </summary>
        /// <param name="boundary">외부 경계 폴리곤</param>
        /// <param name="obstacles">장애물 폴리곤들 (구멍)</param>
        /// <returns>베이킹된 NavMesh 데이터</returns>
        public NavMesh2DData Build(Polygon2D boundary, List<Polygon2D> obstacles = null)
        {
            // 1. Constrained Delaunay Triangulation 수행
            List<Triangle2D> triangles = _triangulator.Triangulate(boundary, obstacles);

            if (triangles.Count == 0)
            {
                Debug.LogWarning("[NavMeshBuilder] No triangles generated from triangulation");
                return null;
            }

            // 2. NavMesh2DData 생성 및 데이터 설정
            NavMesh2DData navMesh = ScriptableObject.CreateInstance<NavMesh2DData>();
            navMesh.SetData(triangles);

            Debug.Log($"[NavMeshBuilder] NavMesh built: {navMesh.TriangleCount} triangles, {navMesh.VertexCount} vertices");

            return navMesh;
        }

        /// <summary>
        /// 직사각형 경계로 간단하게 NavMesh 생성
        /// </summary>
        /// <param name="min">최소 좌표</param>
        /// <param name="max">최대 좌표</param>
        /// <param name="obstacles">장애물 폴리곤들</param>
        /// <returns>베이킹된 NavMesh 데이터</returns>
        public NavMesh2DData BuildFromRect(Vector2Fixed min, Vector2Fixed max, List<Polygon2D> obstacles = null)
        {
            // 직사각형 경계 폴리곤 생성 (CCW)
            Polygon2D boundary = new Polygon2D(
                new Vector2Fixed(min.x, min.y),
                new Vector2Fixed(max.x, min.y),
                new Vector2Fixed(max.x, max.y),
                new Vector2Fixed(min.x, max.y)
            );

            return Build(boundary, obstacles);
        }

        /// <summary>
        /// Unity Vector2 기반 간편 빌드 (에디터용)
        /// </summary>
        public NavMesh2DData BuildFromRect(Vector2 min, Vector2 max, List<Polygon2D> obstacles = null)
        {
            return BuildFromRect(
                new Vector2Fixed((Fixed64)min.x, (Fixed64)min.y),
                new Vector2Fixed((Fixed64)max.x, (Fixed64)max.y),
                obstacles
            );
        }

        /// <summary>
        /// 기존 Asset에 NavMesh 데이터 저장 (에디터용)
        /// </summary>
        /// <param name="target">저장할 NavMesh Asset</param>
        /// <param name="min">최소 좌표</param>
        /// <param name="max">최대 좌표</param>
        /// <param name="obstacles">장애물 폴리곤들</param>
        /// <param name="maxTriangleCount">최대 삼각형 개수 (0이면 세분화 안함)</param>
        public void BuildFromRect(NavMesh2DData target, Vector2 min, Vector2 max, List<Polygon2D> obstacles = null, int maxTriangleCount = 0)
        {
            // 직사각형 경계 폴리곤 생성 (CCW)
            Polygon2D boundary = new Polygon2D(
                new Vector2Fixed((Fixed64)min.x, (Fixed64)min.y),
                new Vector2Fixed((Fixed64)max.x, (Fixed64)min.y),
                new Vector2Fixed((Fixed64)max.x, (Fixed64)max.y),
                new Vector2Fixed((Fixed64)min.x, (Fixed64)max.y)
            );

            // 삼각분할
            List<Triangle2D> triangles = _triangulator.Triangulate(boundary, obstacles);

            if (triangles.Count == 0)
            {
                Debug.LogWarning("[NavMeshBuilder] No triangles generated from triangulation");
                return;
            }

            // 세분화 (maxTriangleCount가 현재보다 크면)
            if (maxTriangleCount > triangles.Count)
            {
                triangles = TriangleSubdivider.Subdivide(triangles, maxTriangleCount);
            }

            // 기존 Asset에 데이터 설정
            target.SetData(triangles);

            Debug.Log($"[NavMeshBuilder] NavMesh built to asset: {target.TriangleCount} triangles, {target.VertexCount} vertices");
        }

        /// <summary>
        /// Unity Vector2[] 배열로 경계 생성 (에디터용)
        /// </summary>
        public NavMesh2DData BuildFromPoints(Vector2[] boundaryPoints, List<Vector2[]> obstaclePoints = null)
        {
            // 경계 폴리곤 변환
            Polygon2D boundary = new Polygon2D();
            foreach (var p in boundaryPoints)
            {
                boundary.AddVertex(new Vector2Fixed((Fixed64)p.x, (Fixed64)p.y));
            }

            // 장애물 폴리곤 변환
            List<Polygon2D> obstacles = null;
            if (obstaclePoints != null && obstaclePoints.Count > 0)
            {
                obstacles = new List<Polygon2D>();
                foreach (var obstacleVerts in obstaclePoints)
                {
                    Polygon2D obstacle = new Polygon2D();
                    foreach (var p in obstacleVerts)
                    {
                        obstacle.AddVertex(new Vector2Fixed((Fixed64)p.x, (Fixed64)p.y));
                    }
                    obstacles.Add(obstacle);
                }
            }

            return Build(boundary, obstacles);
        }
    }

    /// <summary>
    /// NavMesh 베이킹 설정
    /// </summary>
    [System.Serializable]
    public class NavMeshBuildSettings
    {
        /// <summary>
        /// 에이전트 반지름 (장애물에서의 여유 공간)
        /// </summary>
        public Fixed64 AgentRadius = Fixed64.One / (Fixed64)2;

        /// <summary>
        /// 최소 삼각형 면적 (이보다 작은 삼각형은 제거)
        /// </summary>
        public Fixed64 MinTriangleArea = Fixed64.One / (Fixed64)100;

        /// <summary>
        /// 장애물 팽창 (에이전트 반지름만큼 장애물을 확장)
        /// </summary>
        public bool InflateObstacles = true;
    }

    /// <summary>
    /// 삼각형 세분화 유틸리티
    /// Conforming Subdivision: 에지를 분할할 때 그 에지를 공유하는 이웃도 같이 분할
    /// </summary>
    public static class TriangleSubdivider
    {
        /// <summary>
        /// 최대 삼각형 개수에 도달할 때까지 큰 삼각형부터 분할
        /// </summary>
        /// <param name="triangles">원본 삼각형 리스트</param>
        /// <param name="maxTriangleCount">목표 최대 삼각형 개수</param>
        /// <returns>세분화된 삼각형 리스트</returns>
        public static List<Triangle2D> Subdivide(List<Triangle2D> triangles, int maxTriangleCount)
        {
            if (triangles == null || triangles.Count == 0)
                return triangles;

            // 최대 개수가 현재보다 작거나 같으면 그대로 반환
            if (maxTriangleCount <= triangles.Count)
            {
                Debug.Log($"[TriangleSubdivider] maxTriangleCount({maxTriangleCount}) <= current({triangles.Count}), no subdivision needed");
                return triangles;
            }

            var result = new List<Triangle2D>(triangles);
            int subdivisionCount = 0;

            while (result.Count < maxTriangleCount)
            {
                // 에지-삼각형 맵 구축
                var edgeMap = BuildEdgeToTriangleMap(result);

                // 가장 큰 삼각형 찾기 (면적 기준)
                int largestIndex = FindLargestTriangle(result);
                if (largestIndex < 0)
                    break;

                Triangle2D largest = result[largestIndex];

                // 가장 긴 에지 찾기
                var (edgeA, edgeB, _) = FindLongestEdge(largest);
                var edgeKey = MakeEdgeKey(edgeA, edgeB);
                Vector2Fixed midpoint = (edgeA + edgeB) / (Fixed64)2;

                // 이 에지를 공유하는 모든 삼각형 찾기 (1개 또는 2개)
                List<int> sharingIndices;
                if (!edgeMap.TryGetValue(edgeKey, out sharingIndices))
                {
                    sharingIndices = new List<int> { largestIndex };
                }

                // 새 삼각형들 생성
                var newTriangles = new List<Triangle2D>();
                foreach (int idx in sharingIndices)
                {
                    var tri = result[idx];
                    var split = SplitTriangleAtEdge(tri, edgeA, edgeB, midpoint);
                    newTriangles.AddRange(split);
                }

                // 인덱스 내림차순 정렬 (삭제 시 인덱스 밀림 방지)
                var sortedIndices = new List<int>(sharingIndices);
                sortedIndices.Sort((a, b) => b.CompareTo(a));

                // 기존 삼각형 삭제
                foreach (int idx in sortedIndices)
                {
                    result.RemoveAt(idx);
                }

                // 새 삼각형 추가
                result.AddRange(newTriangles);
                subdivisionCount++;
            }

            Debug.Log($"[TriangleSubdivider] Subdivided {subdivisionCount} times: {triangles.Count} -> {result.Count}");
            return result;
        }

        /// <summary>
        /// 에지 → 삼각형 인덱스 맵 구축
        /// </summary>
        private static Dictionary<long, List<int>> BuildEdgeToTriangleMap(List<Triangle2D> triangles)
        {
            var map = new Dictionary<long, List<int>>();

            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                AddEdgeToMap(map, tri.V0, tri.V1, i);
                AddEdgeToMap(map, tri.V1, tri.V2, i);
                AddEdgeToMap(map, tri.V2, tri.V0, i);
            }

            return map;
        }

        private static void AddEdgeToMap(Dictionary<long, List<int>> map, Vector2Fixed a, Vector2Fixed b, int triIndex)
        {
            var key = MakeEdgeKey(a, b);
            if (!map.TryGetValue(key, out var list))
            {
                list = new List<int>();
                map[key] = list;
            }
            list.Add(triIndex);
        }

        /// <summary>
        /// 에지 키 생성 (순서 무관하게 동일한 키)
        /// </summary>
        private static long MakeEdgeKey(Vector2Fixed a, Vector2Fixed b)
        {
            int hashA = a.GetHashCode();
            int hashB = b.GetHashCode();

            // 작은 값이 앞에 오도록 정렬
            if (hashA > hashB)
            {
                int temp = hashA;
                hashA = hashB;
                hashB = temp;
            }

            return ((long)hashA << 32) | (uint)hashB;
        }

        /// <summary>
        /// 가장 긴 에지를 가진 삼각형의 인덱스 찾기
        /// (면적 대신 에지 길이 기준 - 길쭉한 삼각형도 우선 분할)
        /// </summary>
        private static int FindLargestTriangle(List<Triangle2D> triangles)
        {
            if (triangles.Count == 0)
                return -1;

            int bestIndex = 0;
            Fixed64 longestEdgeSqr = GetLongestEdgeSqr(triangles[0]);

            for (int i = 1; i < triangles.Count; i++)
            {
                Fixed64 edgeSqr = GetLongestEdgeSqr(triangles[i]);
                if (edgeSqr > longestEdgeSqr)
                {
                    longestEdgeSqr = edgeSqr;
                    bestIndex = i;
                }
            }

            return bestIndex;
        }

        /// <summary>
        /// 삼각형의 가장 긴 에지 길이 제곱 반환
        /// </summary>
        private static Fixed64 GetLongestEdgeSqr(Triangle2D tri)
        {
            Fixed64 len0 = Vector2Fixed.SqrDistance(tri.V0, tri.V1);
            Fixed64 len1 = Vector2Fixed.SqrDistance(tri.V1, tri.V2);
            Fixed64 len2 = Vector2Fixed.SqrDistance(tri.V2, tri.V0);

            if (len0 >= len1 && len0 >= len2) return len0;
            if (len1 >= len0 && len1 >= len2) return len1;
            return len2;
        }

        /// <summary>
        /// 삼각형에서 가장 긴 에지 찾기
        /// </summary>
        private static (Vector2Fixed, Vector2Fixed, Vector2Fixed) FindLongestEdge(Triangle2D tri)
        {
            Fixed64 len0 = Vector2Fixed.SqrDistance(tri.V0, tri.V1);
            Fixed64 len1 = Vector2Fixed.SqrDistance(tri.V1, tri.V2);
            Fixed64 len2 = Vector2Fixed.SqrDistance(tri.V2, tri.V0);

            if (len0 >= len1 && len0 >= len2)
                return (tri.V0, tri.V1, tri.V2); // E0이 가장 김, V2가 반대편 정점
            else if (len1 >= len0 && len1 >= len2)
                return (tri.V1, tri.V2, tri.V0); // E1이 가장 김, V0이 반대편 정점
            else
                return (tri.V2, tri.V0, tri.V1); // E2가 가장 김, V1이 반대편 정점
        }

        /// <summary>
        /// 삼각형을 지정된 에지의 중점에서 2개로 분할
        /// </summary>
        private static List<Triangle2D> SplitTriangleAtEdge(Triangle2D tri, Vector2Fixed edgeA, Vector2Fixed edgeB, Vector2Fixed midpoint)
        {
            // 삼각형에서 edgeA, edgeB가 아닌 정점 찾기
            Vector2Fixed opposite;
            if (tri.V0 != edgeA && tri.V0 != edgeB)
                opposite = tri.V0;
            else if (tri.V1 != edgeA && tri.V1 != edgeB)
                opposite = tri.V1;
            else
                opposite = tri.V2;

            // 새 삼각형 2개 생성 (원래 winding order 유지)
            // opposite → edgeA → midpoint
            // opposite → midpoint → edgeB
            Triangle2D tri1 = new Triangle2D(opposite, edgeA, midpoint);
            Triangle2D tri2 = new Triangle2D(opposite, midpoint, edgeB);

            return new List<Triangle2D> { tri1, tri2 };
        }
    }
}
