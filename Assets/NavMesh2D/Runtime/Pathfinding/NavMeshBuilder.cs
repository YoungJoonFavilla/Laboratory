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
        public void BuildFromRect(NavMesh2DData target, Vector2 min, Vector2 max, List<Polygon2D> obstacles = null)
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
}
