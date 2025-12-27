using System.Collections.Generic;
using FixedMathSharp;
using NavMesh2D.Geometry;
using UnityEngine;

namespace NavMesh2D.Triangulation
{
    /// <summary>
    /// 들로네 삼각분할 (Bowyer-Watson Algorithm)
    /// 점 집합을 들로네 조건을 만족하는 삼각형들로 분할
    /// </summary>
    public class DelaunayTriangulator
    {
        private List<Triangle2D> _triangles;
        private List<Vector2Fixed> _points;

        // Super Triangle 정점 (나중에 제거용)
        private Vector2Fixed _superA, _superB, _superC;

        public DelaunayTriangulator()
        {
            _triangles = new List<Triangle2D>();
            _points = new List<Vector2Fixed>();
        }

        /// <summary>
        /// 점 집합에 대해 들로네 삼각분할 수행
        /// </summary>
        /// <param name="points">입력 점 집합</param>
        /// <returns>삼각형 목록</returns>
        public List<Triangle2D> Triangulate(IEnumerable<Vector2Fixed> points)
        {
            _points = new List<Vector2Fixed>(points);
            _triangles = new List<Triangle2D>();

            if (_points.Count < 3)
                return _triangles;

            // 1. Super Triangle 생성 (모든 점을 포함)
            CreateSuperTriangle();

            // 2. 각 점을 하나씩 삽입
            foreach (var point in _points)
            {
                InsertPoint(point);
            }

            // 3. Super Triangle과 연결된 삼각형 제거
            RemoveSuperTriangleConnections();

            return _triangles;
        }

        /// <summary>
        /// 모든 점을 포함하는 큰 삼각형(Super Triangle) 생성
        /// </summary>
        private void CreateSuperTriangle()
        {
            // AABB 계산
            Fixed64 minX = _points[0].x, maxX = _points[0].x;
            Fixed64 minY = _points[0].y, maxY = _points[0].y;

            foreach (var p in _points)
            {
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }

            Fixed64 dx = maxX - minX;
            Fixed64 dy = maxY - minY;
            Fixed64 deltaMax = dx > dy ? dx : dy;
            Fixed64 midX = (minX + maxX) / (Fixed64)2;
            Fixed64 midY = (minY + maxY) / (Fixed64)2;

            // Super Triangle 정점 (충분히 크게)
            Fixed64 margin = deltaMax * (Fixed64)20;
            _superA = new Vector2Fixed(midX - margin, minY - margin);
            _superB = new Vector2Fixed(midX, maxY + margin * (Fixed64)2);
            _superC = new Vector2Fixed(midX + margin, minY - margin);

            _triangles.Add(new Triangle2D(_superA, _superB, _superC));
        }

        /// <summary>
        /// 점 삽입 (Bowyer-Watson)
        /// </summary>
        private void InsertPoint(Vector2Fixed point)
        {
            // 1. 외접원에 점이 포함되는 삼각형들 찾기 (Bad Triangles)
            List<Triangle2D> badTriangles = new List<Triangle2D>();
            foreach (var tri in _triangles)
            {
                if (tri.CircumcircleContains(point))
                {
                    badTriangles.Add(tri);
                }
            }

            // 2. Bad Triangles의 경계 에지 찾기 (Polygon Hole)
            List<Edge> polygon = new List<Edge>();
            foreach (var tri in badTriangles)
            {
                for (int i = 0; i < 3; i++)
                {
                    var edge = tri.GetEdge(i);
                    Edge e = new Edge(edge.A, edge.B);

                    // 에지가 다른 bad triangle과 공유되는지 확인
                    bool isShared = false;
                    foreach (var other in badTriangles)
                    {
                        if (tri.Equals(other)) continue;

                        for (int j = 0; j < 3; j++)
                        {
                            var otherEdge = other.GetEdge(j);
                            if (e.IsSame(new Edge(otherEdge.A, otherEdge.B)))
                            {
                                isShared = true;
                                break;
                            }
                        }
                        if (isShared) break;
                    }

                    // 공유되지 않는 에지만 폴리곤에 추가 (구멍의 경계)
                    if (!isShared)
                    {
                        polygon.Add(e);
                    }
                }
            }

            // 3. Bad Triangles 제거
            foreach (var tri in badTriangles)
            {
                _triangles.Remove(tri);
            }

            // 4. 새로운 삼각형 생성 (점과 폴리곤 에지 연결)
            foreach (var edge in polygon)
            {
                _triangles.Add(new Triangle2D(edge.A, edge.B, point));
            }
        }

        /// <summary>
        /// Super Triangle과 연결된 삼각형 제거
        /// </summary>
        private void RemoveSuperTriangleConnections()
        {
            _triangles.RemoveAll(tri =>
                tri.HasVertex(_superA) ||
                tri.HasVertex(_superB) ||
                tri.HasVertex(_superC)
            );
        }

        /// <summary>
        /// 에지 구조체 (비교용)
        /// </summary>
        private struct Edge
        {
            public Vector2Fixed A;
            public Vector2Fixed B;

            public Edge(Vector2Fixed a, Vector2Fixed b)
            {
                A = a;
                B = b;
            }

            /// <summary>
            /// 순서와 관계없이 같은 에지인지 확인
            /// </summary>
            public bool IsSame(Edge other)
            {
                return (A == other.A && B == other.B) ||
                       (A == other.B && B == other.A);
            }
        }
    }

    /// <summary>
    /// Constrained Delaunay Triangulation (CDT)
    /// 장애물 에지를 유지하면서 삼각분할
    /// </summary>
    public class ConstrainedDelaunayTriangulator
    {
        private DelaunayTriangulator _baseTriangulator;
        private List<Triangle2D> _triangles;
        private List<ConstraintEdge> _constraints;

        public ConstrainedDelaunayTriangulator()
        {
            _baseTriangulator = new DelaunayTriangulator();
            _constraints = new List<ConstraintEdge>();
        }

        /// <summary>
        /// 경계 폴리곤과 장애물 폴리곤들로 Constrained 삼각분할 수행
        /// </summary>
        /// <param name="boundary">외부 경계 폴리곤</param>
        /// <param name="obstacles">장애물 폴리곤들 (구멍)</param>
        /// <param name="additionalPoints">메시 품질 개선용 추가 점들 (제약 조건 없음)</param>
        /// <returns>삼각형 목록</returns>
        public List<Triangle2D> Triangulate(Polygon2D boundary, List<Polygon2D> obstacles = null, List<Vector2Fixed> additionalPoints = null)
        {
            // 초기화
            _constraints.Clear();
            _triangles = new List<Triangle2D>();

            if (boundary == null || boundary.VertexCount < 3)
            {
                return _triangles;
            }

            // 장애물 겹침 검사
            if (obstacles != null && obstacles.Count > 1)
            {
                if (!ValidateNoOverlappingObstacles(obstacles, out string errorMsg))
                {
                    Debug.LogError($"[CDT] NavMesh 빌드 실패: {errorMsg}");
                    return _triangles;
                }
            }

            // 1. 모든 정점 수집
            List<Vector2Fixed> allPoints = new List<Vector2Fixed>();

            // 경계 정점
            foreach (var v in boundary.Vertices)
            {
                allPoints.Add(v);
            }

            // 경계 에지를 제약 조건으로 추가
            for (int i = 0; i < boundary.VertexCount; i++)
            {
                int next = (i + 1) % boundary.VertexCount;
                _constraints.Add(new ConstraintEdge(
                    boundary.GetVertex(i),
                    boundary.GetVertex(next),
                    true // 경계
                ));
            }

            // 장애물 정점 및 에지
            if (obstacles != null)
            {
                foreach (var obstacle in obstacles)
                {
                    foreach (var v in obstacle.Vertices)
                    {
                        allPoints.Add(v);
                    }

                    for (int i = 0; i < obstacle.VertexCount; i++)
                    {
                        int next = (i + 1) % obstacle.VertexCount;
                        _constraints.Add(new ConstraintEdge(
                            obstacle.GetVertex(i),
                            obstacle.GetVertex(next),
                            false // 장애물
                        ));
                    }
                }
            }

            // 추가 점 (메시 품질 개선용 - 제약 조건 없음)
            // 중복 점은 degenerate 삼각형을 발생시키므로 제외
            if (additionalPoints != null)
            {
                foreach (var p in additionalPoints)
                {
                    bool isDuplicate = false;
                    foreach (var existing in allPoints)
                    {
                        if (existing == p)
                        {
                            isDuplicate = true;
                            break;
                        }
                    }
                    if (!isDuplicate)
                    {
                        allPoints.Add(p);
                    }
                }
            }

            // 2. 기본 들로네 삼각분할 수행
            _triangles = _baseTriangulator.Triangulate(allPoints);

            if (_triangles == null || _triangles.Count == 0)
            {
                _triangles = new List<Triangle2D>();
                return _triangles;
            }

            // 3. 제약 에지 복원 (Edge Flipping)
            RestoreConstraintEdges();

            // 4. 장애물 내부 삼각형 제거
            if (obstacles != null)
            {
                RemoveTrianglesInsideObstacles(obstacles);
            }

            // 5. 경계 외부 삼각형 제거
            RemoveTrianglesOutsideBoundary(boundary);

            // 6. Degenerate 삼각형 제거 (면적이 0이거나 매우 작은 삼각형)
            RemoveDegenerateTriangles();

            return _triangles;
        }

        /// <summary>
        /// 제약 에지 복원 (Edge Flipping 기반)
        /// </summary>
        private void RestoreConstraintEdges()
        {
            foreach (var constraint in _constraints)
            {
                if (!EdgeExists(constraint.A, constraint.B))
                {
                    // 제약 에지가 없으면 Edge Flip으로 생성
                    RestoreEdge(constraint.A, constraint.B);
                }
            }
        }

        /// <summary>
        /// 에지가 존재하는지 확인
        /// </summary>
        private bool EdgeExists(Vector2Fixed a, Vector2Fixed b)
        {
            foreach (var tri in _triangles)
            {
                for (int i = 0; i < 3; i++)
                {
                    var edge = tri.GetEdge(i);
                    if ((edge.A == a && edge.B == b) || (edge.A == b && edge.B == a))
                        return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Edge Flipping으로 제약 에지 복원
        /// 단순화된 구현: 교차하는 에지를 찾아 Flip
        /// </summary>
        private void RestoreEdge(Vector2Fixed a, Vector2Fixed b)
        {
            // 이 구현은 단순화된 버전입니다
            // 실제로는 교차하는 모든 에지를 찾아 순차적으로 Flip해야 합니다

            int maxIterations = _triangles.Count * 2;
            int iterations = 0;

            while (!EdgeExists(a, b) && iterations < maxIterations)
            {
                iterations++;

                // a-b를 교차하는 에지를 찾아 Flip
                bool flipped = false;
                for (int i = 0; i < _triangles.Count && !flipped; i++)
                {
                    var tri = _triangles[i];

                    for (int e = 0; e < 3; e++)
                    {
                        var edge = tri.GetEdge(e);

                        // 이 에지가 a-b와 교차하는지 확인
                        if (EdgesIntersect(a, b, edge.A, edge.B))
                        {
                            // 인접 삼각형 찾기
                            int adjIndex = FindAdjacentTriangle(i, edge.A, edge.B);
                            if (adjIndex >= 0)
                            {
                                // Edge Flip 수행
                                FlipEdge(i, adjIndex, edge.A, edge.B);
                                flipped = true;
                                break;
                            }
                        }
                    }
                }

                if (!flipped) break;
            }
        }

        /// <summary>
        /// 두 선분이 교차하는지 확인 (끝점 제외)
        /// </summary>
        private bool EdgesIntersect(Vector2Fixed a1, Vector2Fixed a2, Vector2Fixed b1, Vector2Fixed b2)
        {
            // 끝점이 같으면 교차하지 않음
            if (a1 == b1 || a1 == b2 || a2 == b1 || a2 == b2)
                return false;

            Fixed64 d1 = Vector2Fixed.Cross(a2 - a1, b1 - a1);
            Fixed64 d2 = Vector2Fixed.Cross(a2 - a1, b2 - a1);
            Fixed64 d3 = Vector2Fixed.Cross(b2 - b1, a1 - b1);
            Fixed64 d4 = Vector2Fixed.Cross(b2 - b1, a2 - b1);

            if (((d1 > Fixed64.Zero && d2 < Fixed64.Zero) || (d1 < Fixed64.Zero && d2 > Fixed64.Zero)) &&
                ((d3 > Fixed64.Zero && d4 < Fixed64.Zero) || (d3 < Fixed64.Zero && d4 > Fixed64.Zero)))
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// 에지를 공유하는 인접 삼각형 찾기
        /// </summary>
        private int FindAdjacentTriangle(int triIndex, Vector2Fixed edgeA, Vector2Fixed edgeB)
        {
            for (int i = 0; i < _triangles.Count; i++)
            {
                if (i == triIndex) continue;

                var tri = _triangles[i];
                if (tri.HasVertex(edgeA) && tri.HasVertex(edgeB))
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// Edge Flip 수행
        /// </summary>
        private void FlipEdge(int tri1Index, int tri2Index, Vector2Fixed edgeA, Vector2Fixed edgeB)
        {
            var tri1 = _triangles[tri1Index];
            var tri2 = _triangles[tri2Index];

            // 공유하지 않는 정점 찾기
            Vector2Fixed opp1 = tri1.GetOppositeVertex(edgeA, edgeB);
            Vector2Fixed opp2 = tri2.GetOppositeVertex(edgeA, edgeB);

            // 새로운 삼각형 생성
            Triangle2D newTri1 = new Triangle2D(opp1, opp2, edgeA);
            Triangle2D newTri2 = new Triangle2D(opp1, opp2, edgeB);

            // 새 삼각형이 유효한지 확인 (면적 > 0)
            if (newTri1.Area() > Fixed64.Zero && newTri2.Area() > Fixed64.Zero)
            {
                _triangles[tri1Index] = newTri1;
                _triangles[tri2Index] = newTri2;
            }
        }

        /// <summary>
        /// 장애물 내부 삼각형 제거
        /// </summary>
        private void RemoveTrianglesInsideObstacles(List<Polygon2D> obstacles)
        {
            _triangles.RemoveAll(tri =>
            {
                Vector2Fixed centroid = tri.Centroid;
                foreach (var obstacle in obstacles)
                {
                    if (obstacle.ContainsPoint(centroid))
                        return true;
                }
                return false;
            });
        }

        /// <summary>
        /// 경계 외부 삼각형 제거
        /// </summary>
        private void RemoveTrianglesOutsideBoundary(Polygon2D boundary)
        {
            _triangles.RemoveAll(tri =>
            {
                Vector2Fixed centroid = tri.Centroid;
                return !boundary.ContainsPoint(centroid);
            });
        }

        /// <summary>
        /// Degenerate 삼각형 제거 (면적이 0이거나 매우 작은 삼각형, 중복 정점)
        /// </summary>
        private void RemoveDegenerateTriangles()
        {
            // 최소 면적 임계값 (Fixed64 기준 매우 작은 값)
            Fixed64 minArea = Fixed64.One / (Fixed64)10000;

            _triangles.RemoveAll(tri =>
            {
                // 중복 정점 검사
                if (tri.V0 == tri.V1 || tri.V1 == tri.V2 || tri.V2 == tri.V0)
                    return true;

                // 면적이 0이거나 매우 작은 삼각형
                Fixed64 area = tri.Area();
                if (area < Fixed64.Zero)
                    area = -area;  // 절대값

                return area < minArea;
            });
        }

        /// <summary>
        /// 장애물 폴리곤들이 겹치는지 검사
        /// </summary>
        /// <param name="obstacles">장애물 리스트</param>
        /// <param name="errorMsg">에러 메시지 (실패 시)</param>
        /// <returns>겹침이 없으면 true</returns>
        private bool ValidateNoOverlappingObstacles(List<Polygon2D> obstacles, out string errorMsg)
        {
            errorMsg = null;

            for (int i = 0; i < obstacles.Count; i++)
            {
                for (int j = i + 1; j < obstacles.Count; j++)
                {
                    if (PolygonsOverlap(obstacles[i], obstacles[j], i, j, out string detail))
                    {
                        string nameI = string.IsNullOrEmpty(obstacles[i].Name) ? $"#{i}" : $"[{obstacles[i].Name}]";
                        string nameJ = string.IsNullOrEmpty(obstacles[j].Name) ? $"#{j}" : $"[{obstacles[j].Name}]";
                        errorMsg = $"장애물 {nameI}와 {nameJ}가 겹칩니다. {detail}";
                        return false;
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// 두 폴리곤이 겹치는지 검사 (에지 교차 검사)
        /// </summary>
        private bool PolygonsOverlap(Polygon2D polyA, Polygon2D polyB, int indexA, int indexB, out string detail)
        {
            detail = null;
            string nameA = string.IsNullOrEmpty(polyA.Name) ? $"#{indexA}" : polyA.Name;
            string nameB = string.IsNullOrEmpty(polyB.Name) ? $"#{indexB}" : polyB.Name;

            // 1. 에지 교차 검사
            for (int i = 0; i < polyA.VertexCount; i++)
            {
                int nextI = (i + 1) % polyA.VertexCount;
                Vector2Fixed a1 = polyA.GetVertex(i);
                Vector2Fixed a2 = polyA.GetVertex(nextI);

                for (int j = 0; j < polyB.VertexCount; j++)
                {
                    int nextJ = (j + 1) % polyB.VertexCount;
                    Vector2Fixed b1 = polyB.GetVertex(j);
                    Vector2Fixed b2 = polyB.GetVertex(nextJ);

                    // 공유 에지는 허용 (정확히 같은 에지)
                    if (EdgesAreIdentical(a1, a2, b1, b2))
                        continue;

                    // 에지가 교차하는지 검사
                    if (EdgesIntersect(a1, a2, b1, b2))
                    {
                        // Fixed64 raw 값도 출력해서 정확히 비교 가능하게
                        detail = $"에지 교차 - {nameA}[{i}]-[{nextI}]와 {nameB}[{j}]-[{nextJ}]\n" +
                                 $"  {nameA}[{i}] = ({a1.x}, {a1.y})\n" +
                                 $"  {nameA}[{nextI}] = ({a2.x}, {a2.y})\n" +
                                 $"  {nameB}[{j}] = ({b1.x}, {b1.y})\n" +
                                 $"  {nameB}[{nextJ}] = ({b2.x}, {b2.y})";
                        return true;
                    }
                }
            }

            // 2. 포함 관계 검사 (한 폴리곤이 다른 폴리곤 안에 완전히 들어간 경우)
            // A의 정점 중 B의 정점이 아닌 것이 B 내부에 있는지 확인
            for (int i = 0; i < polyA.VertexCount; i++)
            {
                Vector2Fixed vertA = polyA.GetVertex(i);

                // B의 정점과 공유하는지 확인
                if (IsSharedVertex(vertA, polyB))
                    continue;

                if (polyB.ContainsPoint(vertA))
                {
                    detail = $"{nameA}의 정점[{i}]({(float)vertA.x:F2},{(float)vertA.y:F2})이 {nameB} 내부에 위치함";
                    return true;
                }
            }

            // B의 정점 중 A의 정점이 아닌 것이 A 내부에 있는지 확인
            for (int i = 0; i < polyB.VertexCount; i++)
            {
                Vector2Fixed vertB = polyB.GetVertex(i);

                // A의 정점과 공유하는지 확인
                if (IsSharedVertex(vertB, polyA))
                    continue;

                if (polyA.ContainsPoint(vertB))
                {
                    detail = $"{nameB}의 정점[{i}]({(float)vertB.x:F2},{(float)vertB.y:F2})이 {nameA} 내부에 위치함";
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// 두 에지가 동일한지 검사 (순서 무관)
        /// </summary>
        private bool EdgesAreIdentical(Vector2Fixed a1, Vector2Fixed a2, Vector2Fixed b1, Vector2Fixed b2)
        {
            return (a1 == b1 && a2 == b2) || (a1 == b2 && a2 == b1);
        }

        /// <summary>
        /// 점이 폴리곤의 정점 중 하나와 일치하는지 확인
        /// </summary>
        private bool IsSharedVertex(Vector2Fixed point, Polygon2D poly)
        {
            for (int i = 0; i < poly.VertexCount; i++)
            {
                if (point == poly.GetVertex(i))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// 제약 에지 구조체
        /// </summary>
        private struct ConstraintEdge
        {
            public Vector2Fixed A;
            public Vector2Fixed B;
            public bool IsBoundary;

            public ConstraintEdge(Vector2Fixed a, Vector2Fixed b, bool isBoundary)
            {
                A = a;
                B = b;
                IsBoundary = isBoundary;
            }
        }
    }
}
