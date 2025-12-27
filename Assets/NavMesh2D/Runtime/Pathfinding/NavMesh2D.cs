using System;
using System.Collections.Generic;
using FixedMathSharp;
using NavMesh2D.Geometry;
using UnityEngine;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// 2D 네비게이션 메시
    /// 삼각형들과 인접 관계를 저장하여 경로 찾기에 사용
    /// </summary>
    [CreateAssetMenu(fileName = "NavMesh2D", menuName = "NavMesh2D/NavMesh Data")]
    public class NavMesh2DData : ScriptableObject
    {
        [SerializeField]
        private List<NavTriangle> _triangles = new List<NavTriangle>();

        [SerializeField]
        private List<Vector2Fixed> _vertices = new List<Vector2Fixed>();

        // Spatial Grid (런타임 전용, 직렬화 안함)
        private const int GRID_RESOLUTION = 32; // 32x32 그리드
        // Flattened grid: 단일 배열 + offset 방식 (cache friendly)
        private int[] _gridData;      // 모든 셀의 삼각형 인덱스 (연속 배열)
        private int[] _gridOffsets;   // 각 셀의 시작 오프셋
        private int[] _gridCounts;    // 각 셀의 삼각형 개수
        private Vector2Fixed _gridMin;
        private Vector2Fixed _gridMax;
        private Fixed64 _cellWidth;
        private Fixed64 _cellHeight;
        private bool _gridBuilt = false;

        // Edge Center 캐시 (런타임 전용)
        // _edgeCenters[triIndex * 3 + edgeIndex]
        private Vector2Fixed[] _edgeCenters;

        /// <summary>
        /// 삼각형 목록
        /// </summary>
        public IReadOnlyList<NavTriangle> Triangles => _triangles;

        /// <summary>
        /// 정점 목록
        /// </summary>
        public IReadOnlyList<Vector2Fixed> Vertices => _vertices;

        /// <summary>
        /// 삼각형 개수
        /// </summary>
        public int TriangleCount => _triangles.Count;

        /// <summary>
        /// 정점 개수
        /// </summary>
        public int VertexCount => _vertices.Count;

        /// <summary>
        /// NavMesh 데이터 설정
        /// </summary>
        public void SetData(List<Triangle2D> triangles)
        {
            _triangles.Clear();
            _vertices.Clear();

            // 정점 인덱스 매핑
            Dictionary<Vector2Fixed, int> vertexMap = new Dictionary<Vector2Fixed, int>();

            foreach (var tri in triangles)
            {
                int[] indices = new int[3];

                for (int i = 0; i < 3; i++)
                {
                    Vector2Fixed v = tri[i];
                    if (!vertexMap.TryGetValue(v, out int index))
                    {
                        index = _vertices.Count;
                        _vertices.Add(v);
                        vertexMap[v] = index;
                    }
                    indices[i] = index;
                }

                _triangles.Add(new NavTriangle(indices[0], indices[1], indices[2]));
            }

            // 인접 관계 계산
            CalculateAdjacency();

            // Spatial Grid 빌드
            BuildSpatialGrid();
        }

        /// <summary>
        /// Spatial Grid 빌드 (빠른 삼각형 조회용)
        /// </summary>
        private void BuildSpatialGrid()
        {
            if (_vertices.Count == 0 || _triangles.Count == 0)
            {
                _gridBuilt = false;
                return;
            }

            // 1. 경계 계산
            _gridMin = _vertices[0];
            _gridMax = _vertices[0];

            foreach (var v in _vertices)
            {
                if (v.x < _gridMin.x) _gridMin = new Vector2Fixed(v.x, _gridMin.y);
                if (v.y < _gridMin.y) _gridMin = new Vector2Fixed(_gridMin.x, v.y);
                if (v.x > _gridMax.x) _gridMax = new Vector2Fixed(v.x, _gridMax.y);
                if (v.y > _gridMax.y) _gridMax = new Vector2Fixed(_gridMax.x, v.y);
            }

            // 약간의 여유 추가 (경계 위 점 처리)
            Fixed64 epsilon = (Fixed64)0.001;
            _gridMin = new Vector2Fixed(_gridMin.x - epsilon, _gridMin.y - epsilon);
            _gridMax = new Vector2Fixed(_gridMax.x + epsilon, _gridMax.y + epsilon);

            // 2. 셀 크기 계산
            _cellWidth = (_gridMax.x - _gridMin.x) / (Fixed64)GRID_RESOLUTION;
            _cellHeight = (_gridMax.y - _gridMin.y) / (Fixed64)GRID_RESOLUTION;

            // 0으로 나누기 방지
            if (_cellWidth <= Fixed64.Zero) _cellWidth = Fixed64.One;
            if (_cellHeight <= Fixed64.Zero) _cellHeight = Fixed64.One;

            int cellCount = GRID_RESOLUTION * GRID_RESOLUTION;
            _gridCounts = new int[cellCount];
            _gridOffsets = new int[cellCount];

            // 3. 첫 번째 패스: 각 셀의 삼각형 개수 카운트
            for (int triIndex = 0; triIndex < _triangles.Count; triIndex++)
            {
                var tri = _triangles[triIndex];
                var v0 = _vertices[tri.V0];
                var v1 = _vertices[tri.V1];
                var v2 = _vertices[tri.V2];

                // 삼각형 AABB 계산
                Fixed64 minX = FixedMath.Min(v0.x, FixedMath.Min(v1.x, v2.x));
                Fixed64 minY = FixedMath.Min(v0.y, FixedMath.Min(v1.y, v2.y));
                Fixed64 maxX = FixedMath.Max(v0.x, FixedMath.Max(v1.x, v2.x));
                Fixed64 maxY = FixedMath.Max(v0.y, FixedMath.Max(v1.y, v2.y));

                int cellMinX = GetCellX(minX);
                int cellMinY = GetCellY(minY);
                int cellMaxX = GetCellX(maxX);
                int cellMaxY = GetCellY(maxY);

                for (int cy = cellMinY; cy <= cellMaxY; cy++)
                {
                    for (int cx = cellMinX; cx <= cellMaxX; cx++)
                    {
                        int cellIndex = cy * GRID_RESOLUTION + cx;
                        _gridCounts[cellIndex]++;
                    }
                }
            }

            // 4. 오프셋 계산 (누적 합)
            int totalEntries = 0;
            for (int i = 0; i < cellCount; i++)
            {
                _gridOffsets[i] = totalEntries;
                totalEntries += _gridCounts[i];
            }

            // 5. 데이터 배열 할당 및 채우기
            _gridData = new int[totalEntries];
            int[] fillIndex = new int[cellCount]; // 각 셀의 현재 채우기 위치

            for (int triIndex = 0; triIndex < _triangles.Count; triIndex++)
            {
                var tri = _triangles[triIndex];
                var v0 = _vertices[tri.V0];
                var v1 = _vertices[tri.V1];
                var v2 = _vertices[tri.V2];

                Fixed64 minX = FixedMath.Min(v0.x, FixedMath.Min(v1.x, v2.x));
                Fixed64 minY = FixedMath.Min(v0.y, FixedMath.Min(v1.y, v2.y));
                Fixed64 maxX = FixedMath.Max(v0.x, FixedMath.Max(v1.x, v2.x));
                Fixed64 maxY = FixedMath.Max(v0.y, FixedMath.Max(v1.y, v2.y));

                int cellMinX = GetCellX(minX);
                int cellMinY = GetCellY(minY);
                int cellMaxX = GetCellX(maxX);
                int cellMaxY = GetCellY(maxY);

                for (int cy = cellMinY; cy <= cellMaxY; cy++)
                {
                    for (int cx = cellMinX; cx <= cellMaxX; cx++)
                    {
                        int cellIndex = cy * GRID_RESOLUTION + cx;
                        int dataIndex = _gridOffsets[cellIndex] + fillIndex[cellIndex];
                        _gridData[dataIndex] = triIndex;
                        fillIndex[cellIndex]++;
                    }
                }
            }

            _gridBuilt = true;

            // 5. Edge Center 캐시 빌드
            BuildEdgeCenterCache();
        }

        /// <summary>
        /// Edge Center 캐시 빌드
        /// </summary>
        private void BuildEdgeCenterCache()
        {
            _edgeCenters = new Vector2Fixed[_triangles.Count * 3];

            for (int triIndex = 0; triIndex < _triangles.Count; triIndex++)
            {
                var tri = _triangles[triIndex];
                var v0 = _vertices[tri.V0];
                var v1 = _vertices[tri.V1];
                var v2 = _vertices[tri.V2];

                // Edge 0: V0-V1
                _edgeCenters[triIndex * 3 + 0] = new Vector2Fixed(
                    (v0.x + v1.x) / (Fixed64)2,
                    (v0.y + v1.y) / (Fixed64)2
                );

                // Edge 1: V1-V2
                _edgeCenters[triIndex * 3 + 1] = new Vector2Fixed(
                    (v1.x + v2.x) / (Fixed64)2,
                    (v1.y + v2.y) / (Fixed64)2
                );

                // Edge 2: V2-V0
                _edgeCenters[triIndex * 3 + 2] = new Vector2Fixed(
                    (v2.x + v0.x) / (Fixed64)2,
                    (v2.y + v0.y) / (Fixed64)2
                );
            }
        }

        private int GetCellX(Fixed64 x)
        {
            int cell = (int)((x - _gridMin.x) / _cellWidth);
            return Math.Clamp(cell, 0, GRID_RESOLUTION - 1);
        }

        private int GetCellY(Fixed64 y)
        {
            int cell = (int)((y - _gridMin.y) / _cellHeight);
            return Math.Clamp(cell, 0, GRID_RESOLUTION - 1);
        }

        /// <summary>
        /// 인접 관계 계산
        /// </summary>
        private void CalculateAdjacency()
        {
            int count = _triangles.Count;

            for (int i = 0; i < count; i++)
            {
                var tri = _triangles[i];
                int[] neighbors = { -1, -1, -1 };

                for (int j = 0; j < count; j++)
                {
                    if (i == j) continue;

                    var other = _triangles[j];

                    // 공유 에지 찾기
                    int sharedEdge = FindSharedEdge(tri, other);
                    if (sharedEdge >= 0)
                    {
                        neighbors[sharedEdge] = j;
                    }
                }

                _triangles[i] = new NavTriangle(tri.V0, tri.V1, tri.V2, neighbors[0], neighbors[1], neighbors[2]);
            }
        }

        /// <summary>
        /// 두 삼각형이 공유하는 에지 인덱스 찾기
        /// </summary>
        private int FindSharedEdge(NavTriangle a, NavTriangle b)
        {
            int[] aVerts = { a.V0, a.V1, a.V2 };
            int[] bVerts = { b.V0, b.V1, b.V2 };

            // 에지 0: V0-V1
            if (ContainsEdge(bVerts, aVerts[0], aVerts[1])) return 0;
            // 에지 1: V1-V2
            if (ContainsEdge(bVerts, aVerts[1], aVerts[2])) return 1;
            // 에지 2: V2-V0
            if (ContainsEdge(bVerts, aVerts[2], aVerts[0])) return 2;

            return -1;
        }

        private bool ContainsEdge(int[] verts, int a, int b)
        {
            bool hasA = false, hasB = false;
            foreach (int v in verts)
            {
                if (v == a) hasA = true;
                if (v == b) hasB = true;
            }
            return hasA && hasB;
        }

        /// <summary>
        /// 정점 가져오기
        /// </summary>
        public Vector2Fixed GetVertex(int index)
        {
            return _vertices[index];
        }

        /// <summary>
        /// 삼각형 가져오기
        /// </summary>
        public NavTriangle GetTriangle(int index)
        {
            return _triangles[index];
        }

        /// <summary>
        /// 삼각형의 정점 좌표 가져오기
        /// </summary>
        public Triangle2D GetTriangleGeometry(int index)
        {
            var tri = _triangles[index];
            return new Triangle2D(
                _vertices[tri.V0],
                _vertices[tri.V1],
                _vertices[tri.V2]
            );
        }

        /// <summary>
        /// 점이 포함된 삼각형 인덱스 찾기
        /// </summary>
        public int FindTriangleContainingPoint(Vector2Fixed point)
        {
            // Spatial Grid 사용 (빌드된 경우)
            if (_gridBuilt && _gridData != null)
            {
                // 점이 그리드 범위 밖이면 -1
                if (point.x < _gridMin.x || point.x > _gridMax.x ||
                    point.y < _gridMin.y || point.y > _gridMax.y)
                {
                    return -1;
                }

                int cellX = GetCellX(point.x);
                int cellY = GetCellY(point.y);
                int cellIndex = cellY * GRID_RESOLUTION + cellX;

                // 해당 셀의 삼각형만 체크 (flattened grid)
                int offset = _gridOffsets[cellIndex];
                int count = _gridCounts[cellIndex];
                for (int i = 0; i < count; i++)
                {
                    int triIndex = _gridData[offset + i];
                    var geo = GetTriangleGeometry(triIndex);
                    if (geo.ContainsPoint(point))
                        return triIndex;
                }
                return -1;
            }

            // Fallback: 전체 순회
            for (int i = 0; i < _triangles.Count; i++)
            {
                var geo = GetTriangleGeometry(i);
                if (geo.ContainsPoint(point))
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// Spatial Grid가 빌드 안됐으면 빌드 (런타임 로드 시)
        /// </summary>
        public void EnsureSpatialGrid()
        {
            if (!_gridBuilt)
            {
                BuildSpatialGrid();
            }
        }

        /// <summary>
        /// 가장 가까운 삼각형 인덱스 찾기
        /// </summary>
        public int FindNearestTriangle(Vector2Fixed point)
        {
            // Spatial Grid 사용
            if (_gridBuilt && _gridData != null)
            {
                return FindNearestTriangleWithGrid(point);
            }

            // Fallback: 전체 순회
            int nearest = -1;
            Fixed64 nearestDist = Fixed64.MAX_VALUE;

            for (int i = 0; i < _triangles.Count; i++)
            {
                var geo = GetTriangleGeometry(i);
                Fixed64 dist = Vector2Fixed.SqrDistance(point, geo.Centroid);

                if (dist < nearestDist)
                {
                    nearestDist = dist;
                    nearest = i;
                }
            }

            return nearest;
        }

        /// <summary>
        /// Spatial Grid를 이용한 가장 가까운 삼각형 찾기
        /// 중심 셀에서 시작해서 링 형태로 확장
        /// </summary>
        private int FindNearestTriangleWithGrid(Vector2Fixed point)
        {
            int centerX = GetCellX(point.x);
            int centerY = GetCellY(point.y);

            int nearest = -1;
            Fixed64 nearestDistSqr = Fixed64.MAX_VALUE;

            // 링 형태로 확장 탐색 (최대 그리드 크기까지)
            int maxRing = GRID_RESOLUTION;

            for (int ring = 0; ring <= maxRing; ring++)
            {
                // 현재 링에서 찾은 최소 거리가 다음 링의 최소 가능 거리보다 작으면 종료
                if (nearest >= 0)
                {
                    // 다음 링까지의 최소 거리 (셀 경계까지의 거리)
                    Fixed64 nextRingMinDist = _cellWidth * (Fixed64)(ring);
                    Fixed64 nextRingMinDistSqr = nextRingMinDist * nextRingMinDist;

                    if (nearestDistSqr <= nextRingMinDistSqr)
                    {
                        break; // 더 가까운 삼각형이 다음 링에 없음
                    }
                }

                // 링의 셀들을 순회
                for (int dx = -ring; dx <= ring; dx++)
                {
                    for (int dy = -ring; dy <= ring; dy++)
                    {
                        // 링의 테두리만 (내부는 이전 링에서 처리됨)
                        if (ring > 0 && Math.Abs(dx) < ring && Math.Abs(dy) < ring)
                            continue;

                        int cx = centerX + dx;
                        int cy = centerY + dy;

                        // 범위 체크
                        if (cx < 0 || cx >= GRID_RESOLUTION || cy < 0 || cy >= GRID_RESOLUTION)
                            continue;

                        int cellIndex = cy * GRID_RESOLUTION + cx;

                        // 해당 셀의 삼각형들 체크 (flattened grid)
                        int offset = _gridOffsets[cellIndex];
                        int count = _gridCounts[cellIndex];
                        for (int i = 0; i < count; i++)
                        {
                            int triIndex = _gridData[offset + i];
                            var geo = GetTriangleGeometry(triIndex);
                            Fixed64 distSqr = Vector2Fixed.SqrDistance(point, geo.Centroid);

                            if (distSqr < nearestDistSqr)
                            {
                                nearestDistSqr = distSqr;
                                nearest = triIndex;
                            }
                        }
                    }
                }

                // 현재 링에서 찾았고, 다음 링은 확실히 더 멀면 종료
                if (nearest >= 0 && ring > 0)
                {
                    Fixed64 nextRingMinDist = _cellWidth * (Fixed64)(ring + 1);
                    if (nearestDistSqr <= nextRingMinDist * nextRingMinDist)
                    {
                        break;
                    }
                }
            }

            return nearest;
        }

        /// <summary>
        /// 에지의 중심점 (Portal) 가져오기
        /// </summary>
        public Vector2Fixed GetEdgeCenter(int triIndex, int edgeIndex)
        {
            // 캐시된 값 사용
            if (_edgeCenters != null)
            {
                return _edgeCenters[triIndex * 3 + edgeIndex];
            }

            // Fallback (캐시 없을 때)
            var tri = _triangles[triIndex];
            int v0, v1;

            switch (edgeIndex)
            {
                case 0: v0 = tri.V0; v1 = tri.V1; break;
                case 1: v0 = tri.V1; v1 = tri.V2; break;
                case 2: v0 = tri.V2; v1 = tri.V0; break;
                default: throw new IndexOutOfRangeException();
            }

            return new Vector2Fixed(
                (_vertices[v0].x + _vertices[v1].x) / (Fixed64)2,
                (_vertices[v0].y + _vertices[v1].y) / (Fixed64)2
            );
        }

        /// <summary>
        /// 에지의 두 끝점 가져오기 (Portal)
        /// </summary>
        public (Vector2Fixed Left, Vector2Fixed Right) GetEdgePortal(int triIndex, int edgeIndex)
        {
            var tri = _triangles[triIndex];
            int v0, v1;

            switch (edgeIndex)
            {
                case 0: v0 = tri.V0; v1 = tri.V1; break;
                case 1: v0 = tri.V1; v1 = tri.V2; break;
                case 2: v0 = tri.V2; v1 = tri.V0; break;
                default: throw new IndexOutOfRangeException();
            }

            return (_vertices[v0], _vertices[v1]);
        }

        /// <summary>
        /// 두 삼각형 사이의 공유 에지(Portal) 가져오기
        /// </summary>
        public (Vector2Fixed Left, Vector2Fixed Right)? GetPortalBetween(int triA, int triB)
        {
            var ta = _triangles[triA];

            for (int i = 0; i < 3; i++)
            {
                int neighbor = ta.GetNeighbor(i);
                if (neighbor == triB)
                {
                    return GetEdgePortal(triA, i);
                }
            }

            return null;
        }

    }

    /// <summary>
    /// 네비게이션 삼각형
    /// 정점 인덱스와 인접 삼각형 인덱스 저장
    /// </summary>
    [Serializable]
    public struct NavTriangle
    {
        /// <summary>
        /// 정점 인덱스
        /// </summary>
        public int V0, V1, V2;

        /// <summary>
        /// 인접 삼각형 인덱스 (에지별, -1이면 없음)
        /// N0: V0-V1 에지의 인접 삼각형
        /// N1: V1-V2 에지의 인접 삼각형
        /// N2: V2-V0 에지의 인접 삼각형
        /// </summary>
        public int N0, N1, N2;

        public NavTriangle(int v0, int v1, int v2)
        {
            V0 = v0;
            V1 = v1;
            V2 = v2;
            N0 = -1;
            N1 = -1;
            N2 = -1;
        }

        public NavTriangle(int v0, int v1, int v2, int n0, int n1, int n2)
        {
            V0 = v0;
            V1 = v1;
            V2 = v2;
            N0 = n0;
            N1 = n1;
            N2 = n2;
        }

        /// <summary>
        /// 에지별 인접 삼각형 가져오기
        /// </summary>
        public int GetNeighbor(int edgeIndex)
        {
            return edgeIndex switch
            {
                0 => N0,
                1 => N1,
                2 => N2,
                _ => -1
            };
        }

        /// <summary>
        /// 인접 삼각형 목록 (유효한 것만)
        /// </summary>
        public IEnumerable<int> GetNeighbors()
        {
            if (N0 >= 0) yield return N0;
            if (N1 >= 0) yield return N1;
            if (N2 >= 0) yield return N2;
        }

        /// <summary>
        /// 특정 삼각형과 인접한 에지 인덱스 찾기
        /// </summary>
        public int GetEdgeToNeighbor(int neighborIndex)
        {
            if (N0 == neighborIndex) return 0;
            if (N1 == neighborIndex) return 1;
            if (N2 == neighborIndex) return 2;
            return -1;
        }
    }
}
