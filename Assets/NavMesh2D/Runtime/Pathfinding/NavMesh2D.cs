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
            for (int i = 0; i < _triangles.Count; i++)
            {
                var geo = GetTriangleGeometry(i);
                if (geo.ContainsPoint(point))
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// 가장 가까운 삼각형 인덱스 찾기
        /// </summary>
        public int FindNearestTriangle(Vector2Fixed point)
        {
            int nearest = -1;
            Fixed64 nearestDist = (Fixed64)999999999;

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
        /// 에지의 중심점 (Portal) 가져오기
        /// </summary>
        public Vector2Fixed GetEdgeCenter(int triIndex, int edgeIndex)
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
