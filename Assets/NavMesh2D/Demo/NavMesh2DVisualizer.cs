using System.Collections.Generic;
using FixedMathSharp;
using NavMesh2D.Geometry;
using NavMesh2D.Pathfinding;
using UnityEngine;

namespace NavMesh2D.Demo
{
    /// <summary>
    /// NavMesh2D 시각화 컴포넌트
    /// 삼각형, 에지, 경로 등을 Gizmos로 표시
    /// </summary>
    public class NavMesh2DVisualizer : MonoBehaviour
    {
        [Header("=== NavMesh Data ===")]
        [SerializeField] private NavMesh2DData _navMesh;

        [Header("=== Visualization Settings ===")]
        [SerializeField] private bool _showTriangles = true;
        [SerializeField] private bool _showEdges = true;
        [SerializeField] private bool _showVertices = false;
        [SerializeField] private bool _showCentroids = false;
        [SerializeField] private bool _showNeighborConnections = false;
        [SerializeField] private bool _showTriangleIndices = false;

        [Header("=== Colors ===")]
        [SerializeField] private Color _triangleFillColor = new Color(0.2f, 0.6f, 0.9f, 0.3f);
        [SerializeField] private Color _edgeColor = new Color(0.1f, 0.3f, 0.6f, 1f);
        [SerializeField] private Color _vertexColor = Color.yellow;
        [SerializeField] private Color _centroidColor = Color.cyan;
        [SerializeField] private Color _neighborColor = new Color(1f, 0.5f, 0f, 0.5f);

        [Header("=== Path Visualization ===")]
        [SerializeField] private bool _showPath = true;
        [SerializeField] private Color _pathColor = Color.green;
        [SerializeField] private Color _portalColor = Color.magenta;
        [SerializeField] private float _pathWidth = 0.1f;
        [SerializeField] private bool _showTrianglePathIndices = true;
        [SerializeField] private float _trianglePathSphereSize = 0.15f;

        [Header("=== Agent Settings ===")]
        [SerializeField] private float _agentSpeed = 20f;
        [SerializeField] private float _agentSize = 0.5f;
        [SerializeField] private float _trailWidth = 0.08f;

        [Header("=== Normal Agent (웨이포인트 직선) ===")]
        [SerializeField] private Color _normalAgentColor = Color.black;
        [SerializeField] private Color _normalTrailColor = Color.gray;

        [Header("=== Spline Agent (스플라인 곡선) ===")]
        [SerializeField] private Color _splineAgentColor = Color.blue;
        [SerializeField] private Color _splineTrailColor = Color.cyan;

        [Header("=== Steering Agent ===")]
        [SerializeField] private Color _steeringAgentColor = Color.red;
        [SerializeField] private Color _steeringTrailColor = Color.yellow;

        // Public 프로퍼티
        public float AgentSpeed => _agentSpeed;
        public float AgentSize => _agentSize;
        public float TrailWidth => _trailWidth;

        public Color NormalAgentColor => _normalAgentColor;
        public Color NormalTrailColor => _normalTrailColor;
        public Color SplineAgentColor => _splineAgentColor;
        public Color SplineTrailColor => _splineTrailColor;
        public Color SteeringAgentColor => _steeringAgentColor;
        public Color SteeringTrailColor => _steeringTrailColor;

        // 현재 경로 (외부에서 설정)
        private List<Vector2Fixed> _currentPath;
        private List<TriangleAStar.Portal> _currentPortals;
        private List<int> _currentTrianglePath;

        // 캐싱 (성능 최적화)
        private List<Vector3> _edgePointsCache = new List<Vector3>();
        private Vector3[] _edgeArrayCache;  // ToArray() 대신 재사용
        private HashSet<long> _drawnEdgesCache = new HashSet<long>();

        public NavMesh2DData NavMesh
        {
            get => _navMesh;
            set => _navMesh = value;
        }

        /// <summary>
        /// 경로 시각화 설정
        /// </summary>
        public void SetPath(List<Vector2Fixed> path, List<TriangleAStar.Portal> portals = null, List<int> trianglePath = null)
        {
            _currentPath = path;
            _currentPortals = portals;
            _currentTrianglePath = trianglePath;
        }

        /// <summary>
        /// 경로 초기화
        /// </summary>
        public void ClearPath()
        {
            _currentPath = null;
            _currentPortals = null;
            _currentTrianglePath = null;
        }

        private void OnDrawGizmos()
        {
            if (_navMesh == null)
                return;

            DrawNavMesh();
            DrawPath();
        }

        private void DrawNavMesh()
        {
            int triCount = _navMesh.TriangleCount;

            // 에지 배치 그리기 (중복 제거 + 배치 처리)
            if (_showEdges)
            {
                DrawEdgesBatched(triCount);
            }

            // 삼각형 채우기
            if (_showTriangles)
            {
                DrawTrianglesFilled(triCount);
            }

            // 정점 그리기
            if (_showVertices)
            {
                DrawVertices();
            }

            // 중심점 그리기
            if (_showCentroids)
            {
                DrawCentroids(triCount);
            }

            // 이웃 연결 그리기
            if (_showNeighborConnections)
            {
                DrawNeighborConnections(triCount);
            }

            // 삼각형 인덱스 표시
            if (_showTriangleIndices)
            {
                DrawAllTriangleIndices();
            }
        }

        /// <summary>
        /// 에지 배치 그리기 (중복 제거 + Handles.DrawLines 사용)
        /// </summary>
        private void DrawEdgesBatched(int triCount)
        {
            _edgePointsCache.Clear();
            _drawnEdgesCache.Clear();

            for (int i = 0; i < triCount; i++)
            {
                var tri = _navMesh.GetTriangle(i);

                // 각 에지를 중복 없이 추가
                TryAddEdge(tri.V0, tri.V1);
                TryAddEdge(tri.V1, tri.V2);
                TryAddEdge(tri.V2, tri.V0);
            }

#if UNITY_EDITOR
            if (_edgePointsCache.Count > 0)
            {
                UnityEditor.Handles.color = _edgeColor;
                DrawLinesWithCache();
            }
#endif
        }

        private void TryAddEdge(int v0Index, int v1Index)
        {
            // 에지 키 생성 (작은 인덱스가 앞에)
            long edgeKey = v0Index < v1Index
                ? ((long)v0Index << 32) | (uint)v1Index
                : ((long)v1Index << 32) | (uint)v0Index;

            if (_drawnEdgesCache.Contains(edgeKey))
                return;

            _drawnEdgesCache.Add(edgeKey);

            Vector3 p0 = ToVector3(_navMesh.GetVertex(v0Index));
            Vector3 p1 = ToVector3(_navMesh.GetVertex(v1Index));
            _edgePointsCache.Add(p0);
            _edgePointsCache.Add(p1);
        }

        /// <summary>
        /// 삼각형 채우기 (배치)
        /// </summary>
        private void DrawTrianglesFilled(int triCount)
        {
#if UNITY_EDITOR
            UnityEditor.Handles.color = _triangleFillColor;
            for (int i = 0; i < triCount; i++)
            {
                var tri = _navMesh.GetTriangle(i);
                Vector3 v0 = ToVector3(_navMesh.GetVertex(tri.V0));
                Vector3 v1 = ToVector3(_navMesh.GetVertex(tri.V1));
                Vector3 v2 = ToVector3(_navMesh.GetVertex(tri.V2));
                UnityEditor.Handles.DrawAAConvexPolygon(v0, v1, v2);
            }
#endif
        }

        /// <summary>
        /// 정점 그리기 (중복 제거)
        /// </summary>
        private void DrawVertices()
        {
            Gizmos.color = _vertexColor;
            int vertCount = _navMesh.VertexCount;
            for (int i = 0; i < vertCount; i++)
            {
                Vector3 v = ToVector3(_navMesh.GetVertex(i));
                Gizmos.DrawSphere(v, 0.05f);
            }
        }

        /// <summary>
        /// 중심점 그리기
        /// </summary>
        private void DrawCentroids(int triCount)
        {
            Gizmos.color = _centroidColor;
            for (int i = 0; i < triCount; i++)
            {
                var geo = _navMesh.GetTriangleGeometry(i);
                Vector3 centroid = ToVector3(geo.Centroid);
                Gizmos.DrawSphere(centroid, 0.08f);
            }
        }

        /// <summary>
        /// 이웃 연결 그리기
        /// </summary>
        private void DrawNeighborConnections(int triCount)
        {
            _edgePointsCache.Clear();

            for (int i = 0; i < triCount; i++)
            {
                var tri = _navMesh.GetTriangle(i);
                var geo = _navMesh.GetTriangleGeometry(i);
                Vector3 centroid = ToVector3(geo.Centroid);

                for (int e = 0; e < 3; e++)
                {
                    int neighbor = tri.GetNeighbor(e);
                    if (neighbor >= 0 && neighbor > i) // 중복 방지
                    {
                        var neighborGeo = _navMesh.GetTriangleGeometry(neighbor);
                        Vector3 neighborCentroid = ToVector3(neighborGeo.Centroid);
                        _edgePointsCache.Add(centroid);
                        _edgePointsCache.Add(neighborCentroid);
                    }
                }
            }

#if UNITY_EDITOR
            if (_edgePointsCache.Count > 0)
            {
                UnityEditor.Handles.color = _neighborColor;
                DrawLinesWithCache();
            }
#endif
        }

        private void DrawAllTriangleIndices()
        {
#if UNITY_EDITOR
            var style = new GUIStyle();
            style.normal.textColor = Color.white;
            style.fontSize = 12;
            style.fontStyle = FontStyle.Bold;
            style.alignment = TextAnchor.MiddleCenter;

            for (int i = 0; i < _navMesh.TriangleCount; i++)
            {
                var geo = _navMesh.GetTriangleGeometry(i);
                Vector3 centroid = ToVector3(geo.Centroid);
                UnityEditor.Handles.Label(centroid, i.ToString(), style);
            }
#endif
        }

        private void DrawPath()
        {
            if (!_showPath)
                return;

            // 포탈 그리기
            if (_currentPortals != null)
            {
                Gizmos.color = _portalColor;
                foreach (var portal in _currentPortals)
                {
                    Vector3 left = ToVector3(portal.Left);
                    Vector3 right = ToVector3(portal.Right);
                    Gizmos.DrawLine(left, right);
                    Gizmos.DrawSphere(left, 0.04f);
                    Gizmos.DrawSphere(right, 0.04f);
                }
            }

            // 경로 그리기
            if (_currentPath != null && _currentPath.Count >= 2)
            {
                Gizmos.color = _pathColor;

                for (int i = 0; i < _currentPath.Count - 1; i++)
                {
                    Vector3 from = ToVector3(_currentPath[i]);
                    Vector3 to = ToVector3(_currentPath[i + 1]);

                    // 두꺼운 선 효과
                    DrawThickLine(from, to, _pathWidth);
                }

                // 웨이포인트 표시
                for (int i = 0; i < _currentPath.Count; i++)
                {
                    Vector3 point = ToVector3(_currentPath[i]);
                    float size = (i == 0 || i == _currentPath.Count - 1) ? 0.15f : 0.08f;
                    Gizmos.DrawSphere(point, size);
                }
            }

            // 삼각형 경로 인덱스 표시
            DrawTrianglePathIndices();
        }

        private void DrawTrianglePathIndices()
        {
            if (!_showTrianglePathIndices || _currentTrianglePath == null || _currentTrianglePath.Count == 0 || _navMesh == null)
                return;

            for (int i = 0; i < _currentTrianglePath.Count; i++)
            {
                int triIndex = _currentTrianglePath[i];
                var geo = _navMesh.GetTriangleGeometry(triIndex);
                Vector3 centroid = ToVector3(geo.Centroid);

                // 배경 원 그리기 (파란색)
                Gizmos.color = new Color(0.2f, 0.2f, 0.9f, 0.9f);
                Gizmos.DrawSphere(centroid, _trianglePathSphereSize);

                // 경로 순서를 선으로 연결
                if (i > 0)
                {
                    int prevTriIndex = _currentTrianglePath[i - 1];
                    var prevGeo = _navMesh.GetTriangleGeometry(prevTriIndex);
                    Vector3 prevCentroid = ToVector3(prevGeo.Centroid);

                    Gizmos.color = Color.yellow;
                    Gizmos.DrawLine(prevCentroid, centroid);
                }
            }

#if UNITY_EDITOR
            // 텍스트 레이블
            var style = new GUIStyle();
            style.normal.textColor = Color.white;
            style.fontSize = 16;
            style.fontStyle = FontStyle.Bold;
            style.alignment = TextAnchor.MiddleCenter;

            for (int i = 0; i < _currentTrianglePath.Count; i++)
            {
                int triIndex = _currentTrianglePath[i];
                var geo = _navMesh.GetTriangleGeometry(triIndex);
                Vector3 centroid = ToVector3(geo.Centroid);
                UnityEditor.Handles.Label(centroid + Vector3.up * 0.1f, $"{i}:{triIndex}", style);
            }
#endif
        }

        private void DrawThickLine(Vector3 from, Vector3 to, float width)
        {
            Vector3 dir = (to - from).normalized;
            Vector3 perpendicular = new Vector3(-dir.y, dir.x, 0) * width * 0.5f;

            Gizmos.DrawLine(from, to);
            Gizmos.DrawLine(from + perpendicular, to + perpendicular);
            Gizmos.DrawLine(from - perpendicular, to - perpendicular);
        }

        private static Vector3 ToVector3(Vector2Fixed v)
        {
            return new Vector3((float)v.x, (float)v.y, 0);
        }

        /// <summary>
        /// 캐시된 배열로 라인 그리기 (ToArray 호출 방지)
        /// </summary>
        private void DrawLinesWithCache()
        {
#if UNITY_EDITOR
            int count = _edgePointsCache.Count;
            if (count == 0) return;

            // 배열 크기가 정확히 맞을 때만 재사용
            if (_edgeArrayCache == null || _edgeArrayCache.Length != count)
            {
                _edgeArrayCache = new Vector3[count];
            }

            _edgePointsCache.CopyTo(_edgeArrayCache);
            UnityEditor.Handles.DrawLines(_edgeArrayCache);
#endif
        }
    }
}
