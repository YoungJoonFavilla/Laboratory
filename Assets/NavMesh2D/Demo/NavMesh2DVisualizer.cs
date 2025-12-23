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

        public NavMesh2DData NavMesh
        {
            get => _navMesh;
            set => _navMesh = value;
        }

        /// <summary>
        /// 경로 시각화 설정
        /// </summary>
        public void SetPath(List<Vector2Fixed> path, List<TriangleAStar.Portal> portals = null)
        {
            _currentPath = path;
            _currentPortals = portals;
        }

        /// <summary>
        /// 경로 초기화
        /// </summary>
        public void ClearPath()
        {
            _currentPath = null;
            _currentPortals = null;
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

            for (int i = 0; i < triCount; i++)
            {
                var tri = _navMesh.GetTriangle(i);
                Vector3 v0 = ToVector3(_navMesh.GetVertex(tri.V0));
                Vector3 v1 = ToVector3(_navMesh.GetVertex(tri.V1));
                Vector3 v2 = ToVector3(_navMesh.GetVertex(tri.V2));

                // 삼각형 채우기
                if (_showTriangles)
                {
                    Gizmos.color = _triangleFillColor;
                    DrawFilledTriangle(v0, v1, v2);
                }

                // 에지
                if (_showEdges)
                {
                    Gizmos.color = _edgeColor;
                    Gizmos.DrawLine(v0, v1);
                    Gizmos.DrawLine(v1, v2);
                    Gizmos.DrawLine(v2, v0);
                }

                // 정점
                if (_showVertices)
                {
                    Gizmos.color = _vertexColor;
                    Gizmos.DrawSphere(v0, 0.05f);
                    Gizmos.DrawSphere(v1, 0.05f);
                    Gizmos.DrawSphere(v2, 0.05f);
                }

                // 중심점
                if (_showCentroids)
                {
                    var geo = _navMesh.GetTriangleGeometry(i);
                    Vector3 centroid = ToVector3(geo.Centroid);
                    Gizmos.color = _centroidColor;
                    Gizmos.DrawSphere(centroid, 0.08f);
                }

                // 이웃 연결
                if (_showNeighborConnections)
                {
                    var geo = _navMesh.GetTriangleGeometry(i);
                    Vector3 centroid = ToVector3(geo.Centroid);

                    Gizmos.color = _neighborColor;
                    for (int e = 0; e < 3; e++)
                    {
                        int neighbor = tri.GetNeighbor(e);
                        if (neighbor >= 0 && neighbor > i) // 중복 방지
                        {
                            var neighborGeo = _navMesh.GetTriangleGeometry(neighbor);
                            Vector3 neighborCentroid = ToVector3(neighborGeo.Centroid);
                            Gizmos.DrawLine(centroid, neighborCentroid);
                        }
                    }
                }
            }
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
        }

        private void DrawFilledTriangle(Vector3 v0, Vector3 v1, Vector3 v2)
        {
            // Unity Gizmos는 직접 삼각형을 채울 수 없으므로
            // 여러 선으로 근사하거나 Handles.DrawAAConvexPolygon 사용 (에디터 전용)
#if UNITY_EDITOR
            UnityEditor.Handles.color = _triangleFillColor;
            UnityEditor.Handles.DrawAAConvexPolygon(v0, v1, v2);
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
    }
}
