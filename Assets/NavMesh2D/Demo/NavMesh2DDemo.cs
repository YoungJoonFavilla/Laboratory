using System.Collections.Generic;
using FixedMathSharp;
using NavMesh2D.Geometry;
using NavMesh2D.Pathfinding;
using UnityEngine;
using UnityEngine.InputSystem;

namespace NavMesh2D.Demo
{
    /// <summary>
    /// NavMesh2D 데모 컨트롤러
    /// 마우스 클릭으로 경로 탐색 테스트
    /// </summary>
    public class NavMesh2DDemo : MonoBehaviour
    {
        [Header("=== NavMesh Settings ===")]
        [SerializeField] private Vector2 _boundaryMin = new Vector2(-10, -10);
        [SerializeField] private Vector2 _boundaryMax = new Vector2(10, 10);

        [Header("=== Obstacles ===")]
        [SerializeField] private List<PolygonObstacle> _obstacles = new List<PolygonObstacle>();

        [Header("=== Components ===")]
        [SerializeField] private NavMesh2DVisualizer _visualizer;

        [Header("=== Debug ===")]
        [SerializeField] private bool _autoRebuildOnChange = true;
        [SerializeField] private Vector2 _startPoint;
        [SerializeField] private Vector2 _endPoint;

        [Header("=== Camera Control ===")]
        [SerializeField] private float _cameraZoomMin = 3f;
        [SerializeField] private float _cameraZoomMax = 10f;
        [SerializeField] private float _cameraZoomSpeed = 2f;
        [SerializeField] private float _cameraMoveSpeed = 10f;

        private NavMesh2DData _navMesh;
        private NavMeshQuery _query;
        private NavMeshBuilder _builder;

        private bool _isSettingStart = true; // true: 시작점 설정 중, false: 끝점 설정 중

        [Header("=== Background ===")]
        [SerializeField] private bool _createBackground = true;
        [SerializeField] private Color _backgroundColor = Color.white;
        private GameObject _backgroundObject;

        private List<Vector2> _currentPath; // 원본 경로
        private List<Vector2> _splinePath;  // 스플라인 보간된 경로

        // Visualizer에서 설정 가져오기
        private float AgentSpeed => _visualizer != null ? _visualizer.AgentSpeed : 20f;
        private float AgentSize => _visualizer != null ? _visualizer.AgentSize : 0.5f;
        private float TrailWidth => _visualizer != null ? _visualizer.TrailWidth : 0.08f;

        // 세 개의 에이전트
        private PathAgent _normalAgent;
        private PathAgent _splineAgent;
        private PathAgent _steeringAgent;

        /// <summary>
        /// 경로 추적 에이전트
        /// </summary>
        private class PathAgent
        {
            public GameObject gameObject;
            public SpriteRenderer spriteRenderer;
            public LineRenderer trailLine;
            public List<Vector2> trail = new List<Vector2>();
            public int pathIndex;
            public bool isFollowing;
            public Color dotColor;
            public Color trailColor;
            public Vector2 direction; // Steering용
        }

        private void Start()
        {
            _builder = new NavMeshBuilder();
            CreateBackground();
            CreateAgents();
            RebuildNavMesh();
        }

        private void CreateAgents()
        {
            // Visualizer에서 색상 가져오기
            Color normalDot = _visualizer != null ? _visualizer.NormalAgentColor : Color.black;
            Color normalTrail = _visualizer != null ? _visualizer.NormalTrailColor : Color.gray;
            Color splineDot = _visualizer != null ? _visualizer.SplineAgentColor : Color.blue;
            Color splineTrail = _visualizer != null ? _visualizer.SplineTrailColor : Color.cyan;
            Color steeringDot = _visualizer != null ? _visualizer.SteeringAgentColor : Color.red;
            Color steeringTrail = _visualizer != null ? _visualizer.SteeringTrailColor : Color.yellow;

            // Normal Agent (웨이포인트 직선)
            _normalAgent = CreateAgent("NormalAgent", normalDot, normalTrail);

            // Spline Agent (스플라인 곡선)
            _splineAgent = CreateAgent("SplineAgent", splineDot, splineTrail);

            // Steering Agent
            _steeringAgent = CreateAgent("SteeringAgent", steeringDot, steeringTrail);
        }

        private PathAgent CreateAgent(string name, Color dotColor, Color trailColor)
        {
            var agent = new PathAgent();
            agent.dotColor = dotColor;
            agent.trailColor = trailColor;

            // 에이전트 오브젝트 생성
            agent.gameObject = new GameObject(name);
            agent.gameObject.transform.SetParent(transform);

            agent.spriteRenderer = agent.gameObject.AddComponent<SpriteRenderer>();

            // 흰색 원형 텍스처 생성 (색상은 SpriteRenderer.color로 적용)
            int size = 32;
            Texture2D tex = new Texture2D(size, size);
            Vector2 center = new Vector2(size / 2f, size / 2f);
            float radius = size / 2f - 1;

            for (int y = 0; y < size; y++)
            {
                for (int x = 0; x < size; x++)
                {
                    float dist = Vector2.Distance(new Vector2(x, y), center);
                    if (dist <= radius)
                        tex.SetPixel(x, y, Color.white);
                    else
                        tex.SetPixel(x, y, Color.clear);
                }
            }
            tex.Apply();

            agent.spriteRenderer.sprite = Sprite.Create(tex, new Rect(0, 0, size, size), new Vector2(0.5f, 0.5f), 32f);
            agent.spriteRenderer.color = dotColor;
            agent.spriteRenderer.sortingOrder = 100;
            agent.gameObject.transform.localScale = Vector3.one * AgentSize;
            agent.gameObject.SetActive(false);

            // 궤적 LineRenderer 생성
            var lineObj = new GameObject(name + "_Trail");
            lineObj.transform.SetParent(transform);
            agent.trailLine = lineObj.AddComponent<LineRenderer>();
            agent.trailLine.startWidth = TrailWidth;
            agent.trailLine.endWidth = TrailWidth;
            agent.trailLine.material = new Material(Shader.Find("Sprites/Default"));
            agent.trailLine.startColor = trailColor;
            agent.trailLine.endColor = trailColor;
            agent.trailLine.positionCount = 0;
            agent.trailLine.sortingOrder = 50;

            return agent;
        }

        private void CreateBackground()
        {
            if (!_createBackground) return;

            // 기존 배경 제거
            if (_backgroundObject != null)
            {
                DestroyImmediate(_backgroundObject);
            }

            // 배경 오브젝트 생성
            _backgroundObject = new GameObject("Background");
            _backgroundObject.transform.SetParent(transform);

            var spriteRenderer = _backgroundObject.AddComponent<SpriteRenderer>();

            // 1x1 흰색 텍스처 생성
            Texture2D tex = new Texture2D(1, 1);
            tex.SetPixel(0, 0, Color.white);
            tex.Apply();

            // 스프라이트 생성
            Sprite sprite = Sprite.Create(tex, new Rect(0, 0, 1, 1), new Vector2(0.5f, 0.5f), 1f);
            spriteRenderer.sprite = sprite;
            spriteRenderer.color = _backgroundColor;
            spriteRenderer.sortingOrder = -100;

            // 경계에 맞게 크기 조정
            Vector2 size = _boundaryMax - _boundaryMin;
            _backgroundObject.transform.localScale = new Vector3(size.x, size.y, 1f);
            _backgroundObject.transform.position = new Vector3(
                (_boundaryMin.x + _boundaryMax.x) / 2f,
                (_boundaryMin.y + _boundaryMax.y) / 2f,
                1f // z를 뒤로
            );
        }

        private void Update()
        {
            HandleMouseInput();
            HandleCameraControl();
            UpdateAllAgents();
            UpdateAgentColors();
        }

        /// <summary>
        /// Visualizer에서 색상 변경 시 실시간 반영
        /// </summary>
        private void UpdateAgentColors()
        {
            if (_visualizer == null) return;

            // Normal Agent 색상 업데이트
            UpdateAgentColor(_normalAgent, _visualizer.NormalAgentColor, _visualizer.NormalTrailColor);

            // Spline Agent 색상 업데이트
            UpdateAgentColor(_splineAgent, _visualizer.SplineAgentColor, _visualizer.SplineTrailColor);

            // Steering Agent 색상 업데이트
            UpdateAgentColor(_steeringAgent, _visualizer.SteeringAgentColor, _visualizer.SteeringTrailColor);
        }

        private void UpdateAgentColor(PathAgent agent, Color dotColor, Color trailColor)
        {
            if (agent == null) return;

            // 도트 색상 변경
            if (agent.spriteRenderer != null && agent.dotColor != dotColor)
            {
                agent.dotColor = dotColor;
                agent.spriteRenderer.color = dotColor;
            }

            // 트레일 색상 변경
            if (agent.trailLine != null && agent.trailColor != trailColor)
            {
                agent.trailColor = trailColor;
                agent.trailLine.startColor = trailColor;
                agent.trailLine.endColor = trailColor;
            }

            // 크기 업데이트
            if (agent.gameObject != null)
            {
                agent.gameObject.transform.localScale = Vector3.one * AgentSize;
            }

            // 트레일 두께 업데이트
            if (agent.trailLine != null)
            {
                agent.trailLine.startWidth = TrailWidth;
                agent.trailLine.endWidth = TrailWidth;
            }
        }

        private void HandleCameraControl()
        {
            var mouse = Mouse.current;
            var keyboard = Keyboard.current;
            var cam = Camera.main;

            if (cam == null) return;

            // 줌 (마우스 휠)
            if (mouse != null)
            {
                float scroll = mouse.scroll.ReadValue().y;
                if (scroll != 0)
                {
                    float newSize = cam.orthographicSize - scroll * _cameraZoomSpeed * Time.deltaTime * 50f;
                    cam.orthographicSize = Mathf.Clamp(newSize, _cameraZoomMin, _cameraZoomMax);
                }
            }

            // 이동 (WASD)
            if (keyboard != null)
            {
                Vector3 move = Vector3.zero;

                if (keyboard.wKey.isPressed) move.y += 1;
                if (keyboard.sKey.isPressed) move.y -= 1;
                if (keyboard.aKey.isPressed) move.x -= 1;
                if (keyboard.dKey.isPressed) move.x += 1;

                if (move != Vector3.zero)
                {
                    cam.transform.position += move.normalized * _cameraMoveSpeed * Time.deltaTime;
                }
            }
        }

        private void UpdateAllAgents()
        {
            UpdateNormalAgent();
            UpdateSplineAgent();
            UpdateSteeringAgent();
        }

        private void UpdateNormalAgent()
        {
            if (_normalAgent == null || !_normalAgent.isFollowing || _currentPath == null)
                return;

            if (_normalAgent.pathIndex >= _currentPath.Count - 1)
            {
                _normalAgent.isFollowing = false;
                return;
            }

            Vector2 currentPos = _normalAgent.gameObject.transform.position;
            Vector2 targetPos = _currentPath[_normalAgent.pathIndex + 1];

            float step = AgentSpeed * Time.deltaTime;
            Vector2 newPos = Vector2.MoveTowards(currentPos, targetPos, step);

            _normalAgent.gameObject.transform.position = new Vector3(newPos.x, newPos.y, -0.1f);
            RecordTrail(_normalAgent, newPos);

            if (Vector2.Distance(newPos, targetPos) < 0.01f)
            {
                _normalAgent.pathIndex++;
            }
        }

        private void UpdateSplineAgent()
        {
            if (_splineAgent == null || !_splineAgent.isFollowing || _splinePath == null)
                return;

            if (_splineAgent.pathIndex >= _splinePath.Count - 1)
            {
                _splineAgent.isFollowing = false;
                return;
            }

            Vector2 currentPos = _splineAgent.gameObject.transform.position;
            Vector2 targetPos = _splinePath[_splineAgent.pathIndex + 1];

            float step = AgentSpeed * Time.deltaTime;
            Vector2 newPos = Vector2.MoveTowards(currentPos, targetPos, step);

            _splineAgent.gameObject.transform.position = new Vector3(newPos.x, newPos.y, -0.15f);
            RecordTrail(_splineAgent, newPos);

            if (Vector2.Distance(newPos, targetPos) < 0.01f)
            {
                _splineAgent.pathIndex++;
            }
        }

        private void UpdateSteeringAgent()
        {
            if (_steeringAgent == null || !_steeringAgent.isFollowing || _currentPath == null)
                return;

            if (_steeringAgent.pathIndex >= _currentPath.Count - 1)
            {
                _steeringAgent.isFollowing = false;
                return;
            }

            Vector2 currentPos = _steeringAgent.gameObject.transform.position;
            Vector2 targetPos = _currentPath[_steeringAgent.pathIndex + 1];

            float step = AgentSpeed * Time.deltaTime;
            Vector2 newPos = Vector2.MoveTowards(currentPos, targetPos, step);

            _steeringAgent.gameObject.transform.position = new Vector3(newPos.x, newPos.y, -0.2f);
            RecordTrail(_steeringAgent, newPos);

            if (Vector2.Distance(newPos, targetPos) < 0.01f)
            {
                _steeringAgent.pathIndex++;
            }
        }

        private void RecordTrail(PathAgent agent, Vector2 pos)
        {
            if (agent.trail.Count == 0 || Vector2.Distance(agent.trail[agent.trail.Count - 1], pos) > 0.05f)
            {
                agent.trail.Add(pos);
                UpdateTrailVisualization(agent);
            }
        }

        private void UpdateTrailVisualization(PathAgent agent)
        {
            if (agent.trailLine == null)
                return;

            agent.trailLine.positionCount = agent.trail.Count;
            for (int i = 0; i < agent.trail.Count; i++)
            {
                agent.trailLine.SetPosition(i, new Vector3(agent.trail[i].x, agent.trail[i].y, -0.05f));
            }
        }

        private void StartPathFollowing(List<Vector2> path)
        {
            if (path == null || path.Count < 2)
                return;

            // 중복 점 제거 및 검증
            _currentPath = new List<Vector2>();
            _currentPath.Add(path[0]);

            for (int i = 1; i < path.Count; i++)
            {
                // 이전 점과 거리가 0.01 이상일 때만 추가
                if (Vector2.Distance(path[i], _currentPath[_currentPath.Count - 1]) > 0.01f)
                {
                    _currentPath.Add(path[i]);
                }
                else
                {
                    Debug.LogWarning($"[NavMesh2DDemo] Duplicate point removed at index {i}: {path[i]}");
                }
            }

            if (_currentPath.Count < 2)
            {
                Debug.LogError("[NavMesh2DDemo] Path has less than 2 unique points!");
                return;
            }

            Debug.Log($"[NavMesh2DDemo] Path ready! {_currentPath.Count} waypoints. Use ContextMenu to select Move mode.");
        }

        private void StopAgent(PathAgent agent)
        {
            if (agent == null) return;
            agent.isFollowing = false;
            agent.trail.Clear();
            if (agent.trailLine != null)
                agent.trailLine.positionCount = 0;
            if (agent.gameObject != null)
                agent.gameObject.SetActive(false);
        }

        private void StopAllAgents()
        {
            StopAgent(_normalAgent);
            StopAgent(_splineAgent);
            StopAgent(_steeringAgent);
        }

        private void ClearAllPaths()
        {
            StopAllAgents();
            _currentPath = null;
            _splinePath = null;
        }

        /// <summary>
        /// Normal 이동 (웨이포인트 직선 이동)
        /// </summary>
        [ContextMenu("Normal Move")]
        public void MoveByNormal()
        {
            if (_currentPath == null || _currentPath.Count < 2)
            {
                Debug.LogWarning("[NavMesh2DDemo] No path! Click two points first.");
                return;
            }

            StopAgent(_normalAgent);

            _normalAgent.pathIndex = 0;
            _normalAgent.isFollowing = true;
            _normalAgent.trail.Clear();
            _normalAgent.trail.Add(_currentPath[0]);

            _normalAgent.gameObject.transform.position = new Vector3(_currentPath[0].x, _currentPath[0].y, -0.1f);
            _normalAgent.gameObject.SetActive(true);

            Debug.Log($"[NavMesh2DDemo] Normal Move: {_currentPath.Count} waypoints (Black/Gray)");
        }

        /// <summary>
        /// Spline 이동 (Catmull-Rom 스플라인)
        /// </summary>
        [ContextMenu("Spline Move")]
        public void MoveBySpline()
        {
            if (_currentPath == null || _currentPath.Count < 2)
            {
                Debug.LogWarning("[NavMesh2DDemo] No path! Click two points first.");
                return;
            }

            StopAgent(_splineAgent);

            // Catmull-Rom 스플라인 생성
            _splinePath = GenerateCatmullRomPath(_currentPath, 10);

            _splineAgent.pathIndex = 0;
            _splineAgent.isFollowing = true;
            _splineAgent.trail.Clear();
            _splineAgent.trail.Add(_splinePath[0]);

            _splineAgent.gameObject.transform.position = new Vector3(_splinePath[0].x, _splinePath[0].y, -0.15f);
            _splineAgent.gameObject.SetActive(true);

            Debug.Log($"[NavMesh2DDemo] Spline Move: {_splinePath.Count} points (Blue/Cyan)");
        }

        /// <summary>
        /// Steering 이동 (부드러운 방향 전환)
        /// </summary>
        [ContextMenu("Steering Move")]
        public void MoveBySteering()
        {
            if (_currentPath == null || _currentPath.Count < 2)
            {
                Debug.LogWarning("[NavMesh2DDemo] No path! Click two points first.");
                return;
            }

            StopAgent(_steeringAgent);

            // 초기 방향 설정
            _steeringAgent.direction = (_currentPath[1] - _currentPath[0]).normalized;
            _steeringAgent.pathIndex = 0;
            _steeringAgent.isFollowing = true;
            _steeringAgent.trail.Clear();
            _steeringAgent.trail.Add(_currentPath[0]);

            _steeringAgent.gameObject.transform.position = new Vector3(_currentPath[0].x, _currentPath[0].y, -0.2f);
            _steeringAgent.gameObject.SetActive(true);

            Debug.Log($"[NavMesh2DDemo] Steering Move: {_currentPath.Count} waypoints (Red/Yellow)");
        }

        /// <summary>
        /// 세 가지 이동 방식 동시 비교
        /// </summary>
        [ContextMenu("Compare All Moves")]
        public void CompareAllMoves()
        {
            if (_currentPath == null || _currentPath.Count < 2)
            {
                Debug.LogWarning("[NavMesh2DDemo] No path! Click two points first.");
                return;
            }

            StopAllAgents();

            // 스플라인 생성
            _splinePath = GenerateCatmullRomPath(_currentPath, 10);

            // Normal Agent 시작
            _normalAgent.pathIndex = 0;
            _normalAgent.isFollowing = true;
            _normalAgent.trail.Clear();
            _normalAgent.trail.Add(_currentPath[0]);
            _normalAgent.gameObject.transform.position = new Vector3(_currentPath[0].x, _currentPath[0].y, -0.1f);
            _normalAgent.gameObject.SetActive(true);

            // Spline Agent 시작
            _splineAgent.pathIndex = 0;
            _splineAgent.isFollowing = true;
            _splineAgent.trail.Clear();
            _splineAgent.trail.Add(_splinePath[0]);
            _splineAgent.gameObject.transform.position = new Vector3(_splinePath[0].x, _splinePath[0].y, -0.15f);
            _splineAgent.gameObject.SetActive(true);

            // Steering Agent 시작
            _steeringAgent.direction = (_currentPath[1] - _currentPath[0]).normalized;
            _steeringAgent.pathIndex = 0;
            _steeringAgent.isFollowing = true;
            _steeringAgent.trail.Clear();
            _steeringAgent.trail.Add(_currentPath[0]);
            _steeringAgent.gameObject.transform.position = new Vector3(_currentPath[0].x, _currentPath[0].y, -0.2f);
            _steeringAgent.gameObject.SetActive(true);

            Debug.Log("[NavMesh2DDemo] Comparing: Normal(Black), Spline(Blue), Steering(Red)");
        }

        /// <summary>
        /// Catmull-Rom 스플라인 경로 생성
        /// </summary>
        private List<Vector2> GenerateCatmullRomPath(List<Vector2> points, int resolution)
        {
            var result = new List<Vector2>();

            if (points.Count < 2)
                return result;

            // 2개 점이면 직선 보간
            if (points.Count == 2)
            {
                for (int i = 0; i <= resolution; i++)
                {
                    float t = i / (float)resolution;
                    result.Add(Vector2.Lerp(points[0], points[1], t));
                }
                return result;
            }

            // 시작/끝에 가상의 컨트롤 포인트 추가
            var extended = new List<Vector2>();
            extended.Add(points[0] - (points[1] - points[0])); // 가상 시작점
            extended.AddRange(points);
            extended.Add(points[points.Count - 1] + (points[points.Count - 1] - points[points.Count - 2])); // 가상 끝점

            // 각 세그먼트에 대해 보간
            for (int i = 1; i < extended.Count - 2; i++)
            {
                Vector2 p0 = extended[i - 1];
                Vector2 p1 = extended[i];
                Vector2 p2 = extended[i + 1];
                Vector2 p3 = extended[i + 2];

                for (int j = 0; j < resolution; j++)
                {
                    float t = j / (float)resolution;
                    Vector2 pt = CatmullRomPoint(p0, p1, p2, p3, t);

                    // NaN 체크
                    if (!float.IsNaN(pt.x) && !float.IsNaN(pt.y))
                        result.Add(pt);
                }
            }

            // 마지막 점 추가
            result.Add(points[points.Count - 1]);

            return result;
        }

        /// <summary>
        /// Uniform Catmull-Rom 보간
        /// </summary>
        private Vector2 CatmullRomPoint(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t)
        {
            float t2 = t * t;
            float t3 = t2 * t;

            // Catmull-Rom 행렬 (tension = 0.5)
            return 0.5f * (
                (2f * p1) +
                (-p0 + p2) * t +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
                (-p0 + 3f * p1 - 3f * p2 + p3) * t3
            );
        }

        private void HandleMouseInput()
        {
            var mouse = Mouse.current;
            var keyboard = Keyboard.current;

            if (mouse == null) return;

            // 좌클릭: 시작점/끝점 설정
            if (mouse.leftButton.wasPressedThisFrame)
            {
                Vector2 worldPos = GetMouseWorldPosition();

                // 1. 보드(boundary) 밖 클릭 무시
                if (!IsPointInsideBoundary(worldPos))
                {
                    Debug.Log("[NavMesh2DDemo] Click outside boundary - ignored");
                    return;
                }

                // 2. 장애물 내부 클릭 시 가장 가까운 안전한 위치로 보정
                Vector2 safePos = GetSafePosition(worldPos);

                if (_isSettingStart)
                {
                    _startPoint = safePos;
                    if (safePos != worldPos)
                        Debug.Log($"[NavMesh2DDemo] Start point adjusted: {worldPos} -> {safePos}");
                    else
                        Debug.Log($"[NavMesh2DDemo] Start point set: {safePos}");
                }
                else
                {
                    _endPoint = safePos;
                    if (safePos != worldPos)
                        Debug.Log($"[NavMesh2DDemo] End point adjusted: {worldPos} -> {safePos}");
                    else
                        Debug.Log($"[NavMesh2DDemo] End point set: {safePos}");
                    FindAndVisualizePath();
                }

                _isSettingStart = !_isSettingStart;
            }

            // 우클릭: 경로 초기화
            if (mouse.rightButton.wasPressedThisFrame)
            {
                _visualizer?.ClearPath();
                ClearAllPaths();
                _isSettingStart = true;
                Debug.Log("[NavMesh2DDemo] Path cleared");
            }

            // R 키: NavMesh 재빌드
            if (keyboard != null && keyboard.rKey.wasPressedThisFrame)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 점이 boundary 내부에 있는지 확인
        /// </summary>
        private bool IsPointInsideBoundary(Vector2 point)
        {
            return point.x >= _boundaryMin.x && point.x <= _boundaryMax.x &&
                   point.y >= _boundaryMin.y && point.y <= _boundaryMax.y;
        }

        /// <summary>
        /// 장애물 내부라면 가장 가까운 안전한 위치 반환
        /// </summary>
        private Vector2 GetSafePosition(Vector2 point)
        {
            if (_query == null)
                return point;

            Vector2Fixed fixedPoint = new Vector2Fixed((Fixed64)point.x, (Fixed64)point.y);

            // NavMesh 위에 있으면 그대로 반환
            if (_query.IsPointOnNavMesh(fixedPoint))
                return point;

            // NavMesh 위의 가장 가까운 점으로 보정
            Vector2Fixed clampedPoint = _query.ClampToNavMesh(fixedPoint);
            return new Vector2((float)clampedPoint.x, (float)clampedPoint.y);
        }

        private Vector2 GetMouseWorldPosition()
        {
            Vector2 mousePos = Mouse.current.position.ReadValue();
            Vector3 worldPos = Camera.main.ScreenToWorldPoint(new Vector3(mousePos.x, mousePos.y, -Camera.main.transform.position.z));
            return worldPos;
        }

        private bool IsPointInsideObstacle(Vector2 point)
        {
            if (_obstacles == null) return false;

            foreach (var obstacle in _obstacles)
            {
                if (obstacle == null || obstacle.vertices == null || obstacle.vertices.Length < 3)
                    continue;

                if (IsPointInPolygon(point, obstacle.vertices))
                    return true;
            }
            return false;
        }

        private bool IsPointInPolygon(Vector2 point, Vector2[] polygon)
        {
            // Ray Casting Algorithm
            int count = polygon.Length;
            bool inside = false;

            for (int i = 0, j = count - 1; i < count; j = i++)
            {
                Vector2 vi = polygon[i];
                Vector2 vj = polygon[j];

                if ((vi.y > point.y) != (vj.y > point.y) &&
                    point.x < (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x)
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        /// <summary>
        /// NavMesh 빌드
        /// </summary>
        [ContextMenu("Rebuild NavMesh")]
        public void RebuildNavMesh()
        {
            if (_builder == null)
            {
                _builder = new NavMeshBuilder();
            }

            // 장애물 변환
            List<Polygon2D> obstacles = new List<Polygon2D>();
            if (_obstacles == null)
            {
                _obstacles = new List<PolygonObstacle>();
            }
            foreach (var obs in _obstacles)
            {
                if (obs != null && obs.vertices != null && obs.vertices.Length >= 3)
                {
                    Polygon2D poly = new Polygon2D();
                    foreach (var v in obs.vertices)
                    {
                        poly.AddVertex(new Vector2Fixed((Fixed64)v.x, (Fixed64)v.y));
                    }
                    obstacles.Add(poly);
                }
            }

            // 빌드
            _navMesh = _builder.BuildFromRect(_boundaryMin, _boundaryMax, obstacles.Count > 0 ? obstacles : null);

            if (_navMesh != null)
            {
                _query = new NavMeshQuery(_navMesh);

                if (_visualizer != null)
                {
                    _visualizer.NavMesh = _navMesh;
                    _visualizer.ClearPath();
                }

                Debug.Log($"[NavMesh2DDemo] NavMesh rebuilt: {_navMesh.TriangleCount} triangles");
            }
            else
            {
                Debug.LogError("[NavMesh2DDemo] Failed to build NavMesh");
            }
        }

        /// <summary>
        /// 경로 탐색 및 시각화
        /// </summary>
        private void FindAndVisualizePath()
        {
            if (_query == null)
            {
                Debug.LogError("[NavMesh2DDemo] NavMesh not built!");
                return;
            }

            Vector2Fixed start = new Vector2Fixed((Fixed64)_startPoint.x, (Fixed64)_startPoint.y);
            Vector2Fixed end = new Vector2Fixed((Fixed64)_endPoint.x, (Fixed64)_endPoint.y);

            var result = _query.FindPath(start, end);

            if (result.Success)
            {
                Debug.Log($"[NavMesh2DDemo] Path found: {result.Path.Count} waypoints, length: {(float)result.PathLength:F2}");

                if (_visualizer != null)
                {
                    _visualizer.SetPath(result.Path, result.Portals);
                }

                // 경로를 Vector2 리스트로 변환하고 에이전트 이동 시작
                var pathVector2 = new List<Vector2>();
                foreach (var p in result.Path)
                {
                    pathVector2.Add(new Vector2((float)p.x, (float)p.y));
                }
                StartPathFollowing(pathVector2);
            }
            else
            {
                Debug.LogWarning("[NavMesh2DDemo] Path not found!");
                _visualizer?.ClearPath();
                ClearAllPaths();
            }
        }

        /// <summary>
        /// 사각형 장애물 추가
        /// </summary>
        [ContextMenu("Obstacles/Add Rectangle")]
        public void AddRectangleObstacle()
        {
            if (_obstacles == null) _obstacles = new List<PolygonObstacle>();

            var obstacle = new PolygonObstacle
            {
                name = $"Rectangle {_obstacles.Count}",
                vertices = new Vector2[]
                {
                    new Vector2(-2, -1),
                    new Vector2(2, -1),
                    new Vector2(2, 1),
                    new Vector2(-2, 1)
                }
            };
            _obstacles.Add(obstacle);

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 삼각형 장애물 추가
        /// </summary>
        [ContextMenu("Obstacles/Add Triangle")]
        public void AddTriangleObstacle()
        {
            if (_obstacles == null) _obstacles = new List<PolygonObstacle>();

            var obstacle = new PolygonObstacle
            {
                name = $"Triangle {_obstacles.Count}",
                vertices = new Vector2[]
                {
                    new Vector2(0, 2),
                    new Vector2(-2, -1),
                    new Vector2(2, -1)
                }
            };
            _obstacles.Add(obstacle);

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 자유 폴리곤 장애물 추가 (오각형 시작)
        /// </summary>
        [ContextMenu("Obstacles/Add Polygon")]
        public void AddPolygonObstacle()
        {
            if (_obstacles == null) _obstacles = new List<PolygonObstacle>();

            // 오각형으로 시작 (자유롭게 편집 가능)
            float radius = 1.5f;
            int segments = 5;
            Vector2[] verts = new Vector2[segments];

            for (int i = 0; i < segments; i++)
            {
                float angle = (i / (float)segments) * Mathf.PI * 2f - Mathf.PI / 2f; // 위쪽부터 시작
                verts[i] = new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
            }

            var obstacle = new PolygonObstacle
            {
                name = $"Polygon {_obstacles.Count}",
                vertices = verts
            };
            _obstacles.Add(obstacle);

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 원형 장애물 추가 (8각형 근사)
        /// </summary>
        [ContextMenu("Obstacles/Add Circle (8-gon)")]
        public void AddCircleObstacle()
        {
            if (_obstacles == null) _obstacles = new List<PolygonObstacle>();

            float radius = 2f;
            int segments = 8;
            Vector2[] verts = new Vector2[segments];

            for (int i = 0; i < segments; i++)
            {
                float angle = (i / (float)segments) * Mathf.PI * 2f;
                verts[i] = new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
            }

            var obstacle = new PolygonObstacle
            {
                name = $"Circle {_obstacles.Count}",
                vertices = verts
            };
            _obstacles.Add(obstacle);

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 모든 장애물 제거
        /// </summary>
        [ContextMenu("Obstacles/Clear All")]
        public void ClearAllObstacles()
        {
            if (_obstacles != null)
            {
                _obstacles.Clear();
            }

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 마지막 장애물 제거
        /// </summary>
        [ContextMenu("Obstacles/Remove Last")]
        public void RemoveLastObstacle()
        {
            if (_obstacles != null && _obstacles.Count > 0)
            {
                _obstacles.RemoveAt(_obstacles.Count - 1);
            }

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        private void OnDrawGizmos()
        {
            // 경계 표시
            Gizmos.color = Color.white;
            Vector3 min = new Vector3(_boundaryMin.x, _boundaryMin.y, 0);
            Vector3 max = new Vector3(_boundaryMax.x, _boundaryMax.y, 0);
            Gizmos.DrawLine(new Vector3(min.x, min.y, 0), new Vector3(max.x, min.y, 0));
            Gizmos.DrawLine(new Vector3(max.x, min.y, 0), new Vector3(max.x, max.y, 0));
            Gizmos.DrawLine(new Vector3(max.x, max.y, 0), new Vector3(min.x, max.y, 0));
            Gizmos.DrawLine(new Vector3(min.x, max.y, 0), new Vector3(min.x, min.y, 0));

            // 장애물 표시
            Gizmos.color = new Color(1, 0, 0, 0.5f);
            foreach (var obs in _obstacles)
            {
                if (obs != null && obs.vertices != null && obs.vertices.Length >= 3)
                {
                    for (int i = 0; i < obs.vertices.Length; i++)
                    {
                        int next = (i + 1) % obs.vertices.Length;
                        Vector3 a = new Vector3(obs.vertices[i].x, obs.vertices[i].y, 0);
                        Vector3 b = new Vector3(obs.vertices[next].x, obs.vertices[next].y, 0);
                        Gizmos.DrawLine(a, b);
                    }
                }
            }

            // 시작점/끝점 표시
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(new Vector3(_startPoint.x, _startPoint.y, 0), 0.2f);

            Gizmos.color = Color.red;
            Gizmos.DrawSphere(new Vector3(_endPoint.x, _endPoint.y, 0), 0.2f);
        }

        private void OnValidate()
        {
            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }
    }

    /// <summary>
    /// 폴리곤 장애물 정의
    /// </summary>
    [System.Serializable]
    public class PolygonObstacle
    {
        public string name;
        public Vector2[] vertices;
    }
}
