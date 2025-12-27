using System.Collections.Generic;
using System.Diagnostics;
using FixedMathSharp;
using NavMesh2D.Geometry;
using NavMesh2D.Pathfinding;
using UnityEngine;
using UnityEngine.InputSystem;
using Debug = UnityEngine.Debug;

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
        [SerializeField] private int _maxTriangleCount = 0; // 0이면 세분화 안함

        [Header("=== Obstacles ===")]
        [SerializeField] private List<PolygonObstacle> _obstacles = new List<PolygonObstacle>();

        [Header("=== Walkable Polygons (메시 품질 개선용) ===")]
        [SerializeField] private List<PolygonObstacle> _walkables = new List<PolygonObstacle>();

        [Header("=== Components ===")]
        [SerializeField] private NavMesh2DVisualizer _visualizer;

        [Header("=== NavMesh Asset ===")]
        [SerializeField] private NavMesh2DData _navMeshAsset;

        [Header("=== Demo Mode ===")]
        [Tooltip("false: Point Mode (시작/끝점 클릭), true: Agent Mode (에이전트가 클릭 위치로 이동)")]
        [SerializeField] private bool _useAgentMode = false;
        [SerializeField] private Transform _agent;
        [SerializeField] private float _agentMoveSpeed = 5f;

        [Header("=== Debug ===")]
        [SerializeField] private bool _autoRebuildOnChange = true;
        [SerializeField] private Vector2 _startPoint;
        [SerializeField] private Vector2 _endPoint;

        [Header("=== Camera Control ===")]
        [SerializeField] private float _cameraZoomMin = 1.5f;
        [SerializeField] private float _cameraZoomMax = 10f;
        [SerializeField] private float _cameraZoomSpeed = 2f;
        [SerializeField] private float _cameraMoveSpeed = 10f;

        private NavMesh2DData _navMesh;
        private NavMeshQuery _query;
        private NavMeshBuilder _builder;

        private bool _isSettingStart = true; // true: 시작점 설정 중, false: 끝점 설정 중

        // Agent Mode 런타임 변수
        private bool _isAgentMode;  // Start에서 한 번만 읽음
        private List<Vector2> _agentPath;
        private int _agentPathIndex;
        private bool _isAgentMoving;

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
            // Fixed64 상수 확인 (최적화용)
            Debug.Log($"[Fixed64] ONE = {Fixed64.One.m_rawValue}");

            _builder = new NavMeshBuilder();
            CreateBackground();
            CreateAgents();
            RebuildNavMesh();

            // 모드 설정 (한 번만 읽음)
            _isAgentMode = _useAgentMode;

            // Agent Mode 유효성 검사
            if (_isAgentMode && _agent == null)
            {
                Debug.LogError("[NavMesh2DDemo] Agent Mode가 활성화되었지만 Agent가 할당되지 않았습니다. Point Mode로 전환합니다.");
                _isAgentMode = false;
            }

            if (_isAgentMode)
            {
                Debug.Log("[NavMesh2DDemo] Agent Mode 활성화 - 좌클릭으로 에이전트 이동");
            }
            else
            {
                Debug.Log("[NavMesh2DDemo] Point Mode 활성화 - 좌클릭으로 시작/끝점 설정");
            }
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

            // Agent Mode 이동 처리
            if (_isAgentMode)
            {
                UpdateAgentMovement();
            }
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

            // 좌클릭 처리
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

                if (_isAgentMode)
                {
                    // Agent Mode: 클릭 위치로 에이전트 이동
                    HandleAgentModeClick(safePos);
                }
                else
                {
                    // Point Mode: 시작점/끝점 설정
                    HandlePointModeClick(worldPos, safePos);
                }
            }

            // Point Mode에서만 우클릭 허용
            if (!_isAgentMode && mouse.rightButton.wasPressedThisFrame)
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
        /// Point Mode: 시작점/끝점 설정
        /// </summary>
        private void HandlePointModeClick(Vector2 worldPos, Vector2 safePos)
        {
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

        /// <summary>
        /// Agent Mode: 클릭 위치로 에이전트 이동
        /// </summary>
        private void HandleAgentModeClick(Vector2 destination)
        {
            if (_agent == null || _query == null)
                return;

            // 현재 에이전트 위치를 시작점으로
            Vector2 agentPos = _agent.position;
            _startPoint = GetSafePosition(agentPos);
            _endPoint = destination;

            // 경로 찾기
            Vector2Fixed start = new Vector2Fixed((Fixed64)_startPoint.x, (Fixed64)_startPoint.y);
            Vector2Fixed end = new Vector2Fixed((Fixed64)_endPoint.x, (Fixed64)_endPoint.y);

            var result = _query.FindPath(start, end);

            if (result.Success && result.Path.Count >= 2)
            {
                // 경로 시각화
                if (_visualizer != null)
                {
                    _visualizer.SetPath(result.Path, result.Portals, result.TrianglePath);
                }

                // 경로를 Vector2 리스트로 변환
                _agentPath = new List<Vector2>();
                foreach (var p in result.Path)
                {
                    _agentPath.Add(new Vector2((float)p.x, (float)p.y));
                }

                // 이동 시작
                _agentPathIndex = 0;
                _isAgentMoving = true;

                Debug.Log($"[NavMesh2DDemo] Agent moving to {destination}, path waypoints: {_agentPath.Count}");
            }
            else
            {
                Debug.LogWarning($"[NavMesh2DDemo] No path found to {destination}");
            }
        }

        /// <summary>
        /// Agent Mode: 에이전트 이동 업데이트
        /// </summary>
        private void UpdateAgentMovement()
        {
            if (!_isAgentMoving || _agent == null || _agentPath == null || _agentPath.Count == 0)
                return;

            // 다음 웨이포인트까지 이동
            if (_agentPathIndex >= _agentPath.Count - 1)
            {
                // 목적지 도착
                _isAgentMoving = false;
                Debug.Log("[NavMesh2DDemo] Agent arrived at destination");
                return;
            }

            Vector2 currentPos = _agent.position;
            Vector2 targetPos = _agentPath[_agentPathIndex + 1];

            float step = _agentMoveSpeed * Time.deltaTime;
            Vector2 newPos = Vector2.MoveTowards(currentPos, targetPos, step);

            _agent.position = new Vector3(newPos.x, newPos.y, _agent.position.z);

            // 웨이포인트 도착 체크
            if (Vector2.Distance(newPos, targetPos) < 0.01f)
            {
                _agentPathIndex++;
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
        /// 장애물 내부라면 가장 가까운 안전한 위치 반환 (Fixed64)
        /// </summary>
        private Vector2Fixed GetSafePositionFixed(Vector2Fixed point)
        {
            if (_query == null)
                return point;

            // NavMesh 위에 있으면 그대로 반환
            if (_query.IsPointOnNavMesh(point))
                return point;

            // NavMesh 위의 가장 가까운 점으로 보정
            return _query.ClampToNavMesh(point);
        }

        /// <summary>
        /// 장애물 내부라면 가장 가까운 안전한 위치 반환 (UI/입력용)
        /// </summary>
        private Vector2 GetSafePosition(Vector2 point)
        {
            Vector2Fixed fixedPoint = new Vector2Fixed((Fixed64)point.x, (Fixed64)point.y);
            Vector2Fixed result = GetSafePositionFixed(fixedPoint);
            return new Vector2((float)result.x, (float)result.y);
        }

        private Vector2 GetMouseWorldPosition()
        {
            Vector2 mousePos = Mouse.current.position.ReadValue();
            Vector3 worldPos = Camera.main.ScreenToWorldPoint(new Vector3(mousePos.x, mousePos.y, -Camera.main.transform.position.z));
            return worldPos;
        }

        /// <summary>
        /// Inspector에서 입력한 시작/끝점으로 경로 계산
        /// </summary>
        [ContextMenu("Calculate Path")]
        public void CalculatePath()
        {
            if (_query == null)
            {
                Debug.LogError("[NavMesh2DDemo] NavMesh not built! Build first.");
                return;
            }
            FindAndVisualizePath();
        }

        /// <summary>
        /// 시작점/끝점 스왑
        /// </summary>
        [ContextMenu("Swap Start/End")]
        public void SwapStartEnd()
        {
            (_startPoint, _endPoint) = (_endPoint, _startPoint);
            Debug.Log($"[NavMesh2DDemo] Swapped: Start={_startPoint}, End={_endPoint}");
        }

        /// <summary>
        /// 경로 정확도 테스트 (10만회)
        /// </summary>
        [ContextMenu("Path Accuracy Test (100k)")]
        public void PathAccuracyTest()
        {
            if (_query == null)
            {
                Debug.LogError("[NavMesh2DDemo] NavMesh not built!");
                return;
            }

            const int ITERATIONS = 1000;
            var random = new System.Random(42);

            // boundary를 Fixed64로 변환
            Fixed64 minX = (Fixed64)_boundaryMin.x;
            Fixed64 minY = (Fixed64)_boundaryMin.y;
            Fixed64 rangeX = (Fixed64)_boundaryMax.x - minX;
            Fixed64 rangeY = (Fixed64)_boundaryMax.y - minY;

            int successCount = 0;
            int failCount = 0;
            int clampedCount = 0;
            long findPathTotalMs = 0;
            Fixed64 totalRatio = Fixed64.Zero;
            Fixed64 maxRatio = Fixed64.Zero;
            Fixed64 minRatio = Fixed64.MAX_VALUE;
            int worstCaseIndex = -1;
            Vector2Fixed worstStart = Vector2Fixed.Zero, worstEnd = Vector2Fixed.Zero;

            var sw = Stopwatch.StartNew();

            for (int i = 0; i < ITERATIONS; i++)
            {
                // 랜덤 시작/끝점 생성 (Fixed64로 직접)
                Fixed64 startX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 startY = minY + (Fixed64)random.NextDouble() * rangeY;
                Fixed64 endX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 endY = minY + (Fixed64)random.NextDouble() * rangeY;

                Vector2Fixed start = new Vector2Fixed(startX, startY);
                Vector2Fixed end = new Vector2Fixed(endX, endY);

                // NavMesh 밖의 점은 가장 가까운 유효 위치로 클램핑
                int startTri = _navMesh.FindTriangleContainingPoint(start);
                int endTri = _navMesh.FindTriangleContainingPoint(end);

                if (startTri < 0)
                {
                    start = _query.ClampToNavMesh(start);
                    clampedCount++;
                }
                if (endTri < 0)
                {
                    end = _query.ClampToNavMesh(end);
                    clampedCount++;
                }

                // 직선 거리
                Fixed64 straightDist = Vector2Fixed.Distance(start, end);

                // 경로 탐색 (시간 측정)
                var pathSw = Stopwatch.StartNew();
                var result = _query.FindPath(start, end);
                pathSw.Stop();
                findPathTotalMs += pathSw.ElapsedTicks;

                if (!result.Success)
                {
                    failCount++;
                    if (failCount <= 5) // 처음 5개 실패만 로그
                    {
                        Debug.LogWarning($"[PathAccuracyTest] Fail #{failCount}: Start=({(float)start.x:F4},{(float)start.y:F4}), End=({(float)end.x:F4},{(float)end.y:F4})");
                    }
                    continue;
                }

                successCount++;

                // 같은 점이면 ratio 계산 스킵 (0으로 나누기 방지)
                if (straightDist == Fixed64.Zero)
                    continue;

                // Funnel 경로 거리
                Fixed64 funnelDist = result.PathLength;

                // 비율 계산 (funnel / straight)
                Fixed64 ratio = funnelDist / straightDist;
                totalRatio += ratio;

                // 비정상적으로 높은 비율 디버깅
                if (ratio > (Fixed64)5.0)
                {
                    Debug.LogWarning($"[PathAccuracyTest] High ratio {(float)ratio:F2} at iter {i}: straight={(float)straightDist:F4}, funnel={(float)funnelDist:F4}, triPath=[{string.Join(",", result.TrianglePath)}], waypoints={result.Path.Count}");
                }

                if (ratio > maxRatio)
                {
                    maxRatio = ratio;
                    worstCaseIndex = i;
                    worstStart = start;
                    worstEnd = end;

                    // worst case 발견 시 즉시 상세 정보 출력
                    int finalStartTri = _navMesh.FindTriangleContainingPoint(start);
                    int finalEndTri = _navMesh.FindTriangleContainingPoint(end);
                    Debug.Log($"[PathAccuracyTest] New worst case #{i}: ratio={ratio}, startTri={finalStartTri}, endTri={finalEndTri}, triPath=[{string.Join(",", result.TrianglePath)}]");
                }
                if (ratio < minRatio)
                {
                    minRatio = ratio;
                }
            }

            sw.Stop();

            Fixed64 avgRatio = successCount > 0 ? totalRatio / (Fixed64)successCount : Fixed64.Zero;

            int testedCount = successCount + failCount;
            double findPathMs = (double)findPathTotalMs / Stopwatch.Frequency * 1000.0;
            double avgFindPathMs = findPathMs / testedCount;
            Fixed64 worstStraight = Vector2Fixed.Distance(worstStart, worstEnd);
            Debug.Log($"[PathAccuracyTest] {ITERATIONS} iterations, {sw.ElapsedMilliseconds}ms (FindPath only: {findPathMs:F0}ms, avg: {avgFindPathMs:F3}ms/path)\n" +
                      $"  Clamped: {clampedCount} points\n" +
                      $"  Tested: {testedCount}, Success: {successCount}, Fail: {failCount}\n" +
                      $"  Path/Straight Ratio - Avg: {(float)avgRatio:F4}, Min: {(float)minRatio:F4}, Max: {(float)maxRatio:F4}\n" +
                      $"  Worst case: Start=({(float)worstStart.x:F6},{(float)worstStart.y:F6}), End=({(float)worstEnd.x:F6},{(float)worstEnd.y:F6})\n" +
                      $"  Worst straight dist: {(float)worstStraight:F6}, ratio: {(float)maxRatio:F4}");

            // Worst case를 시작/끝점에 설정 (UI용 float 변환은 여기서만)
            if (worstCaseIndex >= 0)
            {
                _startPoint = new Vector2((float)worstStart.x, (float)worstStart.y);
                _endPoint = new Vector2((float)worstEnd.x, (float)worstEnd.y);
                Debug.Log($"[PathAccuracyTest] Worst case set to Start/End points. Use 'Calculate Path' to visualize.");
            }
        }

        /// <summary>
        /// 개별 경로 탐색 로그 테스트 (1000회, 각각 로그)
        /// </summary>
        [ContextMenu("Path Detail Test (1000 logs)")]
        public void PathDetailTest()
        {
            if (_query == null)
            {
                Debug.LogError("[NavMesh2DDemo] NavMesh not built!");
                return;
            }

            const int ITERATIONS = 1000;
            var random = new System.Random(42);

            Fixed64 minX = (Fixed64)_boundaryMin.x;
            Fixed64 minY = (Fixed64)_boundaryMin.y;
            Fixed64 rangeX = (Fixed64)_boundaryMax.x - minX;
            Fixed64 rangeY = (Fixed64)_boundaryMax.y - minY;

            // 누적 타이밍
            long totalFindTri = 0, totalExtract = 0, totalGetTri = 0;
            long totalMove = 0, totalHeur = 0, totalHeap = 0;
            int successCount = 0;

            double freq = Stopwatch.Frequency / 1000.0;

            for (int i = 0; i < ITERATIONS; i++)
            {
                Fixed64 startX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 startY = minY + (Fixed64)random.NextDouble() * rangeY;
                Fixed64 endX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 endY = minY + (Fixed64)random.NextDouble() * rangeY;

                Vector2Fixed start = new Vector2Fixed(startX, startY);
                Vector2Fixed end = new Vector2Fixed(endX, endY);

                int startTri = _navMesh.FindTriangleContainingPoint(start);
                int endTri = _navMesh.FindTriangleContainingPoint(end);

                if (startTri < 0)
                    start = _query.ClampToNavMesh(start);
                if (endTri < 0)
                    end = _query.ClampToNavMesh(end);

                // A* 시간만 측정
                var astarResult = _query.FindPathAStarOnly(start, end);

                if (astarResult.Success)
                {
                    successCount++;
                    totalFindTri += astarResult.TicksFindTriangle;
                    totalExtract += astarResult.TicksExtractMin;
                    totalGetTri += astarResult.TicksGetTriangle;
                    totalMove += astarResult.TicksMoveCost;
                    totalHeur += astarResult.TicksHeuristic;
                    totalHeap += astarResult.TicksHeapOps;
                }
            }

            // 결과 출력
            double findTriMs = totalFindTri / freq;
            double extractMs = totalExtract / freq;
            double getTriMs = totalGetTri / freq;
            double moveMs = totalMove / freq;
            double heurMs = totalHeur / freq;
            double heapMs = totalHeap / freq;
            double totalMs = findTriMs + extractMs + getTriMs + moveMs + heurMs + heapMs;
            double avgMs = totalMs / successCount;

            Debug.Log($"[PathDetailTest] {successCount}/{ITERATIONS} succeeded\n" +
                $"  Total A* time: {totalMs:F2}ms (avg: {avgMs:F4}ms/path)\n" +
                $"  FindTri: {findTriMs:F2}ms ({findTriMs/totalMs*100:F1}%)\n" +
                $"  Extract: {extractMs:F2}ms ({extractMs/totalMs*100:F1}%)\n" +
                $"  GetTri:  {getTriMs:F2}ms ({getTriMs/totalMs*100:F1}%)\n" +
                $"  Move:    {moveMs:F2}ms ({moveMs/totalMs*100:F1}%)\n" +
                $"  Heur:    {heurMs:F2}ms ({heurMs/totalMs*100:F1}%)\n" +
                $"  Heap:    {heapMs:F2}ms ({heapMs/totalMs*100:F1}%)");
        }

        /// <summary>
        /// 순수 벤치마크 (내부 타이밍 없이 외부에서만 측정)
        /// </summary>
        [ContextMenu("Pure Benchmark (1000)")]
        public void PureBenchmark()
        {
            if (_query == null)
            {
                Debug.LogError("[NavMesh2DDemo] NavMesh not built!");
                return;
            }

            const int ITERATIONS = 1000;
            var random = new System.Random(42);

            Fixed64 minX = (Fixed64)_boundaryMin.x;
            Fixed64 minY = (Fixed64)_boundaryMin.y;
            Fixed64 rangeX = (Fixed64)_boundaryMax.x - minX;
            Fixed64 rangeY = (Fixed64)_boundaryMax.y - minY;

            // 테스트 케이스 미리 생성
            var testCases = new List<(Vector2Fixed start, Vector2Fixed end)>();
            for (int i = 0; i < ITERATIONS; i++)
            {
                Fixed64 startX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 startY = minY + (Fixed64)random.NextDouble() * rangeY;
                Fixed64 endX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 endY = minY + (Fixed64)random.NextDouble() * rangeY;

                var start = new Vector2Fixed(startX, startY);
                var end = new Vector2Fixed(endX, endY);

                int startTri = _navMesh.FindTriangleContainingPoint(start);
                int endTri = _navMesh.FindTriangleContainingPoint(end);

                if (startTri < 0)
                    start = _query.ClampToNavMesh(start);
                if (endTri < 0)
                    end = _query.ClampToNavMesh(end);

                testCases.Add((start, end));
            }

            // 순수 A* 벤치마크
            int successCount = 0;
            var sw = Stopwatch.StartNew();
            for (int i = 0; i < ITERATIONS; i++)
            {
                var result = _query.FindPathAStarOnly(testCases[i].start, testCases[i].end);
                if (result.Success) successCount++;
            }
            sw.Stop();
            double astarMs = sw.Elapsed.TotalMilliseconds;

            // 전체 FindPath 벤치마크 (A* + Funnel)
            sw.Restart();
            for (int i = 0; i < ITERATIONS; i++)
            {
                var result = _query.FindPath(testCases[i].start, testCases[i].end);
            }
            sw.Stop();
            double fullMs = sw.Elapsed.TotalMilliseconds;

            Debug.Log($"[PureBenchmark] {successCount}/{ITERATIONS} succeeded\n" +
                $"  A* only:    {astarMs:F2}ms (avg: {astarMs/ITERATIONS:F4}ms/path)\n" +
                $"  Full path:  {fullMs:F2}ms (avg: {fullMs/ITERATIONS:F4}ms/path)");
        }

        /// <summary>
        /// A* + Funnel 분리 측정
        /// </summary>
        [ContextMenu("A* + Funnel Breakdown (1000)")]
        public void AStarFunnelBreakdown()
        {
            if (_query == null)
            {
                Debug.LogError("[NavMesh2DDemo] NavMesh not built!");
                return;
            }

            const int ITERATIONS = 1000;
            var random = new System.Random(42);

            Fixed64 minX = (Fixed64)_boundaryMin.x;
            Fixed64 minY = (Fixed64)_boundaryMin.y;
            Fixed64 rangeX = (Fixed64)_boundaryMax.x - minX;
            Fixed64 rangeY = (Fixed64)_boundaryMax.y - minY;

            long totalAStar = 0, totalFunnel = 0;
            int successCount = 0;

            for (int i = 0; i < ITERATIONS; i++)
            {
                Fixed64 startX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 startY = minY + (Fixed64)random.NextDouble() * rangeY;
                Fixed64 endX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 endY = minY + (Fixed64)random.NextDouble() * rangeY;

                var start = new Vector2Fixed(startX, startY);
                var end = new Vector2Fixed(endX, endY);

                int startTri = _navMesh.FindTriangleContainingPoint(start);
                int endTri = _navMesh.FindTriangleContainingPoint(end);

                if (startTri < 0)
                    start = _query.ClampToNavMesh(start);
                if (endTri < 0)
                    end = _query.ClampToNavMesh(end);

                var result = _query.FindPath(start, end);

                if (result.Success)
                {
                    successCount++;
                    totalAStar += result.TicksAStar;
                    totalFunnel += result.TicksFunnel;
                }
            }

            double freq = Stopwatch.Frequency / 1000.0;
            double astarMs = totalAStar / freq;
            double funnelMs = totalFunnel / freq;
            double totalMs = astarMs + funnelMs;

            Debug.Log($"[A*+Funnel] {successCount}/{ITERATIONS} succeeded\n" +
                $"  A*:     {astarMs:F2}ms ({astarMs/totalMs*100:F1}%) avg: {astarMs/successCount:F4}ms\n" +
                $"  Funnel: {funnelMs:F2}ms ({funnelMs/totalMs*100:F1}%) avg: {funnelMs/successCount:F4}ms\n" +
                $"  Total:  {totalMs:F2}ms avg: {totalMs/successCount:F4}ms");
        }

        // 버텍스 스냅 허용 거리 (이 거리 이내의 정점은 같은 정점으로 통합)
        private const float VERTEX_SNAP_THRESHOLD = 0.0001f;

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

            // 장애물 변환 (버텍스 스냅 적용)
            List<Polygon2D> obstacles = new List<Polygon2D>();
            if (_obstacles == null)
            {
                _obstacles = new List<PolygonObstacle>();
            }

            // 1. 모든 정점을 수집하고 스냅
            List<Vector2Fixed> canonicalVertices = new List<Vector2Fixed>();

            foreach (var obs in _obstacles)
            {
                if (obs != null && obs.vertices != null && obs.vertices.Length >= 3)
                {
                    Polygon2D poly = new Polygon2D();
                    poly.Name = obs.name;

                    foreach (var v in obs.vertices)
                    {
                        Vector2Fixed vertex = new Vector2Fixed((Fixed64)v.x, (Fixed64)v.y);
                        Vector2Fixed snapped = SnapVertex(vertex, canonicalVertices);
                        poly.AddVertex(snapped);
                    }
                    obstacles.Add(poly);
                }
            }

            // 2. 워커블 폴리곤 정점 수집 (메시 품질 개선용)
            List<Vector2Fixed> walkablePoints = new List<Vector2Fixed>();
            if (_walkables != null)
            {
                foreach (var walk in _walkables)
                {
                    if (walk != null && walk.vertices != null && walk.vertices.Length >= 3)
                    {
                        foreach (var v in walk.vertices)
                        {
                            Vector2Fixed vertex = new Vector2Fixed((Fixed64)v.x, (Fixed64)v.y);
                            Vector2Fixed snapped = SnapVertex(vertex, canonicalVertices);
                            walkablePoints.Add(snapped);
                        }
                    }
                }
            }

            // Asset이 있으면 거기에 저장, 없으면 런타임 생성
            if (_navMeshAsset != null)
            {
                _navMesh = _navMeshAsset;
                _builder.BuildFromRect(_navMesh, _boundaryMin, _boundaryMax, obstacles.Count > 0 ? obstacles : null, _maxTriangleCount, walkablePoints.Count > 0 ? walkablePoints : null);

#if UNITY_EDITOR
                UnityEditor.EditorUtility.SetDirty(_navMeshAsset);
#endif
            }
            else
            {
                _navMesh = _builder.BuildFromRect(_boundaryMin, _boundaryMax, obstacles.Count > 0 ? obstacles : null);
            }

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
        /// 정점을 기존 정점 목록에서 가까운 것과 스냅
        /// </summary>
        private Vector2Fixed SnapVertex(Vector2Fixed vertex, List<Vector2Fixed> canonicalVertices)
        {
            Fixed64 thresholdSqr = (Fixed64)(VERTEX_SNAP_THRESHOLD * VERTEX_SNAP_THRESHOLD);

            // 기존 정점 중 가까운 것 찾기
            foreach (var canonical in canonicalVertices)
            {
                Fixed64 distSqr = Vector2Fixed.SqrDistance(vertex, canonical);
                if (distSqr < thresholdSqr)
                {
                    return canonical; // 기존 정점 사용
                }
            }

            // 가까운 것 없으면 새 정점으로 등록
            canonicalVertices.Add(vertex);
            return vertex;
        }

        /// <summary>
        /// 현재 NavMesh 통계 출력
        /// </summary>
        [ContextMenu("Print NavMesh Stats")]
        public void PrintNavMeshStats()
        {
            if (_navMesh == null)
            {
                Debug.LogWarning("[NavMesh2DDemo] NavMesh not built yet!");
                return;
            }

            Debug.Log($"[NavMesh2DDemo] === NavMesh Stats ===\n" +
                      $"  Triangles: {_navMesh.TriangleCount}\n" +
                      $"  Vertices: {_navMesh.VertexCount}");
        }

        /// <summary>
        /// List vs Priority Queue 성능 비교 (10만회)
        /// </summary>
        [ContextMenu("Stress Test: List vs PriorityQueue (100k)")]
        public void StressTestListVsPriorityQueue()
        {
            if (_navMesh == null)
            {
                Debug.LogWarning("[NavMesh2DDemo] NavMesh not built yet!");
                return;
            }

            const int ITERATIONS = 100000;
            var random = new System.Random(42); // 고정 시드로 동일한 테스트

            // boundary를 Fixed64로 변환
            Fixed64 minX = (Fixed64)_boundaryMin.x;
            Fixed64 minY = (Fixed64)_boundaryMin.y;
            Fixed64 rangeX = (Fixed64)_boundaryMax.x - minX;
            Fixed64 rangeY = (Fixed64)_boundaryMax.y - minY;

            // 랜덤 시작/끝점 미리 생성 (Fixed64로 직접)
            var testCases = new List<(Vector2Fixed start, Vector2Fixed end)>();
            for (int i = 0; i < ITERATIONS; i++)
            {
                Fixed64 startX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 startY = minY + (Fixed64)random.NextDouble() * rangeY;
                Fixed64 endX = minX + (Fixed64)random.NextDouble() * rangeX;
                Fixed64 endY = minY + (Fixed64)random.NextDouble() * rangeY;

                testCases.Add((
                    new Vector2Fixed(startX, startY),
                    new Vector2Fixed(endX, endY)
                ));
            }

            Debug.Log($"[StressTest] Starting {ITERATIONS} iterations...");

            // 1. List 버전 테스트
            var listAStar = new TriangleAStarList(_navMesh);
            var sw = Stopwatch.StartNew();
            int listSuccessCount = 0;
            for (int i = 0; i < ITERATIONS; i++)
            {
                var result = listAStar.FindPath(testCases[i].start, testCases[i].end);
                if (result.Success) listSuccessCount++;
            }
            sw.Stop();
            double listMs = sw.Elapsed.TotalMilliseconds;

            // 2. Priority Queue 버전 테스트
            var pqAStar = new TriangleAStarPQ(_navMesh);
            sw.Restart();
            int pqSuccessCount = 0;
            for (int i = 0; i < ITERATIONS; i++)
            {
                var result = pqAStar.FindPath(testCases[i].start, testCases[i].end);
                if (result.Success) pqSuccessCount++;
            }
            sw.Stop();
            double pqMs = sw.Elapsed.TotalMilliseconds;

            // 결과 출력
            Debug.Log($"[StressTest] === Results ({ITERATIONS} iterations) ===\n" +
                      $"  List Version:          {listMs:F3} ms (success: {listSuccessCount})\n" +
                      $"  PriorityQueue Version: {pqMs:F3} ms (success: {pqSuccessCount})\n" +
                      $"  Difference:            {listMs - pqMs:F3} ms\n" +
                      $"  PQ is {(listMs / pqMs):F2}x faster");
        }

        /// <summary>
        /// A* with List (기존 방식)
        /// </summary>
        private class TriangleAStarList
        {
            private NavMesh2DData _navMesh;

            private class AStarNode
            {
                public int TriangleIndex;
                public Fixed64 G;
                public Fixed64 H;
                public Fixed64 F => G + H;
                public AStarNode Parent;
            }

            public struct PathResult { public bool Success; }

            public TriangleAStarList(NavMesh2DData navMesh) { _navMesh = navMesh; }

            public PathResult FindPath(Vector2Fixed start, Vector2Fixed end)
            {
                var result = new PathResult { Success = false };

                int startTri = _navMesh.FindTriangleContainingPoint(start);
                int endTri = _navMesh.FindTriangleContainingPoint(end);

                if (startTri < 0) startTri = _navMesh.FindNearestTriangle(start);
                if (endTri < 0) endTri = _navMesh.FindNearestTriangle(end);
                if (startTri < 0 || endTri < 0) return result;

                if (startTri == endTri) { result.Success = true; return result; }

                var openSet = new List<AStarNode>();
                var closedSet = new HashSet<int>();
                var nodeMap = new Dictionary<int, AStarNode>();

                var startNode = new AStarNode { TriangleIndex = startTri, G = Fixed64.Zero };
                startNode.H = Heuristic(startTri, end);
                openSet.Add(startNode);
                nodeMap[startTri] = startNode;

                while (openSet.Count > 0)
                {
                    // List: O(n) 탐색
                    AStarNode current = openSet[0];
                    Fixed64 lowestF = current.F;
                    for (int i = 1; i < openSet.Count; i++)
                    {
                        if (openSet[i].F < lowestF)
                        {
                            current = openSet[i];
                            lowestF = current.F;
                        }
                    }
                    openSet.Remove(current);

                    if (current.TriangleIndex == endTri) { result.Success = true; return result; }

                    closedSet.Add(current.TriangleIndex);

                    var triangle = _navMesh.GetTriangle(current.TriangleIndex);
                    for (int edge = 0; edge < 3; edge++)
                    {
                        int neighbor = triangle.GetNeighbor(edge);
                        if (neighbor < 0 || closedSet.Contains(neighbor)) continue;

                        Vector2Fixed edgeCenter = _navMesh.GetEdgeCenter(current.TriangleIndex, edge);
                        Fixed64 moveCost = GetMoveCost(current.TriangleIndex, edgeCenter);
                        Fixed64 newG = current.G + moveCost;

                        if (!nodeMap.TryGetValue(neighbor, out AStarNode neighborNode))
                        {
                            neighborNode = new AStarNode { TriangleIndex = neighbor };
                            neighborNode.H = Heuristic(neighbor, end);
                            nodeMap[neighbor] = neighborNode;
                            openSet.Add(neighborNode);
                        }
                        else if (!openSet.Contains(neighborNode)) continue;

                        if (newG < neighborNode.G || neighborNode.Parent == null)
                        {
                            neighborNode.G = newG;
                            neighborNode.Parent = current;
                        }
                    }
                }

                return result;
            }

            private Fixed64 Heuristic(int tri, Vector2Fixed target)
            {
                return Vector2Fixed.Distance(_navMesh.GetTriangleGeometry(tri).Centroid, target);
            }

            private Fixed64 GetMoveCost(int tri, Vector2Fixed to)
            {
                return Vector2Fixed.Distance(_navMesh.GetTriangleGeometry(tri).Centroid, to);
            }
        }

        /// <summary>
        /// A* with Priority Queue (Min Heap)
        /// </summary>
        private class TriangleAStarPQ
        {
            private NavMesh2DData _navMesh;

            private class AStarNode
            {
                public int TriangleIndex;
                public Fixed64 G;
                public Fixed64 H;
                public Fixed64 F => G + H;
                public AStarNode Parent;
            }

            public struct PathResult { public bool Success; }

            public TriangleAStarPQ(NavMesh2DData navMesh) { _navMesh = navMesh; }

            public PathResult FindPath(Vector2Fixed start, Vector2Fixed end)
            {
                var result = new PathResult { Success = false };

                int startTri = _navMesh.FindTriangleContainingPoint(start);
                int endTri = _navMesh.FindTriangleContainingPoint(end);

                if (startTri < 0) startTri = _navMesh.FindNearestTriangle(start);
                if (endTri < 0) endTri = _navMesh.FindNearestTriangle(end);
                if (startTri < 0 || endTri < 0) return result;

                if (startTri == endTri) { result.Success = true; return result; }

                var openSet = new MinHeap();
                var closedSet = new HashSet<int>();
                var nodeMap = new Dictionary<int, AStarNode>();

                var startNode = new AStarNode { TriangleIndex = startTri, G = Fixed64.Zero };
                startNode.H = Heuristic(startTri, end);
                openSet.Insert(startNode);
                nodeMap[startTri] = startNode;

                while (openSet.Count > 0)
                {
                    // PriorityQueue: O(log n) 추출
                    AStarNode current = openSet.ExtractMin();

                    if (current.TriangleIndex == endTri) { result.Success = true; return result; }

                    closedSet.Add(current.TriangleIndex);

                    var triangle = _navMesh.GetTriangle(current.TriangleIndex);
                    for (int edge = 0; edge < 3; edge++)
                    {
                        int neighbor = triangle.GetNeighbor(edge);
                        if (neighbor < 0 || closedSet.Contains(neighbor)) continue;

                        Vector2Fixed edgeCenter = _navMesh.GetEdgeCenter(current.TriangleIndex, edge);
                        Fixed64 moveCost = GetMoveCost(current.TriangleIndex, edgeCenter);
                        Fixed64 newG = current.G + moveCost;

                        if (!nodeMap.TryGetValue(neighbor, out AStarNode neighborNode))
                        {
                            neighborNode = new AStarNode { TriangleIndex = neighbor };
                            neighborNode.H = Heuristic(neighbor, end);
                            neighborNode.G = newG;
                            neighborNode.Parent = current;
                            nodeMap[neighbor] = neighborNode;
                            openSet.Insert(neighborNode);
                        }
                        else if (newG < neighborNode.G)
                        {
                            neighborNode.G = newG;
                            neighborNode.Parent = current;
                            openSet.DecreaseKey(neighborNode);
                        }
                    }
                }

                return result;
            }

            private Fixed64 Heuristic(int tri, Vector2Fixed target)
            {
                return Vector2Fixed.Distance(_navMesh.GetTriangleGeometry(tri).Centroid, target);
            }

            private Fixed64 GetMoveCost(int tri, Vector2Fixed to)
            {
                return Vector2Fixed.Distance(_navMesh.GetTriangleGeometry(tri).Centroid, to);
            }

            /// <summary>
            /// 간단한 Min Heap 구현
            /// </summary>
            private class MinHeap
            {
                private List<AStarNode> _heap = new List<AStarNode>();
                private Dictionary<AStarNode, int> _indices = new Dictionary<AStarNode, int>();

                public int Count => _heap.Count;

                public void Insert(AStarNode node)
                {
                    _heap.Add(node);
                    _indices[node] = _heap.Count - 1;
                    BubbleUp(_heap.Count - 1);
                }

                public AStarNode ExtractMin()
                {
                    if (_heap.Count == 0) return null;

                    var min = _heap[0];
                    _indices.Remove(min);

                    var last = _heap[_heap.Count - 1];
                    _heap.RemoveAt(_heap.Count - 1);

                    if (_heap.Count > 0)
                    {
                        _heap[0] = last;
                        _indices[last] = 0;
                        BubbleDown(0);
                    }

                    return min;
                }

                public void DecreaseKey(AStarNode node)
                {
                    if (_indices.TryGetValue(node, out int idx))
                    {
                        BubbleUp(idx);
                    }
                }

                private void BubbleUp(int idx)
                {
                    while (idx > 0)
                    {
                        int parent = (idx - 1) / 2;
                        if (_heap[idx].F >= _heap[parent].F) break;

                        Swap(idx, parent);
                        idx = parent;
                    }
                }

                private void BubbleDown(int idx)
                {
                    int count = _heap.Count;
                    while (true)
                    {
                        int left = 2 * idx + 1;
                        int right = 2 * idx + 2;
                        int smallest = idx;

                        if (left < count && _heap[left].F < _heap[smallest].F)
                            smallest = left;
                        if (right < count && _heap[right].F < _heap[smallest].F)
                            smallest = right;

                        if (smallest == idx) break;

                        Swap(idx, smallest);
                        idx = smallest;
                    }
                }

                private void Swap(int i, int j)
                {
                    var temp = _heap[i];
                    _heap[i] = _heap[j];
                    _heap[j] = temp;
                    _indices[_heap[i]] = i;
                    _indices[_heap[j]] = j;
                }
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
                string triPathStr = string.Join(" -> ", result.TrianglePath);
                Debug.Log($"[NavMesh2DDemo] Path found: {result.Path.Count} waypoints, length: {(float)result.PathLength:F2}\n  Start: {_startPoint} -> End: {_endPoint}\n  Triangle path: [{triPathStr}]");

                if (_visualizer != null)
                {
                    _visualizer.SetPath(result.Path, result.Portals, result.TrianglePath);
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
        public void AddRectangleObstacle() => AddRectangleObstacle(Vector2.zero);

        public void AddRectangleObstacle(Vector2 center)
        {
            if (_obstacles == null) _obstacles = new List<PolygonObstacle>();

            var obstacle = new PolygonObstacle
            {
                name = $"Rectangle {_obstacles.Count}",
                vertices = new Vector2[]
                {
                    center + new Vector2(-2, -1),
                    center + new Vector2(2, -1),
                    center + new Vector2(2, 1),
                    center + new Vector2(-2, 1)
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
        public void AddTriangleObstacle() => AddTriangleObstacle(Vector2.zero);

        public void AddTriangleObstacle(Vector2 center)
        {
            if (_obstacles == null) _obstacles = new List<PolygonObstacle>();

            var obstacle = new PolygonObstacle
            {
                name = $"Triangle {_obstacles.Count}",
                vertices = new Vector2[]
                {
                    center + new Vector2(0, 2),
                    center + new Vector2(-2, -1),
                    center + new Vector2(2, -1)
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
        public void AddPolygonObstacle() => AddPolygonObstacle(Vector2.zero);

        public void AddPolygonObstacle(Vector2 center)
        {
            if (_obstacles == null) _obstacles = new List<PolygonObstacle>();

            // 오각형으로 시작 (자유롭게 편집 가능)
            float radius = 1.5f;
            int segments = 5;
            Vector2[] verts = new Vector2[segments];

            for (int i = 0; i < segments; i++)
            {
                float angle = (i / (float)segments) * Mathf.PI * 2f - Mathf.PI / 2f; // 위쪽부터 시작
                verts[i] = center + new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
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
        public void AddCircleObstacle() => AddCircleObstacle(Vector2.zero);

        public void AddCircleObstacle(Vector2 center)
        {
            if (_obstacles == null) _obstacles = new List<PolygonObstacle>();

            float radius = 2f;
            int segments = 8;
            Vector2[] verts = new Vector2[segments];

            for (int i = 0; i < segments; i++)
            {
                float angle = (i / (float)segments) * Mathf.PI * 2f;
                verts[i] = center + new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
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

        #region Walkable Polygons

        /// <summary>
        /// 사각형 워커블 폴리곤 추가
        /// </summary>
        [ContextMenu("Walkables/Add Rectangle")]
        public void AddRectangleWalkable() => AddRectangleWalkable(Vector2.zero);

        public void AddRectangleWalkable(Vector2 center)
        {
            if (_walkables == null) _walkables = new List<PolygonObstacle>();

            var walkable = new PolygonObstacle
            {
                name = $"WalkRect {_walkables.Count}",
                vertices = new Vector2[]
                {
                    center + new Vector2(-2, -1),
                    center + new Vector2(2, -1),
                    center + new Vector2(2, 1),
                    center + new Vector2(-2, 1)
                }
            };
            _walkables.Add(walkable);

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 삼각형 워커블 폴리곤 추가
        /// </summary>
        [ContextMenu("Walkables/Add Triangle")]
        public void AddTriangleWalkable() => AddTriangleWalkable(Vector2.zero);

        public void AddTriangleWalkable(Vector2 center)
        {
            if (_walkables == null) _walkables = new List<PolygonObstacle>();

            var walkable = new PolygonObstacle
            {
                name = $"WalkTri {_walkables.Count}",
                vertices = new Vector2[]
                {
                    center + new Vector2(0, 2),
                    center + new Vector2(-2, -1),
                    center + new Vector2(2, -1)
                }
            };
            _walkables.Add(walkable);

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 자유 폴리곤 워커블 추가 (오각형 시작)
        /// </summary>
        [ContextMenu("Walkables/Add Polygon")]
        public void AddPolygonWalkable() => AddPolygonWalkable(Vector2.zero);

        public void AddPolygonWalkable(Vector2 center)
        {
            if (_walkables == null) _walkables = new List<PolygonObstacle>();

            float radius = 1.5f;
            int segments = 5;
            Vector2[] verts = new Vector2[segments];

            for (int i = 0; i < segments; i++)
            {
                float angle = (i / (float)segments) * Mathf.PI * 2f - Mathf.PI / 2f;
                verts[i] = center + new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
            }

            var walkable = new PolygonObstacle
            {
                name = $"WalkPoly {_walkables.Count}",
                vertices = verts
            };
            _walkables.Add(walkable);

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 원형 워커블 폴리곤 추가 (8각형 근사)
        /// </summary>
        [ContextMenu("Walkables/Add Circle (8-gon)")]
        public void AddCircleWalkable() => AddCircleWalkable(Vector2.zero);

        public void AddCircleWalkable(Vector2 center)
        {
            if (_walkables == null) _walkables = new List<PolygonObstacle>();

            float radius = 2f;
            int segments = 8;
            Vector2[] verts = new Vector2[segments];

            for (int i = 0; i < segments; i++)
            {
                float angle = (i / (float)segments) * Mathf.PI * 2f;
                verts[i] = center + new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
            }

            var walkable = new PolygonObstacle
            {
                name = $"WalkCircle {_walkables.Count}",
                vertices = verts
            };
            _walkables.Add(walkable);

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        /// <summary>
        /// 모든 워커블 폴리곤 제거
        /// </summary>
        [ContextMenu("Walkables/Clear All")]
        public void ClearAllWalkables()
        {
            if (_walkables != null)
            {
                _walkables.Clear();
            }

            if (_autoRebuildOnChange && Application.isPlaying)
            {
                RebuildNavMesh();
            }
        }

        #endregion

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

            // 장애물 표시 (빨간색)
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

            // 워커블 폴리곤 표시 (노란색)
            Gizmos.color = new Color(1, 1, 0, 0.5f);
            if (_walkables != null)
            {
                foreach (var walk in _walkables)
                {
                    if (walk != null && walk.vertices != null && walk.vertices.Length >= 3)
                    {
                        for (int i = 0; i < walk.vertices.Length; i++)
                        {
                            int next = (i + 1) % walk.vertices.Length;
                            Vector3 a = new Vector3(walk.vertices[i].x, walk.vertices[i].y, 0);
                            Vector3 b = new Vector3(walk.vertices[next].x, walk.vertices[next].y, 0);
                            Gizmos.DrawLine(a, b);
                        }
                    }
                }
            }

            // 시작점/끝점 표시
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(new Vector3(_startPoint.x, _startPoint.y, 0), 0.07f);

            Gizmos.color = Color.red;
            Gizmos.DrawSphere(new Vector3(_endPoint.x, _endPoint.y, 0), 0.07f);
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
