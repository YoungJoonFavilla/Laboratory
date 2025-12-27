using UnityEngine;
using UnityEditor;
using NavMesh2D.Demo;
using System.Collections.Generic;

namespace NavMesh2D.Editor
{
    /// <summary>
    /// NavMesh2DDemo 에디터 확장
    /// Scene View에서 장애물 폴리곤 편집 가능
    /// </summary>
    [CustomEditor(typeof(NavMesh2DDemo))]
    public class NavMesh2DDemoEditor : UnityEditor.Editor
    {
        private NavMesh2DDemo _demo;
        private SerializedProperty _boundaryMinProp;
        private SerializedProperty _boundaryMaxProp;
        private SerializedProperty _maxTriangleCountProp;
        private SerializedProperty _obstaclesProp;
        private SerializedProperty _walkablesProp;
        private SerializedProperty _visualizerProp;
        private SerializedProperty _navMeshAssetProp;
        private SerializedProperty _autoRebuildProp;
        private SerializedProperty _startPointProp;
        private SerializedProperty _endPointProp;

        // Demo Mode
        private SerializedProperty _useAgentModeProp;
        private SerializedProperty _agentProp;
        private SerializedProperty _agentMoveSpeedProp;

        private int _selectedObstacleIndex = -1;
        private int _selectedWalkableIndex = -1;
        private int _selectedVertexIndex = -1;
        private bool _isEditingWalkable = false;  // true: 워커블 편집 중, false: 장애물 편집 중

        private const float HANDLE_SIZE = 0.3f;
        private const float HIT_DISTANCE = 20f;

        // 캐싱용 (성능 최적화)
        private List<Vector3> _linePointsCache = new List<Vector3>();
        private Vector3[] _lineArrayCache;  // ToArray() 대신 재사용
        private bool _needsRepaint = false;

        // 정점 캐싱 (드래그 중 SerializedProperty 접근 최소화)
        private Vector2[] _cachedVertices;
        private int _cachedObstacleIndex = -1;
        private int _cachedVertexCount = -1;  // 배열 재생성 최소화
        private bool _isDragging = false;

        // 정점 인덱스 라벨 캐싱 (문자열 할당 방지)
        private static string[] _vertexLabelCache;
        private static int _vertexLabelCacheSize = 0;

        // GUIContent 캐싱
        private static GUIContent _maxTriangleCountContent;
        private static GUIContent _navMeshAssetContent;
        private static GUIContent _useAgentModeContent;
        private static GUIContent _agentContent;
        private static GUIContent _moveSpeedContent;

        // 이벤트 중복 등록 방지
        private bool _isEventRegistered = false;

        private void OnEnable()
        {
            _demo = (NavMesh2DDemo)target;
            _boundaryMinProp = serializedObject.FindProperty("_boundaryMin");
            _boundaryMaxProp = serializedObject.FindProperty("_boundaryMax");
            _maxTriangleCountProp = serializedObject.FindProperty("_maxTriangleCount");
            _obstaclesProp = serializedObject.FindProperty("_obstacles");
            _walkablesProp = serializedObject.FindProperty("_walkables");
            _visualizerProp = serializedObject.FindProperty("_visualizer");
            _navMeshAssetProp = serializedObject.FindProperty("_navMeshAsset");
            _autoRebuildProp = serializedObject.FindProperty("_autoRebuildOnChange");
            _startPointProp = serializedObject.FindProperty("_startPoint");
            _endPointProp = serializedObject.FindProperty("_endPoint");

            // Demo Mode
            _useAgentModeProp = serializedObject.FindProperty("_useAgentMode");
            _agentProp = serializedObject.FindProperty("_agent");
            _agentMoveSpeedProp = serializedObject.FindProperty("_agentMoveSpeed");

            // GUIContent 캐싱 (한 번만 생성)
            if (_maxTriangleCountContent == null)
            {
                _maxTriangleCountContent = new GUIContent("Max Triangle Count", "0이면 세분화 안함. 삼각형이 세분화될수록 경로 정확도가 올라가지만 성능은 낮아짐");
                _navMeshAssetContent = new GUIContent("NavMesh Asset", "할당하면 .asset 파일에 저장됨");
                _useAgentModeContent = new GUIContent("Use Agent Mode", "false: 시작/끝점 클릭 모드, true: 에이전트 이동 모드");
                _agentContent = new GUIContent("Agent", "이동할 에이전트 Transform");
                _moveSpeedContent = new GUIContent("Move Speed", "에이전트 이동 속도");
            }

            // 중복 등록 방지 - 먼저 제거 후 등록
            SceneView.duringSceneGui -= OnSceneGUI;
            SceneView.duringSceneGui += OnSceneGUI;
            _isEventRegistered = true;

            // 프로파일링 리셋
            _frameCount = 0;
            _totalDrawMs = 0;
            _totalUpdateMs = 0;
            _totalHandlesMs = 0;
        }

        private void OnDisable()
        {
            if (_isEventRegistered)
            {
                SceneView.duringSceneGui -= OnSceneGUI;
                _isEventRegistered = false;
            }
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.LabelField("NavMesh2D Demo", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            // Boundary
            EditorGUILayout.LabelField("Boundary", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_boundaryMinProp);
            EditorGUILayout.PropertyField(_boundaryMaxProp);

            // Triangle Count Info
            int currentTriCount = 0;
            if (_navMeshAssetProp.objectReferenceValue != null)
            {
                var navMeshAsset = _navMeshAssetProp.objectReferenceValue as NavMesh2D.Pathfinding.NavMesh2DData;
                if (navMeshAsset != null)
                    currentTriCount = navMeshAsset.TriangleCount;
            }

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PropertyField(_maxTriangleCountProp, _maxTriangleCountContent);
            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.IntField(currentTriCount, GUILayout.Width(50));
            EditorGUI.EndDisabledGroup();
            EditorGUILayout.EndHorizontal();

            // Warning if max is set but lower than current
            if (_maxTriangleCountProp.intValue > 0 && _maxTriangleCountProp.intValue <= currentTriCount && currentTriCount > 0)
            {
                EditorGUILayout.HelpBox($"Max({_maxTriangleCountProp.intValue}) ≤ 현재({currentTriCount}): 세분화 안함", MessageType.Info);
            }

            EditorGUILayout.Space();

            // Components
            EditorGUILayout.LabelField("Components", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_visualizerProp);
            EditorGUILayout.PropertyField(_navMeshAssetProp, _navMeshAssetContent);
            EditorGUILayout.PropertyField(_autoRebuildProp);

            EditorGUILayout.Space();

            // Demo Mode
            EditorGUILayout.LabelField("Demo Mode", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_useAgentModeProp, _useAgentModeContent);

            // Agent Mode 옵션들 (Agent Mode일 때만 표시)
            if (_useAgentModeProp.boolValue)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(_agentProp, _agentContent);
                EditorGUILayout.PropertyField(_agentMoveSpeedProp, _moveSpeedContent);

                if (_agentProp.objectReferenceValue == null)
                {
                    EditorGUILayout.HelpBox("Agent를 할당해주세요. 없으면 Point Mode로 전환됩니다.", MessageType.Warning);
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

            // Obstacles Header
            EditorGUILayout.LabelField("Obstacles", EditorStyles.boldLabel);

            // Obstacle buttons
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("+ Rectangle"))
            {
                _demo.AddRectangleObstacle(GetSceneViewCenter());
                SelectNewObstacle();
            }
            if (GUILayout.Button("+ Triangle"))
            {
                _demo.AddTriangleObstacle(GetSceneViewCenter());
                SelectNewObstacle();
            }
            if (GUILayout.Button("+ Circle"))
            {
                _demo.AddCircleObstacle(GetSceneViewCenter());
                SelectNewObstacle();
            }
            if (GUILayout.Button("+ Polygon"))
            {
                _demo.AddPolygonObstacle(GetSceneViewCenter());
                SelectNewObstacle();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Clear All"))
            {
                _demo.ClearAllObstacles();
                _selectedObstacleIndex = -1;
                InvalidateCache();
                serializedObject.Update();
            }
            if (GUILayout.Button("Rebuild NavMesh"))
            {
                _demo.RebuildNavMesh();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Subdivide Edges"))
            {
                SubdivideSelectedPolygonEdges();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space();

            // Pathfinding Test
            EditorGUILayout.LabelField("Pathfinding Test", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_startPointProp);
            EditorGUILayout.PropertyField(_endPointProp);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Calculate Path"))
            {
                _demo.CalculatePath();
            }
            if (GUILayout.Button("Swap"))
            {
                _demo.SwapStartEnd();
                serializedObject.Update();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space();

            // Obstacle List
            DrawObstacleList();

            EditorGUILayout.Space();

            // Walkable Polygons Header
            EditorGUILayout.LabelField("Walkable Polygons (노란색)", EditorStyles.boldLabel);

            // Walkable buttons
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("+ Rectangle"))
            {
                _demo.AddRectangleWalkable(GetSceneViewCenter());
                SelectNewWalkable();
            }
            if (GUILayout.Button("+ Triangle"))
            {
                _demo.AddTriangleWalkable(GetSceneViewCenter());
                SelectNewWalkable();
            }
            if (GUILayout.Button("+ Circle"))
            {
                _demo.AddCircleWalkable(GetSceneViewCenter());
                SelectNewWalkable();
            }
            if (GUILayout.Button("+ Polygon"))
            {
                _demo.AddPolygonWalkable(GetSceneViewCenter());
                SelectNewWalkable();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Clear All Walkables"))
            {
                _demo.ClearAllWalkables();
                _selectedWalkableIndex = -1;
                InvalidateCache();
                serializedObject.Update();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space();

            // Walkable List
            DrawWalkableList();

            EditorGUILayout.Space();

            // Help Box
            EditorGUILayout.HelpBox(
                "Scene View 사용법:\n" +
                "- 장애물 선택 후 정점 드래그: 위치 이동\n" +
                "- 정점 우클릭: 컨텍스트 메뉴 (추가/삭제)\n" +
                "- 장애물 전체 이동: 중심 핸들 드래그",
                MessageType.Info);

            serializedObject.ApplyModifiedProperties();
        }

        private void DrawObstacleList()
        {
            for (int i = 0; i < _obstaclesProp.arraySize; i++)
            {
                var obstacle = _obstaclesProp.GetArrayElementAtIndex(i);
                var nameProp = obstacle.FindPropertyRelative("name");
                var verticesProp = obstacle.FindPropertyRelative("vertices");

                bool isSelected = _selectedObstacleIndex == i;

                EditorGUILayout.BeginHorizontal();

                // Selection toggle
                EditorGUI.BeginChangeCheck();
                bool newSelected = GUILayout.Toggle(isSelected, "", GUILayout.Width(20));
                if (EditorGUI.EndChangeCheck())
                {
                    _selectedObstacleIndex = newSelected ? i : -1;
                    _selectedWalkableIndex = -1;
                    _isEditingWalkable = false;
                    _selectedVertexIndex = -1;
                    InvalidateCache();
                    SceneView.RepaintAll();
                }

                // Name
                EditorGUILayout.PropertyField(nameProp, GUIContent.none, GUILayout.Width(100));

                // Vertex count
                EditorGUILayout.LabelField($"{verticesProp.arraySize} verts", GUILayout.Width(60));

                // Select button
                if (GUILayout.Button("Edit", GUILayout.Width(40)))
                {
                    _selectedObstacleIndex = i;
                    _selectedWalkableIndex = -1;
                    _isEditingWalkable = false;
                    _selectedVertexIndex = -1;
                    InvalidateCache();
                    SceneView.RepaintAll();
                }

                // Delete button
                if (GUILayout.Button("X", GUILayout.Width(25)))
                {
                    _obstaclesProp.DeleteArrayElementAtIndex(i);
                    if (_selectedObstacleIndex >= _obstaclesProp.arraySize)
                    {
                        _selectedObstacleIndex = -1;
                    }
                    InvalidateCache();
                    break;
                }

                EditorGUILayout.EndHorizontal();

                // Show vertices if selected
                if (isSelected)
                {
                    EditorGUI.indentLevel++;
                    for (int v = 0; v < verticesProp.arraySize; v++)
                    {
                        EditorGUILayout.BeginHorizontal();
                        EditorGUILayout.LabelField($"  [{v}]", GUILayout.Width(40));
                        EditorGUILayout.PropertyField(verticesProp.GetArrayElementAtIndex(v), GUIContent.none);
                        EditorGUILayout.EndHorizontal();
                    }
                    EditorGUI.indentLevel--;
                }
            }
        }

        private void DrawWalkableList()
        {
            if (_walkablesProp == null)
                return;

            for (int i = 0; i < _walkablesProp.arraySize; i++)
            {
                var walkable = _walkablesProp.GetArrayElementAtIndex(i);
                var nameProp = walkable.FindPropertyRelative("name");
                var verticesProp = walkable.FindPropertyRelative("vertices");

                bool isSelected = _isEditingWalkable && _selectedWalkableIndex == i;

                EditorGUILayout.BeginHorizontal();

                // Selection toggle (노란색 배경으로 구분)
                var oldColor = GUI.backgroundColor;
                GUI.backgroundColor = isSelected ? Color.yellow : oldColor;

                EditorGUI.BeginChangeCheck();
                bool newSelected = GUILayout.Toggle(isSelected, "", GUILayout.Width(20));
                if (EditorGUI.EndChangeCheck())
                {
                    if (newSelected)
                    {
                        _selectedWalkableIndex = i;
                        _isEditingWalkable = true;
                        _selectedObstacleIndex = -1;  // 장애물 선택 해제
                    }
                    else
                    {
                        _selectedWalkableIndex = -1;
                        _isEditingWalkable = false;
                    }
                    _selectedVertexIndex = -1;
                    InvalidateCache();
                    SceneView.RepaintAll();
                }

                GUI.backgroundColor = oldColor;

                // Name
                EditorGUILayout.PropertyField(nameProp, GUIContent.none, GUILayout.Width(100));

                // Vertex count
                EditorGUILayout.LabelField($"{verticesProp.arraySize} verts", GUILayout.Width(60));

                // Edit button
                if (GUILayout.Button("Edit", GUILayout.Width(40)))
                {
                    _selectedWalkableIndex = i;
                    _isEditingWalkable = true;
                    _selectedObstacleIndex = -1;
                    _selectedVertexIndex = -1;
                    InvalidateCache();
                    SceneView.RepaintAll();
                }

                // Delete button
                if (GUILayout.Button("X", GUILayout.Width(25)))
                {
                    _walkablesProp.DeleteArrayElementAtIndex(i);
                    if (_selectedWalkableIndex >= _walkablesProp.arraySize)
                    {
                        _selectedWalkableIndex = -1;
                        _isEditingWalkable = false;
                    }
                    InvalidateCache();
                    break;
                }

                EditorGUILayout.EndHorizontal();

                // Show vertices if selected
                if (isSelected)
                {
                    EditorGUI.indentLevel++;
                    for (int v = 0; v < verticesProp.arraySize; v++)
                    {
                        EditorGUILayout.BeginHorizontal();
                        EditorGUILayout.LabelField($"  [{v}]", GUILayout.Width(40));
                        EditorGUILayout.PropertyField(verticesProp.GetArrayElementAtIndex(v), GUIContent.none);
                        EditorGUILayout.EndHorizontal();
                    }
                    EditorGUI.indentLevel--;
                }
            }
        }

        // 프로파일링
        private System.Diagnostics.Stopwatch _profileSw = new System.Diagnostics.Stopwatch();
        private int _frameCount = 0;
        private long _totalDrawMs = 0;
        private long _totalUpdateMs = 0;
        private long _totalHandlesMs = 0;
        private int _totalVertexCount = 0;
        private int _totalLineCount = 0;

        private void OnSceneGUI(SceneView sceneView)
        {
            if (_demo == null || _obstaclesProp == null)
                return;

            // Repaint 이벤트만 측정
            bool shouldProfile = Event.current.type == EventType.Repaint;
            Event e = Event.current;

            long t1 = 0, t2 = 0, t3 = 0;
            int vertexCount = 0, lineCount = 0;

            // Layout/Repaint 이벤트에서만 그리기 처리
            if (e.type == EventType.Layout || e.type == EventType.Repaint)
            {
                if (shouldProfile) _profileSw.Restart();
                DrawAllObstaclesOptimized();
                DrawAllWalkablesOptimized();  // 워커블 그리기 추가
                if (shouldProfile)
                {
                    _profileSw.Stop();
                    t1 = _profileSw.ElapsedMilliseconds;
                    lineCount = _linePointsCache.Count / 2;

                    // 장애물 수 진단 (처음 한번만)
                    if (_frameCount == 0)
                    {
                        Debug.Log($"[Diagnostic] Obstacle count: {_obstaclesProp.arraySize}, Walkable count: {(_walkablesProp != null ? _walkablesProp.arraySize : 0)}");
                    }
                }
            }

            // 마우스 이벤트는 별도 처리
            if (e.type == EventType.MouseDown || e.type == EventType.MouseUp || e.type == EventType.MouseDrag)
            {
                HandleMouseEvents(e);
            }

            // 핸들은 선택된 장애물 또는 워커블이 있을 때
            bool hasSelectedObstacle = !_isEditingWalkable && _selectedObstacleIndex >= 0 && _selectedObstacleIndex < _obstaclesProp.arraySize;
            bool hasSelectedWalkable = _isEditingWalkable && _selectedWalkableIndex >= 0 && _walkablesProp != null && _selectedWalkableIndex < _walkablesProp.arraySize;

            if (hasSelectedObstacle || hasSelectedWalkable)
            {
                if (shouldProfile) _profileSw.Restart();
                serializedObject.Update();
                if (shouldProfile)
                {
                    _profileSw.Stop();
                    t2 = _profileSw.ElapsedMilliseconds;
                }

                if (shouldProfile) _profileSw.Restart();

                if (hasSelectedObstacle)
                {
                    DrawSelectedObstacleHandles();
                    if (shouldProfile)
                    {
                        var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
                        vertexCount = obstacle.FindPropertyRelative("vertices").arraySize;
                    }
                }
                else if (hasSelectedWalkable)
                {
                    DrawSelectedWalkableHandles();
                    if (shouldProfile)
                    {
                        var walkable = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
                        vertexCount = walkable.FindPropertyRelative("vertices").arraySize;
                    }
                }

                if (shouldProfile)
                {
                    _profileSw.Stop();
                    t3 = _profileSw.ElapsedMilliseconds;
                }

                if (GUI.changed)
                {
                    serializedObject.ApplyModifiedProperties();
                }
            }

            // 통계 누적 및 출력 (Repaint만)
            if (shouldProfile)
            {
                _frameCount++;
                _totalDrawMs += t1;
                _totalUpdateMs += t2;
                _totalHandlesMs += t3;
                _totalVertexCount += vertexCount;
                _totalLineCount += lineCount;

                // 100프레임마다 평균 출력
                if (_frameCount % 100 == 0)
                {
                    float avgDraw = _totalDrawMs / 100f;
                    float avgUpdate = _totalUpdateMs / 100f;
                    float avgHandles = _totalHandlesMs / 100f;
                    float avgVerts = _totalVertexCount / 100f;
                    float avgLines = _totalLineCount / 100f;
                    float total = avgDraw + avgUpdate + avgHandles;

                    Debug.Log($"[Profile] Frame {_frameCount}: Total={total:F1}ms | Obstacles={_lastObstacleCount}, TotalVerts={_lastTotalVertices}");
                    Debug.Log($"  -> Iterate={_lastIterateMs}ms, DrawLines={_lastDrawLinesMs}ms");

                    // 리셋
                    _totalDrawMs = 0;
                    _totalUpdateMs = 0;
                    _totalHandlesMs = 0;
                    _totalVertexCount = 0;
                    _totalLineCount = 0;
                }
            }
        }

        private void HandleMouseEvents(Event e)
        {
            // 장애물 편집 중
            if (!_isEditingWalkable && _selectedObstacleIndex >= 0 && _selectedObstacleIndex < _obstaclesProp.arraySize)
            {
                // Right click on vertex - show context menu
                if (e.type == EventType.MouseDown && e.button == 1)
                {
                    int clickedVertex = GetClickedVertexIndex(e.mousePosition, false);
                    if (clickedVertex >= 0)
                    {
                        _selectedVertexIndex = clickedVertex;
                        ShowVertexContextMenu(clickedVertex, false);
                        e.Use();
                    }
                }
            }
            // 워커블 편집 중
            else if (_isEditingWalkable && _walkablesProp != null && _selectedWalkableIndex >= 0 && _selectedWalkableIndex < _walkablesProp.arraySize)
            {
                // Right click on vertex - show context menu
                if (e.type == EventType.MouseDown && e.button == 1)
                {
                    int clickedVertex = GetClickedVertexIndex(e.mousePosition, true);
                    if (clickedVertex >= 0)
                    {
                        _selectedVertexIndex = clickedVertex;
                        ShowVertexContextMenu(clickedVertex, true);
                        e.Use();
                    }
                }
            }
        }

        private int GetClickedVertexIndex(Vector2 mousePos, bool isWalkable)
        {
            SerializedProperty targetProp;
            int targetIndex;

            if (isWalkable)
            {
                if (_walkablesProp == null || _selectedWalkableIndex < 0 || _selectedWalkableIndex >= _walkablesProp.arraySize)
                    return -1;
                targetProp = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
                targetIndex = _selectedWalkableIndex;
            }
            else
            {
                if (_selectedObstacleIndex < 0 || _selectedObstacleIndex >= _obstaclesProp.arraySize)
                    return -1;
                targetProp = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
                targetIndex = _selectedObstacleIndex;
            }

            var verticesProp = targetProp.FindPropertyRelative("vertices");

            float closestDistance = float.MaxValue;
            int closestIndex = -1;

            for (int i = 0; i < verticesProp.arraySize; i++)
            {
                Vector2 vertex = verticesProp.GetArrayElementAtIndex(i).vector2Value;
                Vector3 worldPos = new Vector3(vertex.x, vertex.y, 0);
                Vector2 screenPos = HandleUtility.WorldToGUIPoint(worldPos);
                float distance = Vector2.Distance(mousePos, screenPos);

                if (distance < HIT_DISTANCE && distance < closestDistance)
                {
                    closestDistance = distance;
                    closestIndex = i;
                }
            }

            return closestIndex;
        }

        private void ShowVertexContextMenu(int vertexIndex, bool isWalkable)
        {
            SerializedProperty targetProp;
            if (isWalkable)
            {
                targetProp = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
            }
            else
            {
                targetProp = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
            }
            var verticesProp = targetProp.FindPropertyRelative("vertices");

            GenericMenu menu = new GenericMenu();

            string typeName = isWalkable ? "Walkable" : "Obstacle";
            menu.AddItem(new GUIContent($"{typeName} Vertex [{vertexIndex}]"), false, null);
            menu.AddSeparator("");

            menu.AddItem(new GUIContent("Add Point After"), false, () =>
            {
                AddVertexAfter(vertexIndex, isWalkable);
            });

            menu.AddItem(new GUIContent("Add Point Before"), false, () =>
            {
                AddVertexBefore(vertexIndex, isWalkable);
            });

            menu.AddSeparator("");

            if (verticesProp.arraySize > 3)
            {
                menu.AddItem(new GUIContent("Remove This Point"), false, () =>
                {
                    RemoveVertex(vertexIndex, isWalkable);
                });
            }
            else
            {
                menu.AddDisabledItem(new GUIContent("Remove This Point (min 3)"));
            }

            menu.AddSeparator("");

            menu.AddItem(new GUIContent("Reset to Origin"), false, () =>
            {
                ResetVertexPosition(vertexIndex, isWalkable);
            });

            menu.ShowAsContext();
        }

        private void AddVertexAfter(int index, bool isWalkable)
        {
            string undoName = isWalkable ? "Add Walkable Vertex" : "Add Obstacle Vertex";
            Undo.RecordObject(_demo, undoName);
            serializedObject.Update();

            SerializedProperty targetProp;
            if (isWalkable)
                targetProp = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
            else
                targetProp = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);

            var verticesProp = targetProp.FindPropertyRelative("vertices");

            int nextIndex = (index + 1) % verticesProp.arraySize;
            Vector2 current = verticesProp.GetArrayElementAtIndex(index).vector2Value;
            Vector2 next = verticesProp.GetArrayElementAtIndex(nextIndex).vector2Value;
            Vector2 midpoint = (current + next) / 2f;

            verticesProp.InsertArrayElementAtIndex(index + 1);
            verticesProp.GetArrayElementAtIndex(index + 1).vector2Value = midpoint;

            serializedObject.ApplyModifiedProperties();
            InvalidateCache();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();
        }

        private void AddVertexBefore(int index, bool isWalkable)
        {
            string undoName = isWalkable ? "Add Walkable Vertex" : "Add Obstacle Vertex";
            Undo.RecordObject(_demo, undoName);
            serializedObject.Update();

            SerializedProperty targetProp;
            if (isWalkable)
                targetProp = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
            else
                targetProp = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);

            var verticesProp = targetProp.FindPropertyRelative("vertices");

            int prevIndex = (index - 1 + verticesProp.arraySize) % verticesProp.arraySize;
            Vector2 current = verticesProp.GetArrayElementAtIndex(index).vector2Value;
            Vector2 prev = verticesProp.GetArrayElementAtIndex(prevIndex).vector2Value;
            Vector2 midpoint = (current + prev) / 2f;

            verticesProp.InsertArrayElementAtIndex(index);
            verticesProp.GetArrayElementAtIndex(index).vector2Value = midpoint;

            serializedObject.ApplyModifiedProperties();
            InvalidateCache();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();
        }

        private void RemoveVertex(int index, bool isWalkable)
        {
            string undoName = isWalkable ? "Remove Walkable Vertex" : "Remove Obstacle Vertex";
            Undo.RecordObject(_demo, undoName);
            serializedObject.Update();

            SerializedProperty targetProp;
            if (isWalkable)
                targetProp = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
            else
                targetProp = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);

            var verticesProp = targetProp.FindPropertyRelative("vertices");

            if (verticesProp.arraySize <= 3)
            {
                EditorUtility.DisplayDialog("Cannot Remove", "폴리곤은 최소 3개의 정점이 필요합니다.", "OK");
                return;
            }

            verticesProp.DeleteArrayElementAtIndex(index);
            _selectedVertexIndex = -1;

            serializedObject.ApplyModifiedProperties();
            InvalidateCache();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();
        }

        private void ResetVertexPosition(int index, bool isWalkable)
        {
            string undoName = isWalkable ? "Reset Walkable Vertex" : "Reset Obstacle Vertex";
            Undo.RecordObject(_demo, undoName);
            serializedObject.Update();

            SerializedProperty targetProp;
            if (isWalkable)
                targetProp = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
            else
                targetProp = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);

            var verticesProp = targetProp.FindPropertyRelative("vertices");

            verticesProp.GetArrayElementAtIndex(index).vector2Value = Vector2.zero;

            serializedObject.ApplyModifiedProperties();
            InvalidateCache();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();
        }

        // 내부 프로파일링용
        private long _lastIterateMs = 0;
        private long _lastDrawLinesMs = 0;
        private int _lastObstacleCount = 0;
        private int _lastTotalVertices = 0;

        // 모든 장애물 캐싱 (SerializedProperty 접근 최소화)
        private List<Vector2[]> _allObstaclesCache = new List<Vector2[]>();
        private int _allObstaclesCacheVersion = -1;
        private bool _allObstaclesCacheDirty = true;

        // 모든 워커블 캐싱
        private List<Vector2[]> _allWalkablesCache = new List<Vector2[]>();
        private bool _allWalkablesCacheDirty = true;

        /// <summary>
        /// 모든 장애물 캐시 갱신
        /// </summary>
        private void RefreshAllObstaclesCache()
        {
            int count = _obstaclesProp.arraySize;

            // 리스트 크기 조정
            while (_allObstaclesCache.Count < count)
                _allObstaclesCache.Add(null);
            while (_allObstaclesCache.Count > count)
                _allObstaclesCache.RemoveAt(_allObstaclesCache.Count - 1);

            // 각 장애물 정점 캐싱
            for (int i = 0; i < count; i++)
            {
                var obstacle = _obstaclesProp.GetArrayElementAtIndex(i);
                var verticesProp = obstacle.FindPropertyRelative("vertices");
                int vertCount = verticesProp.arraySize;

                // 배열 재사용 또는 생성
                if (_allObstaclesCache[i] == null || _allObstaclesCache[i].Length != vertCount)
                {
                    _allObstaclesCache[i] = new Vector2[vertCount];
                }

                for (int v = 0; v < vertCount; v++)
                {
                    _allObstaclesCache[i][v] = verticesProp.GetArrayElementAtIndex(v).vector2Value;
                }
            }

            _allObstaclesCacheDirty = false;
        }

        /// <summary>
        /// 모든 워커블 캐시 갱신
        /// </summary>
        private void RefreshAllWalkablesCache()
        {
            if (_walkablesProp == null)
            {
                _allWalkablesCache.Clear();
                _allWalkablesCacheDirty = false;
                return;
            }

            int count = _walkablesProp.arraySize;

            while (_allWalkablesCache.Count < count)
                _allWalkablesCache.Add(null);
            while (_allWalkablesCache.Count > count)
                _allWalkablesCache.RemoveAt(_allWalkablesCache.Count - 1);

            for (int i = 0; i < count; i++)
            {
                var walkable = _walkablesProp.GetArrayElementAtIndex(i);
                var verticesProp = walkable.FindPropertyRelative("vertices");
                int vertCount = verticesProp.arraySize;

                if (_allWalkablesCache[i] == null || _allWalkablesCache[i].Length != vertCount)
                {
                    _allWalkablesCache[i] = new Vector2[vertCount];
                }

                for (int v = 0; v < vertCount; v++)
                {
                    _allWalkablesCache[i][v] = verticesProp.GetArrayElementAtIndex(v).vector2Value;
                }
            }

            _allWalkablesCacheDirty = false;
        }

        /// <summary>
        /// 최적화된 장애물 그리기 (배치 드로잉)
        /// </summary>
        private void DrawAllObstaclesOptimized()
        {
            if (_obstaclesProp == null)
                return;

            var sw = System.Diagnostics.Stopwatch.StartNew();

            // 캐시 갱신 (더티 플래그가 설정된 경우만)
            if (_allObstaclesCacheDirty)
            {
                RefreshAllObstaclesCache();
            }

            // 캐시된 데이터로 라인 수집
            _linePointsCache.Clear();
            int obstacleCount = _allObstaclesCache.Count;
            int totalVertices = 0;

            for (int i = 0; i < obstacleCount; i++)
            {
                if (i == _selectedObstacleIndex)
                    continue;

                var vertices = _allObstaclesCache[i];
                if (vertices == null || vertices.Length < 3)
                    continue;

                int count = vertices.Length;
                totalVertices += count;

                for (int v = 0; v < count; v++)
                {
                    int next = (v + 1) % count;
                    Vector2 a = vertices[v];
                    Vector2 b = vertices[next];
                    _linePointsCache.Add(new Vector3(a.x, a.y, 0));
                    _linePointsCache.Add(new Vector3(b.x, b.y, 0));
                }
            }

            sw.Stop();
            _lastIterateMs = sw.ElapsedMilliseconds;
            _lastObstacleCount = obstacleCount;
            _lastTotalVertices = totalVertices;

            sw.Restart();

            // 비선택 장애물 일괄 그리기
            if (_linePointsCache.Count > 0)
            {
                Handles.color = new Color(1f, 0.3f, 0.3f, 0.8f);
                DrawLinesWithCache();
            }

            // 선택된 장애물 그리기
            if (_selectedObstacleIndex >= 0 && _selectedObstacleIndex < obstacleCount)
            {
                DrawSelectedObstacle();
            }

            sw.Stop();
            _lastDrawLinesMs = sw.ElapsedMilliseconds;
        }

        /// <summary>
        /// 최적화된 워커블 그리기 (노란색, 배치 드로잉)
        /// </summary>
        private void DrawAllWalkablesOptimized()
        {
            if (_walkablesProp == null)
                return;

            // 캐시 갱신
            if (_allWalkablesCacheDirty)
            {
                RefreshAllWalkablesCache();
            }

            // 캐시된 데이터로 라인 수집
            _linePointsCache.Clear();
            int walkableCount = _allWalkablesCache.Count;

            for (int i = 0; i < walkableCount; i++)
            {
                // 선택된 워커블은 별도로 그림
                if (_isEditingWalkable && i == _selectedWalkableIndex)
                    continue;

                var vertices = _allWalkablesCache[i];
                if (vertices == null || vertices.Length < 3)
                    continue;

                int count = vertices.Length;
                for (int v = 0; v < count; v++)
                {
                    int next = (v + 1) % count;
                    Vector2 a = vertices[v];
                    Vector2 b = vertices[next];
                    _linePointsCache.Add(new Vector3(a.x, a.y, 0));
                    _linePointsCache.Add(new Vector3(b.x, b.y, 0));
                }
            }

            // 비선택 워커블 일괄 그리기 (진한 노란색)
            if (_linePointsCache.Count > 0)
            {
                Handles.color = new Color(0.9f, 0.8f, 0.2f, 0.8f);
                DrawLinesWithCache();
            }

            // 선택된 워커블 그리기
            if (_isEditingWalkable && _selectedWalkableIndex >= 0 && _selectedWalkableIndex < walkableCount)
            {
                DrawSelectedWalkable();
            }
        }

        /// <summary>
        /// 선택된 워커블만 그리기 (정점 표시 포함)
        /// </summary>
        private void DrawSelectedWalkable()
        {
            if (_selectedWalkableIndex < 0 || _selectedWalkableIndex >= _allWalkablesCache.Count)
                return;

            var vertices = _allWalkablesCache[_selectedWalkableIndex];
            if (vertices == null || vertices.Length < 3)
                return;

            int count = vertices.Length;

            // 선택된 워커블 에지 그리기
            _linePointsCache.Clear();
            for (int i = 0; i < count; i++)
            {
                int next = (i + 1) % count;
                Vector2 a = vertices[i];
                Vector2 b = vertices[next];
                _linePointsCache.Add(new Vector3(a.x, a.y, 0));
                _linePointsCache.Add(new Vector3(b.x, b.y, 0));
            }

            Handles.color = Color.white;  // 선택된 워커블은 흰색
            DrawLinesWithCache();

            // 정점 그리기 (Repaint 이벤트에서만)
            if (Event.current.type == EventType.Repaint)
            {
                EnsureVertexLabelCache(count);

                for (int i = 0; i < count; i++)
                {
                    Vector3 worldPos = new Vector3(vertices[i].x, vertices[i].y, 0);

                    bool isVertexSelected = i == _selectedVertexIndex;
                    Handles.color = isVertexSelected ? Color.white : Color.yellow;

                    float size = HandleUtility.GetHandleSize(worldPos) * HANDLE_SIZE;
                    Handles.RectangleHandleCap(0, worldPos, Quaternion.identity, size, EventType.Repaint);

                    Handles.Label(worldPos + Vector3.up * size * 2f, _vertexLabelCache[i]);
                }
            }
        }

        /// <summary>
        /// 선택된 워커블 핸들 그리기
        /// </summary>
        private void DrawSelectedWalkableHandles()
        {
            if (_selectedWalkableIndex < 0 || _selectedWalkableIndex >= _allWalkablesCache.Count)
                return;

            var vertices = _allWalkablesCache[_selectedWalkableIndex];
            if (vertices == null || vertices.Length < 3)
                return;

            int vertCount = vertices.Length;

            // 중심점 계산
            Vector2 center = Vector2.zero;
            for (int i = 0; i < vertCount; i++)
            {
                center += vertices[i];
            }
            center /= vertCount;

            bool anyChange = false;

            // Draw center move handle
            EditorGUI.BeginChangeCheck();
            Vector3 newCenter = Handles.PositionHandle(new Vector3(center.x, center.y, 0), Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                if (!_isDragging)
                {
                    Undo.RecordObject(_demo, "Move Walkable");
                    _isDragging = true;
                }
                Vector2 delta = new Vector2(newCenter.x, newCenter.y) - center;

                for (int i = 0; i < vertCount; i++)
                {
                    vertices[i] += delta;
                }
                anyChange = true;
            }

            // Draw vertex move handles
            for (int i = 0; i < vertCount; i++)
            {
                Vector2 vertex = vertices[i];
                Vector3 worldPos = new Vector3(vertex.x, vertex.y, 0);

                float size = HandleUtility.GetHandleSize(worldPos) * HANDLE_SIZE;

                EditorGUI.BeginChangeCheck();
                Handles.color = (_selectedVertexIndex == i) ? Color.white : Color.yellow;
                Vector3 newPos = Handles.FreeMoveHandle(
                    worldPos,
                    size,
                    Vector3.zero,
                    Handles.RectangleHandleCap
                );

                if (EditorGUI.EndChangeCheck())
                {
                    if (!_isDragging)
                    {
                        Undo.RecordObject(_demo, "Move Walkable Vertex");
                        _isDragging = true;
                    }
                    vertices[i] = new Vector2(newPos.x, newPos.y);
                    _selectedVertexIndex = i;
                    anyChange = true;
                }
            }

            // 변경 사항을 SerializedProperty에 저장
            if (anyChange)
            {
                var walkable = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
                var verticesProp = walkable.FindPropertyRelative("vertices");

                for (int i = 0; i < vertCount; i++)
                {
                    verticesProp.GetArrayElementAtIndex(i).vector2Value = vertices[i];
                }
                EditorUtility.SetDirty(_demo);
            }

            // 마우스 업 감지 - 드래그 종료
            if (Event.current.type == EventType.MouseUp && _isDragging)
            {
                _isDragging = false;
            }
        }

        /// <summary>
        /// 선택된 장애물만 그리기 (정점 표시 포함)
        /// 전체 캐시 사용
        /// </summary>
        private void DrawSelectedObstacle()
        {
            // 전체 캐시에서 선택된 장애물 가져오기
            if (_selectedObstacleIndex < 0 || _selectedObstacleIndex >= _allObstaclesCache.Count)
                return;

            var vertices = _allObstaclesCache[_selectedObstacleIndex];
            if (vertices == null || vertices.Length < 3)
                return;

            int count = vertices.Length;

            // 선택된 장애물 에지 그리기
            _linePointsCache.Clear();
            for (int i = 0; i < count; i++)
            {
                int next = (i + 1) % count;
                Vector2 a = vertices[i];
                Vector2 b = vertices[next];
                _linePointsCache.Add(new Vector3(a.x, a.y, 0));
                _linePointsCache.Add(new Vector3(b.x, b.y, 0));
            }

            Handles.color = Color.yellow;
            DrawLinesWithCache();

            // 정점 그리기 (Repaint 이벤트에서만)
            if (Event.current.type == EventType.Repaint)
            {
                EnsureVertexLabelCache(count);

                for (int i = 0; i < count; i++)
                {
                    Vector3 worldPos = new Vector3(vertices[i].x, vertices[i].y, 0);

                    bool isVertexSelected = i == _selectedVertexIndex;
                    Handles.color = isVertexSelected ? Color.white : Color.yellow;

                    float size = HandleUtility.GetHandleSize(worldPos) * HANDLE_SIZE;
                    Handles.RectangleHandleCap(0, worldPos, Quaternion.identity, size, EventType.Repaint);

                    Handles.Label(worldPos + Vector3.up * size * 2f, _vertexLabelCache[i]);
                }
            }
        }

        /// <summary>
        /// 정점 라벨 캐시 확보
        /// </summary>
        private static void EnsureVertexLabelCache(int requiredSize)
        {
            if (_vertexLabelCache == null || _vertexLabelCacheSize < requiredSize)
            {
                int newSize = Mathf.Max(requiredSize, 32); // 최소 32개
                _vertexLabelCache = new string[newSize];
                for (int i = 0; i < newSize; i++)
                {
                    _vertexLabelCache[i] = $"[{i}]";
                }
                _vertexLabelCacheSize = newSize;
            }
        }

        // SerializedProperty 캐시 (드래그 종료 시 저장용)
        private SerializedProperty _cachedVerticesProp;
        private int _cachedVerticesPropObstacleIndex = -1;

        private void DrawSelectedObstacleHandles()
        {
            // 전체 캐시에서 선택된 장애물 가져오기
            if (_selectedObstacleIndex < 0 || _selectedObstacleIndex >= _allObstaclesCache.Count)
                return;

            var vertices = _allObstaclesCache[_selectedObstacleIndex];
            if (vertices == null || vertices.Length < 3)
                return;

            int vertCount = vertices.Length;

            // 중심점 계산
            Vector2 center = Vector2.zero;
            for (int i = 0; i < vertCount; i++)
            {
                center += vertices[i];
            }
            center /= vertCount;

            bool anyChange = false;

            // Draw center move handle
            EditorGUI.BeginChangeCheck();
            Vector3 newCenter = Handles.PositionHandle(new Vector3(center.x, center.y, 0), Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                if (!_isDragging)
                {
                    Undo.RecordObject(_demo, "Move Obstacle");
                    _isDragging = true;
                }
                Vector2 delta = new Vector2(newCenter.x, newCenter.y) - center;

                for (int i = 0; i < vertCount; i++)
                {
                    vertices[i] += delta;
                }
                anyChange = true;
            }

            // Draw vertex move handles (square handles)
            for (int i = 0; i < vertCount; i++)
            {
                Vector2 vertex = vertices[i];
                Vector3 worldPos = new Vector3(vertex.x, vertex.y, 0);

                float size = HandleUtility.GetHandleSize(worldPos) * HANDLE_SIZE;

                EditorGUI.BeginChangeCheck();
                Handles.color = (_selectedVertexIndex == i) ? Color.white : Color.yellow;
                Vector3 newPos = Handles.FreeMoveHandle(
                    worldPos,
                    size,
                    Vector3.zero,
                    Handles.RectangleHandleCap
                );

                if (EditorGUI.EndChangeCheck())
                {
                    if (!_isDragging)
                    {
                        Undo.RecordObject(_demo, "Move Vertex");
                        _isDragging = true;
                    }
                    vertices[i] = new Vector2(newPos.x, newPos.y);
                    _selectedVertexIndex = i;
                    anyChange = true;
                }
            }

            // 변경 사항을 SerializedProperty에 저장
            if (anyChange)
            {
                // SerializedProperty 캐시 갱신
                if (_cachedVerticesPropObstacleIndex != _selectedObstacleIndex || _cachedVerticesProp == null)
                {
                    var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
                    _cachedVerticesProp = obstacle.FindPropertyRelative("vertices");
                    _cachedVerticesPropObstacleIndex = _selectedObstacleIndex;
                }

                for (int i = 0; i < vertCount; i++)
                {
                    _cachedVerticesProp.GetArrayElementAtIndex(i).vector2Value = vertices[i];
                }
                EditorUtility.SetDirty(_demo);
            }

            // 마우스 업 감지 - 드래그 종료
            if (Event.current.type == EventType.MouseUp && _isDragging)
            {
                _isDragging = false;
            }
        }

        /// <summary>
        /// 캐시된 배열로 라인 그리기 (ToArray 호출 방지)
        /// </summary>
        private void DrawLinesWithCache()
        {
            int count = _linePointsCache.Count;
            if (count == 0) return;

            // 배열 크기가 정확히 맞을 때만 재사용, 아니면 재할당
            if (_lineArrayCache == null || _lineArrayCache.Length != count)
            {
                _lineArrayCache = new Vector3[count];
            }

            _linePointsCache.CopyTo(_lineArrayCache);
            Handles.DrawLines(_lineArrayCache);
        }

        /// <summary>
        /// 현재 SceneView 카메라 중심점 반환
        /// </summary>
        private Vector2 GetSceneViewCenter()
        {
            SceneView sceneView = SceneView.lastActiveSceneView;
            if (sceneView != null)
            {
                Vector3 pivot = sceneView.pivot;
                return new Vector2(pivot.x, pivot.y);
            }
            return Vector2.zero;
        }

        /// <summary>
        /// 새로 추가된 장애물 선택 및 편집 모드 진입
        /// </summary>
        private void SelectNewObstacle()
        {
            serializedObject.Update();
            _selectedObstacleIndex = _obstaclesProp.arraySize - 1;
            _selectedWalkableIndex = -1;
            _isEditingWalkable = false;
            _selectedVertexIndex = -1;
            InvalidateCache();
            SceneView.RepaintAll();
        }

        /// <summary>
        /// 새로 추가된 워커블 선택 및 편집 모드 진입
        /// </summary>
        private void SelectNewWalkable()
        {
            serializedObject.Update();
            _selectedWalkableIndex = _walkablesProp.arraySize - 1;
            _selectedObstacleIndex = -1;
            _isEditingWalkable = true;
            _selectedVertexIndex = -1;
            InvalidateCache();
            SceneView.RepaintAll();
        }

        /// <summary>
        /// 모든 캐시 무효화
        /// </summary>
        private void InvalidateCache()
        {
            _cachedObstacleIndex = -1;
            _cachedVertexCount = -1;
            _cachedVerticesPropObstacleIndex = -1;
            _cachedVerticesProp = null;
            _allObstaclesCacheDirty = true;  // 전체 장애물 캐시도 무효화
            _allWalkablesCacheDirty = true;  // 전체 워커블 캐시도 무효화
        }

        /// <summary>
        /// 선택된 폴리곤의 에지를 분할
        /// </summary>
        private void SubdivideSelectedPolygonEdges()
        {
            // 선택된 폴리곤 확인
            SerializedProperty targetProp = null;
            int targetIndex = -1;
            string typeName = "";

            if (_isEditingWalkable && _selectedWalkableIndex >= 0 && _walkablesProp != null && _selectedWalkableIndex < _walkablesProp.arraySize)
            {
                targetProp = _walkablesProp.GetArrayElementAtIndex(_selectedWalkableIndex);
                targetIndex = _selectedWalkableIndex;
                typeName = "워커블";
            }
            else if (!_isEditingWalkable && _selectedObstacleIndex >= 0 && _selectedObstacleIndex < _obstaclesProp.arraySize)
            {
                targetProp = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
                targetIndex = _selectedObstacleIndex;
                typeName = "장애물";
            }

            if (targetProp == null)
            {
                EditorUtility.DisplayDialog("Subdivide Edges", "편집 중인 폴리곤이 없습니다.\n장애물 또는 워커블을 먼저 선택해주세요.", "OK");
                return;
            }

            var verticesProp = targetProp.FindPropertyRelative("vertices");
            if (verticesProp == null || verticesProp.arraySize < 3)
            {
                EditorUtility.DisplayDialog("Subdivide Edges", "유효하지 않은 폴리곤입니다.", "OK");
                return;
            }

            // maxEdgeLength 입력 받기
            float maxEdgeLength = EditorPrefs.GetFloat("NavMesh2D_MaxEdgeLength", 2f);
            maxEdgeLength = EditorUtility.DisplayDialogComplex(
                "Subdivide Edges",
                $"최대 에지 길이를 입력하세요.\n현재 설정: {maxEdgeLength:F1}\n\n이 길이를 초과하는 에지가 분할됩니다.",
                "확인 (2.0)", "취소", "확인 (1.0)") switch
            {
                0 => 2.0f,
                2 => 1.0f,
                _ => -1f  // 취소
            };

            if (maxEdgeLength < 0)
                return;

            EditorPrefs.SetFloat("NavMesh2D_MaxEdgeLength", maxEdgeLength);

            // Undo 등록
            Undo.RecordObject(_demo, $"Subdivide {typeName} Edges");
            serializedObject.Update();

            // 에지 분할 수행
            int subdivideCount = SubdivideEdges(verticesProp, maxEdgeLength);

            serializedObject.ApplyModifiedProperties();
            InvalidateCache();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();

            if (subdivideCount > 0)
            {
                Debug.Log($"[Subdivide] {typeName} #{targetIndex}: {subdivideCount}개 에지 분할됨 (maxLength={maxEdgeLength:F1})");
            }
            else
            {
                Debug.Log($"[Subdivide] {typeName} #{targetIndex}: 분할할 에지 없음 (모든 에지가 {maxEdgeLength:F1} 이하)");
            }
        }

        /// <summary>
        /// 에지 분할 수행
        /// </summary>
        /// <param name="verticesProp">정점 배열 프로퍼티</param>
        /// <param name="maxEdgeLength">최대 에지 길이</param>
        /// <returns>분할된 에지 수</returns>
        private int SubdivideEdges(SerializedProperty verticesProp, float maxEdgeLength)
        {
            int subdivideCount = 0;
            float maxLengthSqr = maxEdgeLength * maxEdgeLength;

            // 역순으로 순회 (삽입해도 인덱스 꼬임 방지)
            // 하지만 폴리곤이라 순환하므로 여러 패스 필요
            bool anySubdivided = true;
            int maxIterations = 100;  // 무한루프 방지
            int iteration = 0;

            while (anySubdivided && iteration < maxIterations)
            {
                anySubdivided = false;
                iteration++;

                for (int i = verticesProp.arraySize - 1; i >= 0; i--)
                {
                    int next = (i + 1) % verticesProp.arraySize;
                    Vector2 a = verticesProp.GetArrayElementAtIndex(i).vector2Value;
                    Vector2 b = verticesProp.GetArrayElementAtIndex(next).vector2Value;

                    float lengthSqr = (b - a).sqrMagnitude;
                    if (lengthSqr > maxLengthSqr)
                    {
                        // 중점 삽입
                        Vector2 mid = (a + b) / 2f;

                        // i 다음 위치에 삽입 (next 위치)
                        int insertIndex = (i + 1);
                        if (insertIndex >= verticesProp.arraySize)
                        {
                            // 마지막-첫번째 에지의 경우 맨 끝에 삽입
                            verticesProp.InsertArrayElementAtIndex(verticesProp.arraySize);
                            verticesProp.GetArrayElementAtIndex(verticesProp.arraySize - 1).vector2Value = mid;
                        }
                        else
                        {
                            verticesProp.InsertArrayElementAtIndex(insertIndex);
                            verticesProp.GetArrayElementAtIndex(insertIndex).vector2Value = mid;
                        }

                        subdivideCount++;
                        anySubdivided = true;
                    }
                }
            }

            return subdivideCount;
        }
    }
}
