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
        private SerializedProperty _obstaclesProp;
        private SerializedProperty _visualizerProp;
        private SerializedProperty _autoRebuildProp;

        private int _selectedObstacleIndex = -1;
        private int _selectedVertexIndex = -1;

        private const float HANDLE_SIZE = 0.3f;
        private const float HIT_DISTANCE = 20f;

        private void OnEnable()
        {
            _demo = (NavMesh2DDemo)target;
            _boundaryMinProp = serializedObject.FindProperty("_boundaryMin");
            _boundaryMaxProp = serializedObject.FindProperty("_boundaryMax");
            _obstaclesProp = serializedObject.FindProperty("_obstacles");
            _visualizerProp = serializedObject.FindProperty("_visualizer");
            _autoRebuildProp = serializedObject.FindProperty("_autoRebuildOnChange");

            SceneView.duringSceneGui += OnSceneGUI;
        }

        private void OnDisable()
        {
            SceneView.duringSceneGui -= OnSceneGUI;
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

            EditorGUILayout.Space();

            // Components
            EditorGUILayout.LabelField("Components", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_visualizerProp);
            EditorGUILayout.PropertyField(_autoRebuildProp);

            EditorGUILayout.Space();

            // Obstacles Header
            EditorGUILayout.LabelField("Obstacles", EditorStyles.boldLabel);

            // Obstacle buttons
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("+ Rectangle"))
            {
                _demo.AddRectangleObstacle();
                _selectedObstacleIndex = _obstaclesProp.arraySize - 1;
                serializedObject.Update();
            }
            if (GUILayout.Button("+ Triangle"))
            {
                _demo.AddTriangleObstacle();
                _selectedObstacleIndex = _obstaclesProp.arraySize - 1;
                serializedObject.Update();
            }
            if (GUILayout.Button("+ Circle"))
            {
                _demo.AddCircleObstacle();
                _selectedObstacleIndex = _obstaclesProp.arraySize - 1;
                serializedObject.Update();
            }
            if (GUILayout.Button("+ Polygon"))
            {
                _demo.AddPolygonObstacle();
                _selectedObstacleIndex = _obstaclesProp.arraySize - 1;
                serializedObject.Update();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Clear All"))
            {
                _demo.ClearAllObstacles();
                _selectedObstacleIndex = -1;
                serializedObject.Update();
            }
            if (GUILayout.Button("Rebuild NavMesh"))
            {
                _demo.RebuildNavMesh();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space();

            // Obstacle List
            DrawObstacleList();

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
                    _selectedVertexIndex = -1;
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
                    _selectedVertexIndex = -1;
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

        private void OnSceneGUI(SceneView sceneView)
        {
            if (_demo == null || _obstaclesProp == null)
                return;

            serializedObject.Update();

            Event e = Event.current;

            // Handle mouse events
            HandleMouseEvents(e);

            // Draw all obstacles
            DrawAllObstacles();

            // Draw selected obstacle with handles
            if (_selectedObstacleIndex >= 0 && _selectedObstacleIndex < _obstaclesProp.arraySize)
            {
                DrawSelectedObstacleHandles();
            }

            serializedObject.ApplyModifiedProperties();
        }

        private void HandleMouseEvents(Event e)
        {
            if (_selectedObstacleIndex < 0 || _selectedObstacleIndex >= _obstaclesProp.arraySize)
                return;

            // Right click on vertex - show context menu
            if (e.type == EventType.MouseDown && e.button == 1)
            {
                int clickedVertex = GetClickedVertexIndex(e.mousePosition);
                if (clickedVertex >= 0)
                {
                    _selectedVertexIndex = clickedVertex;
                    ShowVertexContextMenu(clickedVertex);
                    e.Use();
                }
            }
        }

        private int GetClickedVertexIndex(Vector2 mousePos)
        {
            if (_selectedObstacleIndex < 0 || _selectedObstacleIndex >= _obstaclesProp.arraySize)
                return -1;

            var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
            var verticesProp = obstacle.FindPropertyRelative("vertices");

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

        private void ShowVertexContextMenu(int vertexIndex)
        {
            var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
            var verticesProp = obstacle.FindPropertyRelative("vertices");

            GenericMenu menu = new GenericMenu();

            menu.AddItem(new GUIContent($"Vertex [{vertexIndex}]"), false, null);
            menu.AddSeparator("");

            menu.AddItem(new GUIContent("Add Point After"), false, () =>
            {
                AddVertexAfter(vertexIndex);
            });

            menu.AddItem(new GUIContent("Add Point Before"), false, () =>
            {
                AddVertexBefore(vertexIndex);
            });

            menu.AddSeparator("");

            if (verticesProp.arraySize > 3)
            {
                menu.AddItem(new GUIContent("Remove This Point"), false, () =>
                {
                    RemoveVertex(vertexIndex);
                });
            }
            else
            {
                menu.AddDisabledItem(new GUIContent("Remove This Point (min 3)"));
            }

            menu.AddSeparator("");

            menu.AddItem(new GUIContent("Reset to Origin"), false, () =>
            {
                ResetVertexPosition(vertexIndex);
            });

            menu.ShowAsContext();
        }

        private void AddVertexAfter(int index)
        {
            Undo.RecordObject(_demo, "Add Vertex");
            serializedObject.Update();

            var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
            var verticesProp = obstacle.FindPropertyRelative("vertices");

            int nextIndex = (index + 1) % verticesProp.arraySize;
            Vector2 current = verticesProp.GetArrayElementAtIndex(index).vector2Value;
            Vector2 next = verticesProp.GetArrayElementAtIndex(nextIndex).vector2Value;
            Vector2 midpoint = (current + next) / 2f;

            verticesProp.InsertArrayElementAtIndex(index + 1);
            verticesProp.GetArrayElementAtIndex(index + 1).vector2Value = midpoint;

            serializedObject.ApplyModifiedProperties();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();
        }

        private void AddVertexBefore(int index)
        {
            Undo.RecordObject(_demo, "Add Vertex");
            serializedObject.Update();

            var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
            var verticesProp = obstacle.FindPropertyRelative("vertices");

            int prevIndex = (index - 1 + verticesProp.arraySize) % verticesProp.arraySize;
            Vector2 current = verticesProp.GetArrayElementAtIndex(index).vector2Value;
            Vector2 prev = verticesProp.GetArrayElementAtIndex(prevIndex).vector2Value;
            Vector2 midpoint = (current + prev) / 2f;

            verticesProp.InsertArrayElementAtIndex(index);
            verticesProp.GetArrayElementAtIndex(index).vector2Value = midpoint;

            serializedObject.ApplyModifiedProperties();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();
        }

        private void RemoveVertex(int index)
        {
            Undo.RecordObject(_demo, "Remove Vertex");
            serializedObject.Update();

            var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
            var verticesProp = obstacle.FindPropertyRelative("vertices");

            if (verticesProp.arraySize <= 3)
            {
                EditorUtility.DisplayDialog("Cannot Remove", "폴리곤은 최소 3개의 정점이 필요합니다.", "OK");
                return;
            }

            verticesProp.DeleteArrayElementAtIndex(index);
            _selectedVertexIndex = -1;

            serializedObject.ApplyModifiedProperties();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();
        }

        private void ResetVertexPosition(int index)
        {
            Undo.RecordObject(_demo, "Reset Vertex");
            serializedObject.Update();

            var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
            var verticesProp = obstacle.FindPropertyRelative("vertices");

            verticesProp.GetArrayElementAtIndex(index).vector2Value = Vector2.zero;

            serializedObject.ApplyModifiedProperties();
            EditorUtility.SetDirty(_demo);
            SceneView.RepaintAll();
        }

        private void DrawAllObstacles()
        {
            for (int i = 0; i < _obstaclesProp.arraySize; i++)
            {
                bool isSelected = i == _selectedObstacleIndex;
                DrawObstacle(i, isSelected);
            }
        }

        private void DrawObstacle(int obstacleIndex, bool isSelected)
        {
            var obstacle = _obstaclesProp.GetArrayElementAtIndex(obstacleIndex);
            var verticesProp = obstacle.FindPropertyRelative("vertices");

            if (verticesProp.arraySize < 3)
                return;

            // Draw edges
            Handles.color = isSelected ? Color.yellow : new Color(1f, 0.3f, 0.3f, 0.8f);
            for (int i = 0; i < verticesProp.arraySize; i++)
            {
                int next = (i + 1) % verticesProp.arraySize;
                Vector2 a = verticesProp.GetArrayElementAtIndex(i).vector2Value;
                Vector2 b = verticesProp.GetArrayElementAtIndex(next).vector2Value;
                Handles.DrawLine(
                    new Vector3(a.x, a.y, 0),
                    new Vector3(b.x, b.y, 0),
                    isSelected ? 3f : 2f
                );
            }

            // Draw vertices as squares (only for selected obstacle)
            if (isSelected)
            {
                for (int i = 0; i < verticesProp.arraySize; i++)
                {
                    Vector2 vertex = verticesProp.GetArrayElementAtIndex(i).vector2Value;
                    Vector3 worldPos = new Vector3(vertex.x, vertex.y, 0);

                    bool isVertexSelected = i == _selectedVertexIndex;
                    Handles.color = isVertexSelected ? Color.white : Color.yellow;

                    float size = HandleUtility.GetHandleSize(worldPos) * HANDLE_SIZE;
                    Handles.RectangleHandleCap(0, worldPos, Quaternion.identity, size, EventType.Repaint);

                    // Vertex index label
                    Handles.Label(worldPos + Vector3.up * size * 2f, $"[{i}]");
                }
            }
        }

        private void DrawSelectedObstacleHandles()
        {
            var obstacle = _obstaclesProp.GetArrayElementAtIndex(_selectedObstacleIndex);
            var verticesProp = obstacle.FindPropertyRelative("vertices");

            if (verticesProp.arraySize < 3)
                return;

            // Calculate center
            Vector2 center = Vector2.zero;
            for (int i = 0; i < verticesProp.arraySize; i++)
            {
                center += verticesProp.GetArrayElementAtIndex(i).vector2Value;
            }
            center /= verticesProp.arraySize;

            // Draw center move handle
            EditorGUI.BeginChangeCheck();
            Vector3 newCenter = Handles.PositionHandle(new Vector3(center.x, center.y, 0), Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(_demo, "Move Obstacle");
                Vector2 delta = new Vector2(newCenter.x, newCenter.y) - center;

                for (int i = 0; i < verticesProp.arraySize; i++)
                {
                    var vertexProp = verticesProp.GetArrayElementAtIndex(i);
                    vertexProp.vector2Value += delta;
                }

                EditorUtility.SetDirty(_demo);
            }

            // Draw vertex move handles (square handles)
            for (int i = 0; i < verticesProp.arraySize; i++)
            {
                var vertexProp = verticesProp.GetArrayElementAtIndex(i);
                Vector2 vertex = vertexProp.vector2Value;
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
                    Undo.RecordObject(_demo, "Move Vertex");
                    vertexProp.vector2Value = new Vector2(newPos.x, newPos.y);
                    _selectedVertexIndex = i;
                    EditorUtility.SetDirty(_demo);
                }
            }
        }
    }
}
