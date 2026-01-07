using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(WallOccluder))]
[CanEditMultipleObjects]
public class WallOccluderEditor : Editor
{
    private SerializedProperty _normalAngle;
    private SerializedProperty _useCustomBasePos;
    private SerializedProperty _customBasePos;
    private SerializedProperty _depthBias;

    private void OnEnable()
    {
        _normalAngle = serializedObject.FindProperty("_normalAngle");
        _useCustomBasePos = serializedObject.FindProperty("_useCustomBasePos");
        _customBasePos = serializedObject.FindProperty("_customBasePos");
        _depthBias = serializedObject.FindProperty("_depthBias");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.LabelField("법선벡터 (→ 벽 뒤 = 가려지는 방향)", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(_normalAngle, new GUIContent("Normal Angle (°)"));

        EditorGUILayout.Space();
        EditorGUILayout.PropertyField(_useCustomBasePos);

        if (!_useCustomBasePos.hasMultipleDifferentValues && _useCustomBasePos.boolValue)
        {
            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(_customBasePos);

            using (new EditorGUI.DisabledGroupScope(targets.Length > 1))
            {
                if (GUILayout.Button("Set to Current Position"))
                {
                    var occluder = (WallOccluder)target;
                    _customBasePos.vector2Value = occluder.transform.position;
                }
            }
            EditorGUI.indentLevel--;
        }

        EditorGUILayout.Space();
        EditorGUILayout.PropertyField(_depthBias);

        serializedObject.ApplyModifiedProperties();
    }

    private void OnSceneGUI()
    {
        var occluder = (WallOccluder)target;

        var so = new SerializedObject(occluder);
        var useCustom = so.FindProperty("_useCustomBasePos");
        var customPos = so.FindProperty("_customBasePos");
        var angle = so.FindProperty("_normalAngle");

        // 법선벡터 방향 계산
        float rad = angle.floatValue * Mathf.Deg2Rad;
        Vector2 normalDir = new Vector2(Mathf.Cos(rad), Mathf.Sin(rad));
        Vector2 tangent = new Vector2(-normalDir.y, normalDir.x);

        Vector3 basePos = useCustom.boolValue
            ? (Vector3)(Vector2)customPos.vector2Value
            : occluder.transform.position;

        // 법선벡터 표시 (벽 뒤 방향)
        Handles.color = Color.cyan;
        Handles.DrawLine(basePos, basePos + (Vector3)(normalDir * 2f));
        Handles.Label(basePos + (Vector3)(normalDir * 2.2f), "뒤 (가려짐)");

        // 벽 표면 라인 표시 (법선에 수직)
        Handles.color = Color.green;
        Handles.DrawLine(basePos - (Vector3)(tangent * 1.5f), basePos + (Vector3)(tangent * 1.5f));

        if (!useCustom.boolValue)
            return;

        EditorGUI.BeginChangeCheck();

        // Base Position 드래그 핸들
        Handles.color = Color.yellow;
        Vector3 newPos = Handles.FreeMoveHandle(
            basePos,
            0.3f,
            Vector3.zero,
            Handles.SphereHandleCap
        );

        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(occluder, "Move Wall Base Position");
            customPos.vector2Value = new Vector2(newPos.x, newPos.y);
            so.ApplyModifiedProperties();
        }

        // 라벨
        Handles.Label(basePos + Vector3.up * 0.5f, "Base Pos");
    }
}
