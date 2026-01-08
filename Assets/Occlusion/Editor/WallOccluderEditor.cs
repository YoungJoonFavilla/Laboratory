using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(WallOccluder))]
[CanEditMultipleObjects]
public class WallOccluderEditor : Editor
{
    private SerializedProperty _normalAngle;
    private SerializedProperty _useCustomBasePos;
    private SerializedProperty _localOffset;
    private SerializedProperty _depthBias;

    private void OnEnable()
    {
        _normalAngle = serializedObject.FindProperty("_normalAngle");
        _useCustomBasePos = serializedObject.FindProperty("_useCustomBasePos");
        _localOffset = serializedObject.FindProperty("_localOffset");
        _depthBias = serializedObject.FindProperty("_depthBias");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.LabelField("법선벡터 (→ 벽 뒤 = 가려지는 방향)", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(_normalAngle, new GUIContent("Normal Angle (°)"));

        EditorGUILayout.Space();
        EditorGUILayout.PropertyField(_useCustomBasePos, new GUIContent("Use Local Offset"));

        if (!_useCustomBasePos.hasMultipleDifferentValues && _useCustomBasePos.boolValue)
        {
            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(_localOffset, new GUIContent("Local Offset"));

            using (new EditorGUI.DisabledGroupScope(targets.Length > 1))
            {
                if (GUILayout.Button("Reset Offset"))
                {
                    _localOffset.vector2Value = Vector2.zero;
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
        var localOffset = so.FindProperty("_localOffset");
        var angle = so.FindProperty("_normalAngle");

        // 법선벡터 방향 계산
        float rad = angle.floatValue * Mathf.Deg2Rad;
        Vector2 normalDir = new Vector2(Mathf.Cos(rad), Mathf.Sin(rad));
        Vector2 tangent = new Vector2(-normalDir.y, normalDir.x);

        // Base Position 계산 (로컬 오프셋 → 월드 좌표)
        Vector3 basePos = useCustom.boolValue
            ? occluder.transform.TransformPoint(localOffset.vector2Value)
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
        Vector3 newWorldPos = Handles.FreeMoveHandle(
            basePos,
            0.3f,
            Vector3.zero,
            Handles.SphereHandleCap
        );

        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(occluder, "Move Wall Base Position");
            // 월드 좌표 → 로컬 오프셋으로 변환
            Vector3 newLocalPos = occluder.transform.InverseTransformPoint(newWorldPos);
            localOffset.vector2Value = new Vector2(newLocalPos.x, newLocalPos.y);
            so.ApplyModifiedProperties();
        }

        // 라벨
        Handles.Label(basePos + Vector3.up * 0.5f, "Base Pos");
    }
}
