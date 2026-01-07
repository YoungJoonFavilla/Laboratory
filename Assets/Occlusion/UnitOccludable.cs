using UnityEngine;

[ExecuteAlways]
public class UnitOccludable : MonoBehaviour
{
    [Header("Depth Settings")]
    [SerializeField] private float _projectionAngle = 129.5f;

    [Tooltip("음수 = 유닛이 앞으로 옴 (덜 가려짐), 양수 = 유닛이 뒤로 밀림 (더 가려짐)")]
    [SerializeField] private float _depthOffset = 0f;

    private static readonly int UnitDepthId = Shader.PropertyToID("_UnitDepth");

    public Vector2 DepthDir
    {
        get
        {
            float rad = _projectionAngle * Mathf.Deg2Rad;
            return new Vector2(Mathf.Cos(rad), Mathf.Sin(rad)).normalized;
        }
    }

    /// <summary>
    /// 유닛의 균일 깊이값 (발 위치 기준 + 오프셋)
    /// </summary>
    public float UnitDepth => Vector2.Dot(transform.position, DepthDir) + _depthOffset;

    private void Update()
    {
        // 글로벌 셰이더 변수로 유닛 깊이 전달
        Shader.SetGlobalFloat(UnitDepthId, UnitDepth);
    }

#if UNITY_EDITOR
    private void OnDrawGizmosSelected()
    {
        Vector3 pos = transform.position;

        // 투영 방향 표시
        Gizmos.color = Color.green;
        Gizmos.DrawLine(pos, pos + (Vector3)(DepthDir * 1.5f));
        Gizmos.DrawWireSphere(pos, 0.15f);

        // 깊이값 표시
        UnityEditor.Handles.Label(pos + Vector3.up * 0.5f, $"Depth: {UnitDepth:F2}");
    }
#endif
}
