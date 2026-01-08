using UnityEngine;

[RequireComponent(typeof(SpriteRenderer))]
public class WallOccluder : MonoBehaviour
{
    [Header("Normal Vector")]
    [Tooltip("법선벡터 각도 (이 방향 = 벽 뒤, 유닛이 가려지는 방향)")]
    [SerializeField] private float _normalAngle = OccConstant.DEFAULT_NORMAL_ANGLE;

    [Header("Base Position")]
    [Tooltip("체크시 로컬 오프셋 사용, 해제시 transform.position 그대로")]
    [SerializeField] private bool _useCustomBasePos = false;
    [SerializeField] private Vector2 _localOffset = Vector2.zero;

    [Header("Depth Settings")]
    [Tooltip("양수 = 벽이 뒤로 밀림 (덜 가림), 음수 = 벽이 앞으로 옴 (더 가림)")]
    [SerializeField] private float _depthBias = 0.5f;

    private static readonly int WallBasePosId = Shader.PropertyToID("_WallBasePos");
    private static readonly int WallDepthDirId = Shader.PropertyToID("_WallDepthDir");
    private static readonly int DepthBiasId = Shader.PropertyToID("_DepthBias");

    public Vector2 BasePos => _useCustomBasePos
        ? (Vector2)transform.TransformPoint(_localOffset)
        : (Vector2)transform.position;

    private void Awake()
    {
        ApplySettings();
    }

    private void ApplySettings()
    {
        var renderer = GetComponent<SpriteRenderer>();
        if (renderer == null) return;

        var mpb = new MaterialPropertyBlock();
        renderer.GetPropertyBlock(mpb);

        Vector2 basePos = BasePos;
        // 법선벡터 = 벽 뒤 방향 = depth 증가 방향 (그대로 사용)
        float rad = _normalAngle * Mathf.Deg2Rad;
        Vector2 depthDir = new Vector2(Mathf.Cos(rad), Mathf.Sin(rad));

        mpb.SetVector(WallBasePosId, new Vector4(basePos.x, basePos.y, 0, 0));
        mpb.SetVector(WallDepthDirId, new Vector4(depthDir.x, depthDir.y, 0, 0));
        mpb.SetFloat(DepthBiasId, _depthBias);

        renderer.SetPropertyBlock(mpb);
    }

#if UNITY_EDITOR
    private void OnValidate()
    {
        if (gameObject.activeInHierarchy)
        {
            ApplySettings();
        }
    }

    private void OnDrawGizmosSelected()
    {
        float rad = _normalAngle * Mathf.Deg2Rad;
        Vector2 normalDir = new Vector2(Mathf.Cos(rad), Mathf.Sin(rad));

        Vector3 basePos = BasePos;

        // Base Position 표시
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(basePos, 0.2f);

        // 법선벡터 방향 표시 (벽 뒤 = 오클루전 방향)
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(basePos, basePos + (Vector3)(normalDir * 2f));

        // 벽 표면 라인 표시 (법선에 수직)
        Gizmos.color = Color.green;
        Vector2 tangent = new Vector2(-normalDir.y, normalDir.x);
        Gizmos.DrawLine(basePos - (Vector3)(tangent * 1.5f), basePos + (Vector3)(tangent * 1.5f));
    }
#endif
}
