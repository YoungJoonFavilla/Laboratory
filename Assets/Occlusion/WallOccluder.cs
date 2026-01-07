using UnityEngine;

[ExecuteAlways]
[RequireComponent(typeof(SpriteRenderer))]
public class WallOccluder : MonoBehaviour
{
    [Header("Depth Settings")]
    [SerializeField] private float _projectionAngle = 129.5f;

    [Tooltip("양수 = 벽이 뒤로 밀림 (덜 가림), 음수 = 벽이 앞으로 옴 (더 가림)")]
    [SerializeField] private float _depthOffset = 0f;

    private SpriteRenderer _renderer;
    private MaterialPropertyBlock _propBlock;

    private static readonly int DepthDirId = Shader.PropertyToID("_DepthDir");
    private static readonly int DepthOffsetId = Shader.PropertyToID("_DepthOffset");

    public Vector2 DepthDir
    {
        get
        {
            float rad = _projectionAngle * Mathf.Deg2Rad;
            return new Vector2(Mathf.Cos(rad), Mathf.Sin(rad)).normalized;
        }
    }

    private void OnEnable()
    {
        _renderer = GetComponent<SpriteRenderer>();
        _propBlock = new MaterialPropertyBlock();
    }

    private void Update()
    {
        if (_renderer == null) return;

        _renderer.GetPropertyBlock(_propBlock);
        _propBlock.SetVector(DepthDirId, DepthDir);
        _propBlock.SetFloat(DepthOffsetId, _depthOffset);
        _renderer.SetPropertyBlock(_propBlock);
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 pos = transform.position;

        // 투영 방향 표시
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(pos, pos + (Vector3)(DepthDir * 2f));
        Gizmos.DrawWireSphere(pos, 0.2f);
    }
}
