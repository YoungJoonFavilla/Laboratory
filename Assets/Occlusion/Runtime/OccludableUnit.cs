using UnityEngine;

/// <summary>
/// 오클루전 대상 유닛에 붙이는 컴포넌트.
/// 유닛의 레이어가 OcclusionRenderFeature의 unitLayer에 포함되어야 함.
/// </summary>
[ExecuteAlways]
public class OccludableUnit : MonoBehaviour
{
    [Header("Materials")]
    [Tooltip("UnitDepthWrite 셰이더를 사용하는 머티리얼")]
    [SerializeField] private Material _depthWriteMaterial;

    [Tooltip("UnitOcclude 셰이더를 사용하는 머티리얼")]
    [SerializeField] private Material _occludeMaterial;

    private Renderer[] _renderers;

    private void OnEnable()
    {
        _renderers = GetComponentsInChildren<Renderer>();
    }

    /// <summary>
    /// 런타임에서 머티리얼 교체가 필요한 경우 사용
    /// </summary>
    public void SetupMaterials(Material depthWrite, Material occlude)
    {
        _depthWriteMaterial = depthWrite;
        _occludeMaterial = occlude;
    }

#if UNITY_EDITOR
    private void OnDrawGizmosSelected()
    {
        // 유닛 위치 표시
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, 0.2f);
    }
#endif
}
