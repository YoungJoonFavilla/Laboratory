using UnityEngine;

/// <summary>
/// 오클루전 대상 유닛에 붙이는 마커 컴포넌트.
/// 유닛의 레이어가 OcclusionRenderFeature의 unitLayer에 포함되어야 함.
/// </summary>
public class OccludableUnit : MonoBehaviour
{
#if UNITY_EDITOR
    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, 0.2f);
    }
#endif
}
