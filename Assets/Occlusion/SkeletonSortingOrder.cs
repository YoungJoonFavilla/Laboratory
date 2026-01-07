using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
public class SkeletonSortingOrder : MonoBehaviour
{
    [Header("Sorting Settings")]
    [SerializeField] private int _baseSortingOrder = 2000;
    [SerializeField] private float _sortingScale = 100f;

    private MeshRenderer _renderer;
    private Vector2 _depthDir;

    private void Awake()
    {
        _renderer = GetComponent<MeshRenderer>();
        UpdateDepthDir();
    }

    private void Update()
    {
        float depth = Vector2.Dot(transform.position, _depthDir);
        // depth가 작을수록(뒤에 있을수록) order가 낮음 → 먼저 그려짐 → 뒤에 보임
        _renderer.sortingOrder = _baseSortingOrder + Mathf.RoundToInt(depth * _sortingScale);
    }

    private void UpdateDepthDir()
    {
        float rad = OccConstant.DEFAULT_NORMAL_ANGLE * Mathf.Deg2Rad;
        _depthDir = new Vector2(Mathf.Cos(rad), Mathf.Sin(rad));
    }

#if UNITY_EDITOR
    private void OnValidate()
    {
        UpdateDepthDir();
    }
#endif
}
