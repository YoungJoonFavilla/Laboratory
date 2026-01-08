using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 외곽선을 표시할 유닛을 관리 - Screen-Space Silhouette 방식
/// RenderFeature에서 선택된 렌더러 목록을 참조하여 실루엣 마스크 생성
/// </summary>
public class UnitOutlineManager : MonoBehaviour
{
    public static UnitOutlineManager Instance { get; private set; }

    [Header("Outline Settings")]
    [SerializeField] private Color _outlineColor = new Color(1f, 0.92f, 0.016f, 1f);
    [SerializeField] private float _outlineWidth = 3f;

    private Dictionary<Renderer, int> _selectedRenderers = new Dictionary<Renderer, int>();
    private List<Renderer> _selectedRenderersList = new List<Renderer>();
    private bool _listDirty = true;
    private MaterialPropertyBlock _mpb;

    private static readonly int OutlineSelectedId = Shader.PropertyToID("_OutlineSelected");

    public Color OutlineColor => _outlineColor;
    public float OutlineWidth => _outlineWidth;
    public int SelectedCount => _selectedRenderers.Count;

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
        _mpb = new MaterialPropertyBlock();
    }

    private void OnDestroy()
    {
        if (Instance == this)
            Instance = null;
    }

    /// <summary>
    /// RenderFeature에서 호출 - 선택된 렌더러 목록 반환
    /// </summary>
    public IReadOnlyList<Renderer> GetSelectedRenderers()
    {
        if (_listDirty)
        {
            _selectedRenderersList.Clear();
            foreach (var kvp in _selectedRenderers)
            {
                if (kvp.Key != null)
                    _selectedRenderersList.Add(kvp.Key);
            }
            _listDirty = false;
        }
        return _selectedRenderersList;
    }

    public void Select(GameObject target)
    {
        if (target == null) return;

        var renderers = target.GetComponentsInChildren<Renderer>();
        foreach (var r in renderers)
        {
            if (r == null) continue;

            if (_selectedRenderers.ContainsKey(r))
            {
                _selectedRenderers[r]++;
            }
            else
            {
                _selectedRenderers[r] = 1;
                _listDirty = true;
                ApplyOutlineSelection(r, true);
            }
        }
    }

    public void Deselect(GameObject target)
    {
        if (target == null) return;

        var renderers = target.GetComponentsInChildren<Renderer>();
        foreach (var r in renderers)
        {
            if (r == null) continue;

            if (_selectedRenderers.TryGetValue(r, out int count))
            {
                count--;
                if (count <= 0)
                {
                    _selectedRenderers.Remove(r);
                    _listDirty = true;
                    ApplyOutlineSelection(r, false);
                }
                else
                {
                    _selectedRenderers[r] = count;
                }
            }
        }
    }

    public void ClearSelection()
    {
        foreach (var r in _selectedRenderers.Keys)
        {
            if (r != null)
                ApplyOutlineSelection(r, false);
        }
        _selectedRenderers.Clear();
        _listDirty = true;
    }

    public bool IsSelected(Renderer renderer)
    {
        return renderer != null && _selectedRenderers.ContainsKey(renderer);
    }

    private void ApplyOutlineSelection(Renderer renderer, bool selected)
    {
        // 멀티 머티리얼 지원: 각 머티리얼 슬롯에 PropertyBlock 적용
        int materialCount = renderer.sharedMaterials.Length;
        for (int i = 0; i < materialCount; i++)
        {
            renderer.GetPropertyBlock(_mpb, i);
            _mpb.SetFloat(OutlineSelectedId, selected ? 1f : 0f);
            renderer.SetPropertyBlock(_mpb, i);
        }
    }
}
