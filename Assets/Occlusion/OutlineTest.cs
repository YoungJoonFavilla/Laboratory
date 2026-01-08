using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// 외곽선 테스트용 - 클릭으로 유닛 선택/해제
/// </summary>
public class OutlineTest : MonoBehaviour
{
    [SerializeField] private LayerMask _selectableLayer = -1;
    [SerializeField] private Camera _camera;

    private GameObject _currentSelection;

    private void Start()
    {
        if (_camera == null)
            _camera = Camera.main;
    }

    private void Update()
    {
        if (UnitOutlineManager.Instance == null) return;

        var mouse = Mouse.current;
        if (mouse == null) return;

        if (mouse.leftButton.wasPressedThisFrame)
        {
            Vector2 screenPos = mouse.position.ReadValue();
            Vector2 worldPos = _camera.ScreenToWorldPoint(screenPos);

            var hit = Physics2D.Raycast(worldPos, Vector2.zero, 0f, _selectableLayer);

            // 이전 선택 해제
            if (_currentSelection != null)
            {
                UnitOutlineManager.Instance.Deselect(_currentSelection);
                _currentSelection = null;
            }

            // 새로운 선택
            if (hit.collider != null)
            {
                _currentSelection = hit.collider.gameObject;
                UnitOutlineManager.Instance.Select(_currentSelection);
                Debug.Log($"[OutlineTest] Selected: {_currentSelection.name}");
            }
        }
    }
}
