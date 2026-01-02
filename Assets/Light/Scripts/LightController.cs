using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering.Universal;

public class LightController : MonoBehaviour
{
    [SerializeField] private float _moveSpeed = 5f;
    [SerializeField] private Camera _mainCamera;
    [SerializeField] private float _lightOuterRadius = 5f;
    [SerializeField] private float _lightIntensity = 1f;

    private void Start()
    {
        if (_mainCamera == null)
            _mainCamera = Camera.main;
    }

    private void Update()
    {
        Vector2 input = Vector2.zero;

        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        if (keyboard.wKey.isPressed) input.y += 1f;
        if (keyboard.sKey.isPressed) input.y -= 1f;
        if (keyboard.aKey.isPressed) input.x -= 1f;
        if (keyboard.dKey.isPressed) input.x += 1f;

        if (input != Vector2.zero)
        {
            input.Normalize();
            transform.position += (Vector3)(input * _moveSpeed * Time.deltaTime);
        }
    }

    private void LateUpdate()
    {
        if (_mainCamera != null)
        {
            Vector3 cameraPos = transform.position;
            cameraPos.z = _mainCamera.transform.position.z;
            _mainCamera.transform.position = cameraPos;
        }
    }

    [ContextMenu("Generate 60 Random Lights")]
    private void GenerateRandomLights()
    {
        for (int i = 0; i < 60; i++)
        {
            var lightObj = new GameObject($"Light_{i}");
            lightObj.transform.position = new Vector3(
                Random.Range(-100f, 100f),
                Random.Range(-100f, 100f),
                0f
            );

            var light2D = lightObj.AddComponent<Light2D>();
            light2D.lightType = Light2D.LightType.Point;
            light2D.pointLightOuterRadius = _lightOuterRadius;
            light2D.intensity = _lightIntensity;
            light2D.shadowsEnabled = false;
        }

        Debug.Log("Generated 60 random lights");
    }

    [ContextMenu("Clear All Lights")]
    private void ClearAllLights()
    {
        var lights = FindObjectsByType<Light2D>(FindObjectsSortMode.None);
        int count = 0;
        foreach (var light in lights)
        {
            if (light.gameObject.name.StartsWith("Light_"))
            {
                DestroyImmediate(light.gameObject);
                count++;
            }
        }
        Debug.Log($"Cleared {count} lights");
    }
}
