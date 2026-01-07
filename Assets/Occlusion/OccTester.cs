using UnityEngine;
using UnityEngine.InputSystem;

public class OccTester : MonoBehaviour
{
    [SerializeField] private float speed = 5f;
    [SerializeField] private Vector3 cameraOffset = new Vector3(0, 0, -10);

    private Camera mainCam;

    void Start()
    {
        mainCam = Camera.main;
    }

    void Update()
    {
        Vector3 move = Vector3.zero;

        var keyboard = Keyboard.current;
        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed) move.y += 1f;
            if (keyboard.sKey.isPressed) move.y -= 1f;
            if (keyboard.aKey.isPressed) move.x -= 1f;
            if (keyboard.dKey.isPressed) move.x += 1f;
        }

        transform.position += move.normalized * speed * Time.deltaTime;
    }

    void LateUpdate()
    {
        mainCam.transform.position = transform.position + cameraOffset;
    }
}
