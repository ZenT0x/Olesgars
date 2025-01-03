using UnityEngine;

public class PlayerPathfinding : MonoBehaviour
{
    public Pathfinding pathfinding; // Reference to the Pathfinding component
    private Camera mainCamera;

    void Start()
    {
        if (pathfinding == null)
        {
            Debug.LogError("Pathfinding component is not assigned!");
            return;
        }

        mainCamera = Camera.main;
        if (mainCamera == null)
        {
            Debug.LogError("Main camera is not assigned or found!");
            return;
        }
    }

    void Update()
    {
        HandleMouseInput();
    }

    void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(1)) // Right mouse button
        {
            Vector3 mousePosition = Input.mousePosition;
            Vector3 worldPosition = mainCamera.ScreenToWorldPoint(new Vector3(mousePosition.x, mousePosition.y, -mainCamera.transform.position.z));

            Debug.Log("Mouse clicked at: " + worldPosition);

            // Trigger movement using the pathfinding system
            pathfinding.MoveTo(worldPosition);
        }
    }
}
