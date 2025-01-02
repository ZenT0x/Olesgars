using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomMovement : MonoBehaviour
{
    public Pathfinding pathfinding;
    public float minWaitTime = 1f; // Minimum wait time at a destination
    public float maxWaitTime = 3f; // Maximum wait time at a destination

    private void Start()
    {
        if (pathfinding == null)
        {
            Debug.LogError("Pathfinding component is not assigned!");
            return;
        }

        StartCoroutine(RandomMove());
    }

    private IEnumerator RandomMove()
    {
        while (true)
        {
            // Choose a random node
            int randomIndex = Random.Range(0, pathfinding.graphNodes.Count);
            Pathfinding.Node randomNode = pathfinding.graphNodes[randomIndex];
            Vector3 destination = randomNode.position;

            Debug.Log("Chosen random destination: " + destination);

            // Move to the chosen destination
            pathfinding.MoveTo(destination);

            // Wait until the pathfinding has completed the path
            while (pathfinding.currentPath.Count > 0)
            {
                Debug.Log("Moving along path to destination: " + destination);
                yield return null;
            }

            Debug.Log("Reached destination: " + destination);

            // Wait for a random amount of time before choosing a new destination
            float waitTime = Random.Range(minWaitTime, maxWaitTime);
            Debug.Log("Waiting for " + waitTime + " seconds at destination.");
            yield return new WaitForSeconds(waitTime);
        }
    }
}
