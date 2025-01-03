using System.Collections.Generic;
using UnityEngine;

public class Pathfinding : MonoBehaviour
{
    public float speedCoefficient = 5f; // Speed coefficient for movement
    public bool visualizeGraph = true; // Toggle for visualizing the graph
    public int graphStartX = 0; // Start X of the graph
    public int graphStartY = 0; // Start Y of the graph
    public int graphEndX = 10; // End X of the graph
    public int graphEndY = 10; // End Y of the graph

    public float nodeDensity = 1f; // Distance between nodes
    public float hitboxRadius = 0.5f; // Radius of the character's hitbox
    public LayerMask obstacleLayer; // Layer to detect obstacles (exclude character's collider)

    public List<Vector3> currentPath = new List<Vector3>(); // Stores the current path

    // Class to represent a node in the graph
    public class Node
    {
        public Vector3 position;
        public List<Edge> edges = new List<Edge>();

        public Node(Vector3 pos)
        {
            position = pos;
        }
    }

    // Class to represent an edge between nodes
    public class Edge
    {
        public Node fromNode;
        public Node toNode;
        public float cost;

        public Edge(Node from, Node to, float edgeCost)
        {
            fromNode = from;
            toNode = to;
            cost = edgeCost;
        }
    }

    public List<Node> graphNodes = new List<Node>(); // List of all nodes in the graph

    void Start()
    {
        if (obstacleLayer == LayerMask.GetMask("Nothing"))
        {
            Debug.LogWarning("Warning: obstacleLayer is set to Nothing. Pathfinding may not work correctly.");
        }
        GenerateGraph();
        Debug.Log("Graph generated with " + graphNodes.Count + " nodes.");
    }

    void Update()
    {
        if (visualizeGraph)
        {
            VisualizeGraph();
        }

        if (currentPath.Count > 1)
        {
            VisualizePath();
        }
    }

    void GenerateGraph()
    {
        // Clear previous nodes
        graphNodes.Clear();

        // Generate nodes on a grid based on node density
        for (float x = graphStartX; x <= graphEndX; x += nodeDensity)
        {
            for (float y = graphStartY; y <= graphEndY; y += nodeDensity)
            {
                Vector3 nodePosition = new Vector3(x, y, 0);

                // Check if there is an obstacle at the position
                Collider2D collider = Physics2D.OverlapCircle(nodePosition, hitboxRadius, obstacleLayer);
                if (collider == null)
                {
                    Node newNode = new Node(nodePosition);
                    graphNodes.Add(newNode);
                }
            }
        }

        Debug.Log("Nodes generated: " + graphNodes.Count);

        // Connect nodes with their 4 neighbors
        foreach (Node node in graphNodes)
        {
            foreach (Node potentialNeighbor in graphNodes)
            {
                float distance = Vector3.Distance(node.position, potentialNeighbor.position);
                if (Mathf.Approximately(distance, nodeDensity))
                {
                    // Ensure the edge is traversable
                    Vector3 direction = (potentialNeighbor.position - node.position).normalized;
                    RaycastHit2D hit = Physics2D.CircleCast(node.position, hitboxRadius, direction, nodeDensity, obstacleLayer);
                    if (hit.collider == null)
                    {
                        Edge edge = new Edge(node, potentialNeighbor, distance);
                        node.edges.Add(edge);
                    }
                }
            }
        }

        Debug.Log("Graph connections established.");
    }

    void VisualizeGraph()
    {
        foreach (Node node in graphNodes)
        {
            foreach (Edge edge in node.edges)
            {
                Debug.DrawLine(node.position, edge.toNode.position, Color.green);
            }
        }
    }

    void VisualizePath()
    {
        for (int i = 0; i < currentPath.Count - 1; i++)
        {
            Debug.DrawLine(currentPath[i], currentPath[i + 1], Color.blue);
        }
    }

    public List<Vector3> FindPath(Vector3 start, Vector3 destination)
    {
        Node startNode = GetClosestNode(start);
        Node destinationNode = GetClosestNode(destination);

        if (startNode == null || destinationNode == null)
        {
            Debug.LogError("Start or destination node not found!");
            return null;
        }

        Debug.Log("Start node: " + startNode.position + ", Destination node: " + destinationNode.position);

        List<Node> reachableNodes = GetReachableNodes(startNode);
        if (!reachableNodes.Contains(destinationNode))
        {
            destinationNode = GetClosestReachableNode(destination, reachableNodes);
            Debug.Log("Redirected to closest reachable node: " + destinationNode.position);
        }

        Dictionary<Node, float> distances = new Dictionary<Node, float>();
        Dictionary<Node, Node> previousNodes = new Dictionary<Node, Node>();
        HashSet<Node> visited = new HashSet<Node>();
        PriorityQueue<Node> priorityQueue = new PriorityQueue<Node>();

        foreach (Node node in graphNodes)
        {
            distances[node] = float.MaxValue;
            previousNodes[node] = null;
        }

        distances[startNode] = 0;
        priorityQueue.Enqueue(startNode, 0);

        while (priorityQueue.Count > 0)
        {
            Node currentNode = priorityQueue.Dequeue();
            if (currentNode == destinationNode)
                break;

            visited.Add(currentNode);

            foreach (Edge edge in currentNode.edges)
            {
                Node neighbor = edge.toNode;
                if (visited.Contains(neighbor)) continue;

                float tentativeDistance = distances[currentNode] + edge.cost;
                if (tentativeDistance < distances[neighbor])
                {
                    distances[neighbor] = tentativeDistance;
                    previousNodes[neighbor] = currentNode;
                    priorityQueue.Enqueue(neighbor, tentativeDistance);
                }
            }
        }

        // Reconstruct path
        List<Vector3> path = new List<Vector3>();
        Node current = destinationNode;
        while (current != null)
        {
            path.Add(current.position);
            current = previousNodes[current];
        }

        path.Reverse();

        Debug.Log("Path found with " + path.Count + " steps.");
        return path;
    }

    Node GetClosestNode(Vector3 position)
    {
        Node closestNode = null;
        float closestDistance = float.MaxValue;

        foreach (Node node in graphNodes)
        {
            float distance = Vector3.Distance(position, node.position);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestNode = node;
            }
        }

        return closestNode;
    }

    Node GetClosestReachableNode(Vector3 position, List<Node> reachableNodes)
    {
        Node closestNode = null;
        float closestDistance = float.MaxValue;

        foreach (Node node in reachableNodes)
        {
            float distance = Vector3.Distance(position, node.position);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestNode = node;
            }
        }

        return closestNode;
    }

    List<Node> GetReachableNodes(Node startNode)
    {
        List<Node> reachableNodes = new List<Node>();
        Queue<Node> queue = new Queue<Node>();
        HashSet<Node> visited = new HashSet<Node>();

        queue.Enqueue(startNode);
        visited.Add(startNode);

        while (queue.Count > 0)
        {
            Node current = queue.Dequeue();
            reachableNodes.Add(current);

            foreach (Edge edge in current.edges)
            {
                Node neighbor = edge.toNode;
                if (!visited.Contains(neighbor))
                {
                    visited.Add(neighbor);
                    queue.Enqueue(neighbor);
                }
            }
        }

        Debug.Log("Reachable nodes from " + startNode.position + ": " + reachableNodes.Count);
        return reachableNodes;
    }

    public void MoveTo(Vector3 destination)
    {
        Debug.Log("Requested move to: " + destination);

        // Ensure the starting position snaps to the closest node
        Node closestStartNode = GetClosestNode(transform.position);
        if (closestStartNode != null)
        {
            transform.position = closestStartNode.position;
            Debug.Log("Snapped to closest start node: " + closestStartNode.position);
        }

        // Snap the destination to the closest reachable node if it is not accessible
        List<Node> reachableNodes = GetReachableNodes(closestStartNode);
        Node closestDestinationNode = GetClosestNode(destination);
        if (!reachableNodes.Contains(closestDestinationNode))
        {
            closestDestinationNode = GetClosestReachableNode(destination, reachableNodes);
            Debug.Log("Destination adjusted to closest reachable node: " + closestDestinationNode.position);
        }

        if (closestDestinationNode != null)
        {
            destination = closestDestinationNode.position;
        }

        currentPath = FindPath(transform.position, destination);
        if (currentPath != null)
        {
            Debug.Log("Starting path traversal.");
            StartCoroutine(FollowPath(currentPath));
        }
        else
        {
            Debug.LogError("No path found to destination.");
        }
    }

    System.Collections.IEnumerator FollowPath(List<Vector3> path)
    {
        Debug.Log("Following path with " + path.Count + " steps.");

        foreach (Vector3 point in path)
        {
            Debug.Log("Moving to point: " + point);

            while (Vector3.Distance(transform.position, point) > 0.01f) // Use a very small threshold
            {
                Debug.Log("Current distance to point: " + Vector3.Distance(transform.position, point));
                transform.position = Vector3.MoveTowards(transform.position, point, Time.deltaTime * speedCoefficient);
                yield return null;
            }

            // Force snap to the exact point to avoid inaccuracies
            transform.position = point;
            Debug.Log("Reached point: " + point);
        }

        currentPath.Clear(); // Clear path once movement is complete
        Debug.Log("Path traversal complete.");
    }
}

// Priority Queue implementation
public class PriorityQueue<T>
{
    private List<KeyValuePair<T, float>> elements = new List<KeyValuePair<T, float>>();

    public int Count => elements.Count;

    public void Enqueue(T item, float priority)
    {
        elements.Add(new KeyValuePair<T, float>(item, priority));
    }

    public T Dequeue()
    {
        int bestIndex = 0;
        for (int i = 1; i < elements.Count; i++)
        {
            if (elements[i].Value < elements[bestIndex].Value)
            {
                bestIndex = i;
            }
        }

        T bestItem = elements[bestIndex].Key;
        elements.RemoveAt(bestIndex);
        return bestItem;
    }
}
