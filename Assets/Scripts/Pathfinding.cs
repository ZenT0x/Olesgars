// Import required namespaces
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
        GenerateGraph();

        // Move to a destination
        MoveTo(new Vector3(4, 4, 0));
    }

    void Update()
    {
        if (visualizeGraph)
        {
            VisualizeGraph();
        }
    }

    void GenerateGraph()
    {
        // Clear previous nodes
        graphNodes.Clear();

        // Generate nodes on a grid
        for (int x = graphStartX; x <= graphEndX; x++)
        {
            for (int y = graphStartY; y <= graphEndY; y++)
            {
                Vector3 nodePosition = new Vector3(x, y, 0);

                // Check if there is a collider at the position
                Collider2D collider = Physics2D.OverlapPoint(nodePosition);
                if (collider == null)
                {
                    Node newNode = new Node(nodePosition);
                    graphNodes.Add(newNode);
                }
            }
        }

        // Connect nodes with their 4 neighbors
        foreach (Node node in graphNodes)
        {
            foreach (Node potentialNeighbor in graphNodes)
            {
                float distance = Vector3.Distance(node.position, potentialNeighbor.position);
                if (distance == 1f)
                {
                    Edge edge = new Edge(node, potentialNeighbor, distance);
                    node.edges.Add(edge);
                }
            }
        }
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

    public List<Vector3> FindPath(Vector3 start, Vector3 destination)
    {
        Node startNode = GetClosestNode(start);
        Node destinationNode = GetClosestNode(destination);

        if (startNode == null || destinationNode == null)
        {
            Debug.LogError("Start or destination node not found!");
            return null;
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

    public void MoveTo(Vector3 destination)
    {
        // Ensure the starting position snaps to the closest node
        Node closestStartNode = GetClosestNode(transform.position);
        if (closestStartNode != null)
        {
            transform.position = closestStartNode.position;
        }

        // Snap the destination to the closest node
        Node closestDestinationNode = GetClosestNode(destination);
        if (closestDestinationNode != null)
        {
            destination = closestDestinationNode.position;
        }

        List<Vector3> path = FindPath(transform.position, destination);
        if (path != null)
        {
            StartCoroutine(FollowPath(path));
        }
    }

    System.Collections.IEnumerator FollowPath(List<Vector3> path)
    {
        foreach (Vector3 point in path)
        {
            while (Vector3.Distance(transform.position, point) > 0.1f)
            {
                transform.position = Vector3.MoveTowards(transform.position, point, Time.deltaTime * speedCoefficient); // Use speed coefficient
                yield return null;
            }
        }
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
