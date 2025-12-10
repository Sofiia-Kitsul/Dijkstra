import java.util.*;

/**
 * Graph class using an Adjacency Matrix representation.
 * Students must implement the core graph functionality.
 */
public class Graph {

    // Node names for the campus map (DO NOT MODIFY)
    public static final String[] NODE_NAMES = {
            "Gate", "Student Parking", "Senior Parking", "Circle", "Admissions",
            "Business Office", "PA", "Athletics", "Fountain", "Cohen",
            "US 100", "Library", "CHH", "Mariani", "VA",
            "Faculty Parking", "GD", "Science", "BD"
    };

    // Number of nodes in the graph
    public static final int NUM_NODES = NODE_NAMES.length;

    // TODO: Declare your adjacency matrix here
    private int[][] adjacencyMatrix;

    public Graph() {
        // TODO: Initialize your matrix with size [NUM_NODES][NUM_NODES]
        adjacencyMatrix = new int[NUM_NODES][NUM_NODES];
        // These calls will populate your matrix once addEdge is implemented
        addEdge("Gate", "Student Parking", 3);
        addEdge("Gate", "Circle", 1);
        addEdge("Gate", "Senior Parking", 1);
        addEdge("Student Parking", "Circle", 2);
        addEdge("Student Parking", "PA", 2);
        addEdge("Student Parking", "Athletics", 2);
        addEdge("Senior Parking", "Circle", 1);
        addEdge("Senior Parking", "Admissions", 1);
        addEdge("Senior Parking", "Business Office", 1);
        addEdge("Circle", "Admissions", 1);
        addEdge("Circle", "PA", 4);
        addEdge("Circle", "Fountain", 1);
        addEdge("Admissions", "Business Office", 1);
        addEdge("Admissions", "Fountain", 2);
        addEdge("Business Office", "US 100", 3);
        addEdge("PA", "Athletics", 4);
        addEdge("PA", "Cohen", 4);
        addEdge("PA", "Mariani", 3);
        addEdge("Athletics", "Mariani", 5);
        addEdge("Athletics", "Science", 7);
        addEdge("Fountain", "Cohen", 1);
        addEdge("Fountain", "Library", 1);
        addEdge("Fountain", "US 100", 2);
        addEdge("Cohen", "Mariani", 4);
        addEdge("Cohen", "Library", 1);
        addEdge("US 100", "Library", 2);
        addEdge("US 100", "VA", 2);
        addEdge("US 100", "Faculty Parking", 5);
        addEdge("Library", "CHH", 1);
        addEdge("Library", "VA", 4);
        addEdge("CHH", "Mariani", 3);
        addEdge("CHH", "VA", 3);
        addEdge("Mariani", "Science", 1);
        addEdge("Mariani", "GD", 8);
        addEdge("VA", "Faculty Parking", 2);
        addEdge("VA", "GD", 1);
        addEdge("Science", "BD", 1);
        addEdge("Science", "GD", 8);
        addEdge("BD", "GD", 7);
        addEdge("BD", "Science", 3);
    }

    /**
     * Adds a bidirectional edge between two nodes.
     * * @param from  the name of the first node
     * @param to    the name of the second node  
     * @param weight the weight/distance of the edge
     */
    public void addEdge(String from, String to, int weight) {
        // TODO: Get indices for 'from' and 'to'
        // TODO: Update matrix at [i][j] and [j][i]
        int fromInd = getNodeIndex(from);
        int toInd = getNodeIndex(to);

        adjacencyMatrix[fromInd][toInd] = weight;
        adjacencyMatrix[toInd][fromInd] = weight;

    }

    /**
     * Gets the index of a node given its name.
     * * @param name the name of the node
     * @return the index of the node in NODE_NAMES array, or -1 if not found
     */
    public int getNodeIndex(String name) {
        // TODO: Search NODE_NAMES array for the string
        int index = 0;
        for(int i = 0; i < NUM_NODES; i++){
            if(NODE_NAMES[i].equals(name)){
                index = i;
            }
        }
        return index;
    }

    /**
     * Gets the weight of the edge between two nodes.
     * * @param fromIndex the index of the source node
     * @param toIndex  the index of the destination node
     * @return the weight of the edge, or 0 if no connection
     */
    public int getEdgeWeight(int fromIndex, int toIndex) {
        // TODO: Return value from matrix
        int returning = adjacencyMatrix[fromIndex][toIndex];
        return returning;
    }

    /**
     * Checks if there is an edge between two nodes.
     */
    public boolean hasEdge(int fromIndex, int toIndex) {
        // TODO: Check if weight > 0
        if(getEdgeWeight(fromIndex, toIndex) > 0){
            return true;
        }
        return false;
    }

    /**
     * Gets all neighbors of a node (nodes with direct edges).
     * * @param nodeIndex the index of the node
     * @return a list of indices of neighboring nodes
     */
    public List<Integer> getNeighbors(int nodeIndex) {
        List<Integer> neighbors = new ArrayList<>();
        // TODO: Iterate through the row in your matrix
        // TODO: If a column has a weight > 0, add that column index to neighbors
        for(int i = 0; i < NUM_NODES; i++){
            if(adjacencyMatrix[nodeIndex][i] > 0){
                neighbors.add(adjacencyMatrix[nodeIndex][i]);
            }
        }
        return neighbors;
    }

    public String getNodeName(int index) {
        if (index >= 0 && index < NUM_NODES) {
            return NODE_NAMES[index];
        }
        return null;
    }
}