import org.w3c.dom.Node;

import java.lang.reflect.Array;
import java.util.*;

/**
 * PathFinder class containing Dijkstra's shortest path algorithm.
 * Students must implement the algorithm using a PriorityQueue.
 */
public class PathFinder {

    private Graph graph;

    public PathFinder(Graph graph) {
        this.graph = graph;
    }

    /**
     * Implements Dijkstra's shortest path algorithm using a PriorityQueue.
     *
     * @param startIndex the index of the starting node
     * @param endIndex   the index of the destination node
     * @return a DijkstraResult containing all steps and the final path
     */
    public DijkstraResult findShortestPath(int startIndex, int endIndex) {
        DijkstraResult result = new DijkstraResult();

        // 1. INITIALIZATION
        // TODO: Initialize distances array with Integer.MAX_VALUE for all nodes
        int size = graph.NUM_NODES;
        int[] distances = new int[size];
         for(int i = 0; i < size; i++){
             distances[i] = Integer.MAX_VALUE;
         }

        // TODO: Initialize previous array with -1 for all nodes
        int[] previous = new int[size];
         for(int i = 0; i < size; i++){
             previous[i] = -1;
         }

        // TODO: Initialize visited set
        Set<Integer> visited = new HashSet<>();
        // TODO: Set distance to start node as 0
        distances[startIndex] = 0;


        // TODO: Create a PriorityQueue of NodeDistance objects
        // The PriorityQueue should order by distance (smallest first)
        // Hint: Use a Comparator or make NodeDistance implement Comparable
        PriorityQueue<NodeDistance> myqueue = new PriorityQueue<>(new Comparator<NodeDistance>() {
            @Override
            public int compare(NodeDistance o1, NodeDistance o2) {
                Integer temp = o1.distance;
                Integer temp2 = o2.distance;

                return temp.compareTo(temp2);
            }
        });


        // TODO: Add the starting node to the priority queue with distance 0
        NodeDistance starting = new NodeDistance(startIndex, 0);
        myqueue.add(starting);

        // 2. MAIN LOOP
        // TODO: While the priority queue is not empty...
        while(!myqueue.isEmpty()){
            // a. Poll the node with smallest distance from the priority queue
            NodeDistance newN = myqueue.poll();

            // b. If this node has already been visited, skip it (continue)
            if(visited.contains(newN.nodeIndex)){
                continue;
            }

            // c. If this node is the endIndex, we've found the shortest path - stop.
            if(endIndex == newN.nodeIndex){
                break;
            }

            // d. Mark current node as visited
            visited.add(newN.nodeIndex);

            // e. VITAL: Record the step for the visualizer!
            result.addStep(newN.nodeIndex, visited, distances, previous);


            // f. Iterate through all neighbors of the current node

            // For each neighbor:
            // - Calculate new distance: distances[current] + edge weight
            // - If new distance < distances[neighbor]:
            //     * Update distances[neighbor]
            //     * Update previous[neighbor] = current
            //     * Add neighbor to priority queue with new distance
            for( int k : graph.getNeighbors(newN.nodeIndex)){
                int distanceN = distances[newN.nodeIndex] + graph.getEdgeWeight(newN.nodeIndex, k);
                if(distanceN < distances[k]){
                    distances[k] = distanceN;
                    previous[k] = newN.nodeIndex;
                    myqueue.add(new NodeDistance(k, distanceN));
                }
            }


        }


        // 3. RECONSTRUCT PATH
        // TODO: Call helper method to get the path list
        // TODO: result.setFinalPath(path, distances[endIndex]);
        List newL = reconstructPath(previous, startIndex, endIndex);
        result.setFinalPath(newL, distances[endIndex]);
        return result;
    }

    /**
     * Helper method to reconstruct the path from start to end.
     *
     * @param previous   array where previous[i] is the node before i
     * @param startIndex the starting node index
     * @param endIndex   the ending node index
     * @return list of node indices representing the path from start to end
     */
    private List reconstructPath(int[] previous, int startIndex, int endIndex) {
        List path = new ArrayList<>();
        // TODO: Trace backwards from endIndex using the previous[] array
        // TODO: Don't forget to reverse the list so it goes Start -> End!

        int count = endIndex;
        System.out.println("End ");
        while((count != -1)){
              path.add(count);
              count = previous[count];
      }
        Collections.reverse(path);
        return path;
    }

    /**
     * Inner class to represent a node and its distance in the priority queue.
     * Students must implement Comparable to allow PriorityQueue ordering.
     */
    private static class NodeDistance {
        int nodeIndex;
        int distance;

        public NodeDistance(int nodeIndex, int distance) {
            this.nodeIndex = nodeIndex;
            this.distance = distance;
        }

    }
}