package roadgraph;

import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * A class which represents a graph of geographic locations.
 * Nodes in the graph are intersections or dead ends between roads,
 * while the edges are the road segments between these intersections.
 *
 * @author UCSD MOOC development team and YOU
 */
public class MapGraph {
    //TODO: Add your member variables here in WEEK 2
    private final Map<GeographicPoint, MapNode> nodes;

    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        // TODO: Implement in this constructor in WEEK 2
        this.nodes = new LinkedHashMap<>();
    }

    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        System.out.println("DONE.");

        // You can use this method for testing.

		/* Use this code in Week 3 End of Week Quiz
        MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		*/
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        //TODO: Implement this method in WEEK 2
        return nodes.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        //TODO: Implement this method in WEEK 2
        return nodes.keySet();
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        //TODO: Implement this method in WEEK 2
        int numEdges = 0;
        for (MapNode node: nodes.values()) {
            numEdges += node.getNumEdges();
        }
        return numEdges;
    }

    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        // TODO: Implement this method in WEEK 2
        if (location == null || nodes.containsKey(location)) {
            return false;
        }
        nodes.put(location, new MapNode(location));
        return true;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {

        //TODO: Implement this method in WEEK 2
        if (length < 0.0) {
            throw new IllegalArgumentException("`length` must not be negative");
        }

        if (from == null || !nodes.containsKey(from)) {
            throw new IllegalArgumentException("GeographicPoint `from` must not be null and must exist in the graph");
        }

        if (to == null || !nodes.containsKey(to)) {
            throw new IllegalArgumentException("GeographicPoint `to` must not be null and must exist in the graph");
        }

        MapNode fromNode = nodes.get(from);
        MapNode toNode = nodes.get(to);

        // add a directed edge between `from` and `to` MapNodes
        fromNode.addEdge(toNode, roadName, roadType, length);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 2
        MapNode startNode = nodes.get(start);
        MapNode goalNode = nodes.get(goal);
        Map<MapNode, MapNode> parentMap = new LinkedHashMap<>();

        // init the BFS queue & the visited nodes set
        Queue<MapNode> queue = new LinkedList<>();
        Set<MapNode> visited = new HashSet<>();

        // push start node onto the BFS queue and add to the visited nodes set
        queue.add(startNode);
        visited.add(startNode);

        while (!queue.isEmpty()) {
            MapNode current = queue.remove();

            // goal is reached
            if (current.equals(goalNode)) {
                return createPath(startNode, goalNode, parentMap);
            }

            // for each of current's neighbors, which is not in the visited nodes set
            for (MapEdge edge : current.getEdges()) {
                MapNode next = edge.getTo();
                if (!visited.contains(next)) {
                    // Hook for visualization. See writeup.
                    nodeSearched.accept(next.getLocation());

                    // add next to the visited nodes set
                    visited.add(next);
                    // add current as next's parent
                    parentMap.put(next, current);
                    // enqueue next onto the BFS queue
                    queue.add(next);
                }
            }
        }

        // if we get here, there is no parentMap
        return null;
    }

    private List<GeographicPoint> createPath(MapNode start, MapNode goal, Map<MapNode, MapNode> parentMap) {
        LinkedList<GeographicPoint> path = new LinkedList<>();
        MapNode current = goal;
        while (current != start) {
            path.addFirst(current.getLocation());
            current = parentMap.get(current);
        }
        path.addFirst(start.getLocation());
        return path;
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }
}
