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
    private final Map<GeographicPoint, MapNode> pointNodeMap;

    /**
     * Create a new empty MapGraph.
     */
    public MapGraph() {
        // TODO: Implement in this constructor in WEEK 2
        this.pointNodeMap = new HashMap<>();
    }

    /**
     * Get the number of vertices (road intersections) in the graph.
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        //TODO: Implement this method in WEEK 2
        return pointNodeMap.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints.
     */
    public Set<GeographicPoint> getVertices() {
        //TODO: Implement this method in WEEK 2
        return pointNodeMap.keySet();
    }

    /**
     * Get the number of road segments in the graph.
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        //TODO: Implement this method in WEEK 2
        int numEdges = 0;
        for (MapNode node: pointNodeMap.values()) {
            numEdges += node.getEdges().size();
        }
        return numEdges;
    }

    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection.
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        // TODO: Implement this method in WEEK 2
        if (location == null || pointNodeMap.containsKey(location)) {
            return false;
        }
        pointNodeMap.put(location, new MapNode(location));
        return true;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph.
     *
     * @param from     The starting point of the edge.
     * @param to       The ending point of the edge.
     * @param roadName The name of the road.
     * @param roadType The type of the road.
     * @param length   The length of the road, in km.
     * @throws IllegalArgumentException If the points have not already been added as nodes to the graph,
     * if any of the arguments is null, or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to,
                        String roadName, String roadType, double length) throws IllegalArgumentException {
        //TODO: Implement this method in WEEK 2
        if (from == null || !pointNodeMap.containsKey(from)) {
            throw new IllegalArgumentException("GeographicPoint `from` must not be null " +
                    "and must have already been added to the graph");
        }

        if (to == null || !pointNodeMap.containsKey(to)) {
            throw new IllegalArgumentException("GeographicPoint `to` must not be null " +
                    "and must have already been added to the graph");
        }

        if (roadName == null) {
            throw new IllegalArgumentException("`roadName` must not be null");
        }

        if (roadType == null) {
            throw new IllegalArgumentException("`roadType` must not be null");
        }

        if (length < 0.0) {
            throw new IllegalArgumentException("`length` must not be negative");
        }

        MapNode fromNode = pointNodeMap.get(from);
        MapNode toNode = pointNodeMap.get(to);

        // add a directed edge between `from` and `to` MapNodes
        fromNode.addEdge(toNode, roadName, roadType, length);
    }

    /**
     * Find the path from start to goal using breadth first search.
     *
     * @param start The starting location.
     * @param goal  The goal location.
     * @return The list of intersections that form the shortest (unweighted).
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search.
     *
     * @param start        The starting location.
     * @param goal         The goal location.
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted).
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 2
        // setup - check validity of inputs
        if (start == null || goal == null) {
            throw new NullPointerException("Cannot find route from or to null node");
        }

        MapNode startNode = pointNodeMap.get(start);
        MapNode goalNode = pointNodeMap.get(goal);

        if (startNode == null) {
            System.err.println("Start node " + start + " does not exist");
            return null;
        }

        if (goalNode == null) {
            System.err.println("Goal node " + goal + " does not exist");
            return null;
        }

        // setup to begin BFS
        Queue<MapNode> queue = new LinkedList<>();
        Set<MapNode> visited = new HashSet<>();
        Map<MapNode, MapNode> parentMap = new HashMap<>();

        // push start node onto the BFS queue
        queue.add(startNode);

        while (!queue.isEmpty()) {
            MapNode next = queue.remove();

            // hook for visualization
            nodeSearched.accept(next.getLocation());

            // goal is reached
            if (next.equals(goalNode)) {
                return createPath(startNode, goalNode, parentMap);
            }

            // for each of next's neighbors, which is not in the visited nodes set
            for (MapEdge edge : next.getEdges()) {
                MapNode neighbor = edge.getTo();

                if (visited.contains(neighbor)) {
                    continue;
                }

                // add neighbor to the visited nodes set
                visited.add(neighbor);
                // add next as neighbor's parent
                parentMap.put(neighbor, next);
                // enqueue neighbor onto the BFS queue
                queue.add(neighbor);
            }
        }

        // if we get here, there is no path
        System.out.println("No path found from " + start + " to " + goal);
        return null;
    }

    private List<GeographicPoint> createPath(MapNode start, MapNode goal, Map<MapNode, MapNode> parentMap) {
        LinkedList<GeographicPoint> path = new LinkedList<>();
        MapNode current = goal;

        while (!current.equals(start)) {
            path.addFirst(current.getLocation());
            current = parentMap.get(current);
        }

        // add start
        path.addFirst(start.getLocation());
        return path;
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm.
     *
     * @param start The starting location.
     * @param goal  The goal location.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm.
     *
     * @param start        The starting location.
     * @param goal         The goal location.
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3
        // setup - check validity of inputs
        if (start == null || goal == null) {
            throw new NullPointerException("Cannot find route from or to null node");
        }

        MapNode startNode = pointNodeMap.get(start);
        MapNode goalNode = pointNodeMap.get(goal);

        if (startNode == null) {
            System.err.println("Start node " + start + " does not exist");
            return null;
        }

        if (goalNode == null) {
            System.err.println("Goal node " + goal + " does not exist");
            return null;
        }

        // setup to begin Dijkstra
        PriorityQueue<MapNode> queue = new PriorityQueue<>();
        Set<MapNode> visited = new HashSet<>();
        Map<MapNode, MapNode> parentMap = new HashMap<>();
        pointNodeMap.values().forEach(mapNode -> mapNode.setDistance(Double.POSITIVE_INFINITY));

        // enqueue {start, 0} into the PQ
        startNode.setDistance(0.0);
        queue.add(startNode);

        while (!queue.isEmpty()) {
            MapNode next = queue.remove();

            // hook for visualization
            nodeSearched.accept(next.getLocation());

            // skip already visited nodes
            if (visited.contains(next)) {
                continue;
            }

            // add next to visited set
            visited.add(next);

            // the goal is reached
            if (next.equals(goalNode)) {
                return createPath(startNode, goalNode, parentMap);
            }

            // for each of next's neighbors, which is not in the visited nodes set
            for (MapEdge edge : next.getEdges()) {
                MapNode neighbor = edge.getTo();

                // once again, check & skip already visited nodes
                if (visited.contains(neighbor)) {
                    continue;
                }

                // cost from the starting node to reach n
                double distance = next.getDistance() + edge.getLength();

                if (distance < neighbor.getDistance()) {
                    // update neighbor's distance
                    neighbor.setDistance(distance);
                    // update next as neighbor's parent in the parent map
                    parentMap.put(neighbor, next);
                    // enqueue {neighbor, distance} into the PQ
                    queue.add(neighbor);
                }
            }
        }

        // if we get here, there is no path
        System.out.println("No path found from " + start + " to " + goal);
        return null;
    }

    /**
     * Find the path from start to goal using A-Star search.
     *
     * @param start The starting location.
     * @param goal  The goal location.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search.
     *
     * @param start        The starting location.
     * @param goal         The goal location.
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3
        // setup - check validity of inputs
        if (start == null || goal == null) {
            throw new NullPointerException("Cannot find route from or to null node");
        }

        MapNode startNode = pointNodeMap.get(start);
        MapNode goalNode = pointNodeMap.get(goal);

        if (startNode == null) {
            System.err.println("Start node " + start + " does not exist");
            return null;
        }

        if (goalNode == null) {
            System.err.println("Goal node " + goal + " does not exist");
            return null;
        }

        // setup to begin A*
        PriorityQueue<MapNode> queue = new PriorityQueue<>();
        Set<MapNode> visited = new HashSet<>();
        Map<MapNode, MapNode> parentMap = new HashMap<>();
        pointNodeMap.values().forEach(mapNode -> {
            mapNode.setActualDistance(Double.POSITIVE_INFINITY);
            mapNode.setDistance(Double.POSITIVE_INFINITY);
        });

        // enqueue {start, 0, 0} into the PQ
        startNode.setActualDistance(0.0);
        startNode.setDistance(0.0);
        queue.add(startNode);

        while (!queue.isEmpty()) {
            MapNode next = queue.remove();

            // hook for visualization
            nodeSearched.accept(next.getLocation());

            // skip already visited nodes
            if (visited.contains(next)) {
                continue;
            }

            // add next to visited set
            visited.add(next);

            // the goal is reached
            if (next.equals(goalNode)) {
                return createPath(startNode, goalNode, parentMap);
            }

            // for each of next's neighbors, which is not in the visited nodes set
            for (MapEdge edge : next.getEdges()) {
                MapNode neighbor = edge.getTo();

                // once again, check & skip already visited nodes
                if (visited.contains(neighbor)) {
                    continue;
                }

                // cost from the starting node to reach n
                double actualDistance = next.getActualDistance() + edge.getLength();
                // estimate of the cost of the cheapest path from n to the goal node - Euclidean distance
                double estimate = neighbor.getLocation().distance(goal);
                // predicted distance
                double distance = actualDistance + estimate;

                if (distance < neighbor.getDistance()) {
                    // update neighbor's distance
                    neighbor.setActualDistance(actualDistance);
                    neighbor.setDistance(distance);
                    // update next as neighbor's parent in the parent map
                    parentMap.put(neighbor, next);
                    // enqueue {neighbor, distance} into the PQ
                    queue.add(neighbor);
                }
            }
        }

        // if we get here, there is no path
        System.out.println("No path found from " + start + " to " + goal);
        return null;
    }

    public static void main(String[] args) {
		// Use this code in Week 3 End of Week Quiz
        System.out.print("Creating MapGraph... ");
        MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map... ");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("  DONE.\n");

		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);

		List<GeographicPoint> dijkstraRoute = theMap.dijkstra(start, end);
        System.out.println("Dijkstra route: " + dijkstraRoute);

        List<GeographicPoint> aStarRoute = theMap.aStarSearch(start, end);
        System.out.println("A* route:       " + aStarRoute);
    }
}
