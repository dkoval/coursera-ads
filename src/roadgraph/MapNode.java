package roadgraph;

import geography.GeographicPoint;

import java.util.*;

/**
 * Represents a vertex (or a node) in {@link MapGraph}.
 *
 * @author dkoval
 */
public class MapNode implements Comparable<MapNode> {

    /**
     * The latitude and longitude of this node.
     */
    private final GeographicPoint location;

    /**
     * The list of edges out of this node.
     */
    private final Set<MapEdge> edges = new HashSet<>();

    /**
     * The predicted distance of this node (used in Week 3 algorithms).
     */
    private double distance = 0.0;

    /**
     * The actual distance of this node from start (used in Week 3 algorithms).
     */
    private double actualDistance = 0.0;

    public MapNode(GeographicPoint location) {
        this.location = location;
    }

    /**
     * @return the geographic location of this node.
     */
    public GeographicPoint getLocation() {
        return location;
    }

    /**
     * @return the edges out of this node.
     */
    public Set<MapEdge> getEdges() {
        return edges;
    }

    public void addEdge(MapNode to, String roadName, String roadType, double length) {
        MapEdge newEdge = new MapEdge(this, to, roadName, roadType, length);
        edges.add(newEdge);
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getActualDistance() {
        return actualDistance;
    }

    public void setActualDistance(double actualDistance) {
        this.actualDistance = actualDistance;
    }

    /**
     * Nodes are considered equal if their geographic locations are the same,
     * even if their street list is different.
     *
     * @param obj the node with which to compare.
     * @return {@code true}, two nodes are equal.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        MapNode that = (MapNode) obj;
        return Objects.equals(location, that.location);
    }

    /**
     * @return the hash code for this node, which is the hash code
     * for the underlying geographic location.
     */
    @Override
    public int hashCode() {
        return Objects.hash(location);
    }

    @Override
    public int compareTo(MapNode that) {
        return Double.compare(distance, that.distance);
    }
}
