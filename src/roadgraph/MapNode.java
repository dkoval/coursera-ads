package roadgraph;

import geography.GeographicPoint;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

/**
 * Represents a node (or a vertex) of a {@link roadgraph.MapGraph}.
 *
 * @author dkoval
 */
public class MapNode {
    private final GeographicPoint location;
    private final List<MapEdge> edges = new LinkedList<>();

    public MapNode(GeographicPoint location) {
        this.location = location;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public List<MapEdge> getEdges() {
        return Collections.unmodifiableList(edges);
    }

    public int getNumEdges() {
        return edges.size();
    }

    public void addEdge(MapNode to, String roadName, String roadType, double length) {
        MapEdge newEdge = new MapEdge(this, to, roadName, roadType, length);
        edges.add(newEdge);
    }
}
