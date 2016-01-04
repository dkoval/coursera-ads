package roadgraph;

/**
 * Represents an edge of a {@link roadgraph.MapGraph}.
 *
 * @author dkoval
 */
public class MapEdge {
    private final MapNode from;
    private final MapNode to;
    private final String roadName;
    private final String roadType;
    private final double length;

    public MapEdge(MapNode from, MapNode to, String roadName, String roadType, double length) {
        this.from = from;
        this.to = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public MapNode getFrom() {
        return from;
    }

    public MapNode getTo() {
        return to;
    }

    public String getRoadName() {
        return roadName;
    }

    public String getRoadType() {
        return roadType;
    }

    public double getLength() {
        return length;
    }
}
