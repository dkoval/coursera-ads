package roadgraph;

/**
 * A directed edge in a {@link MapGraph}.
 *
 * @author dkoval
 */
public class MapEdge {

    /**
     * The start of the edge.
     */
    private final MapNode from;

    /**
     * The end of the edge.
     */
    private final MapNode to;

    /**
     * The name of the road.
     */
    private final String roadName;

    /**
     * The type of the road.
     */
    private final String roadType;

    /**
     * The length of the road segment, in km.
     */
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
