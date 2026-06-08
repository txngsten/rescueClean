package solution.Pathfinding;

/**
 * An immutable unit of work submitted to the RoutingEngine. Tasks are split into
 * OUTBOUND (base->building, low priority) and RETURN (building->base, high
 * priority), plus two D*-Lite-only types: EDGE_CHANGE and COLLAPSE, which feed
 * map updates into the shared incremental search instance. The kind determines
 * which internal queue the task waits in and which worker handles it.
 */
public final class PathTask {

    public enum Kind {
        OUTBOUND,
        RETURN,
        // D* Lite only
        EDGE_CHANGE,
        // D* Lite only
        COLLAPSE
    }

    private final Kind kind;
    private final int vehicle;     // vehicle this work concerns; -1 for EDGE_CHANGE/COLLAPSE
    private final String start;    // path start (vehicle's current location)
    private final String goal;     // path goal
    private final String from;     // EDGE_CHANGE: arc source; COLLAPSE: collapsed node
    private final String to;       // EDGE_CHANGE: arc target; unused for COLLAPSE
    private final long createdAtNanos; // for latency measurement / FIFO tie-breaking

    private PathTask(Kind kind, int vehicle, String start, String goal,
                     String from, String to) {
        this.kind = kind;
        this.vehicle = vehicle;
        this.start = start;
        this.goal = goal;
        this.from = from;
        this.to = to;
        this.createdAtNanos = System.nanoTime();
    }

    public static PathTask outbound(int vehicle, String start, String goal) {
        return new PathTask(Kind.OUTBOUND, vehicle, start, goal, null, null);
    }

    public static PathTask returnToBase(int vehicle, String start, String goal) {
        return new PathTask(Kind.RETURN, vehicle, start, goal, null, null);
    }

    public static PathTask edgeChange(String from, String to) {
        return new PathTask(Kind.EDGE_CHANGE, -1, null, null, from, to);
    }

    public static PathTask collapse(String node) {
        return new PathTask(Kind.COLLAPSE, -1, null, null, node, null);
    }

    public Kind kind()            { return kind; }
    public int vehicle()          { return vehicle; }
    public String start()         { return start; }
    public String goal()          { return goal; }
    public String from()          { return from; }
    public String to()            { return to; }
    public long createdAtNanos()  { return createdAtNanos; }

    @Override
    public String toString() {
        switch (kind) {
            case EDGE_CHANGE:
                return "EDGE_CHANGE(" + from + "->" + to + ")";
            case COLLAPSE:
                return "COLLAPSE(" + from + ")";
            case OUTBOUND:
                return "OUTBOUND(v" + vehicle + " " + start + "->" + goal + ")";
            case RETURN:
                return "RETURN(v" + vehicle + " " + start + "->" + goal + ")";
            default:
                return "PathTask(" + kind + ")";
        }
    }
}