package solution.Pathfinding;

import java.util.List;

/**
 * The outcome of a path computation, delivered to the responder via the
 * RoutingEngine's result queue. Decoupled from the task so workers never touch
 * the outgoing message queue.
 */
public final class PathResult {

    private final PathTask.Kind kind;
    private final int vehicle;
    private final String goal;
    private final List<String> path;
    private final long computeNanos;

    public PathResult(PathTask.Kind kind, int vehicle, String goal,
                      List<String> path, long computeNanos) {
        this.kind = kind;
        this.vehicle = vehicle;
        this.goal = goal;
        this.path = path;
        this.computeNanos = computeNanos;
    }

    public PathTask.Kind kind()   { return kind; }
    public int vehicle()          { return vehicle; }
    public String goal()          { return goal; }
    public List<String> path()    { return path; }
    public long computeNanos()    { return computeNanos; }

    public boolean hasPath() {
        return path != null && !path.isEmpty();
    }

    @Override
    public String toString() {
        return "PathResult(" + kind + " v" + vehicle + " ->" + goal
                + (hasPath() ? " path=" + path.size() + " nodes" : " UNREACHABLE")
                + ", " + (computeNanos / 1000) + "us)";
    }
}