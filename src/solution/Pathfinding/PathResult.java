package solution.Pathfinding;

import java.util.List;

/**
 * The outcome of a {@link PathTask} computation, delivered to the responder via
 * the {@link RoutingEngine}'s result queue.
 *
 * <p>Decoupling the result from the task (rather than mutating the task or
 * calling back into the responder from a worker) keeps the potentially-blocking
 * {@code outMessageQueue.put} off the worker threads: workers only enqueue a
 * {@code PathResult}, and a single dedicated sender drains results and emits the
 * corresponding PATH / HALT messages. This matters because the Simulator's
 * incoming queue is bounded (capacity 100), so a send can block.
 *
 * <p>A {@code null} {@link #path()} means no route currently exists for the
 * request (the goal is unreachable under the present graph state); the
 * responder decides how to react (defer, retry, or abandon the mission).
 *
 * @author solution
 */
public final class PathResult {

    private final PathTask.Kind kind;
    private final int vehicle;
    private final String goal;
    private final List<String> path;   // null if unreachable
    private final long computeNanos;   // wall-clock spent computing, for metrics

    /**
     * @param kind         the kind of task this result is for (OUTBOUND/RETURN)
     * @param vehicle      the vehicle the result concerns
     * @param goal         the goal the path was computed toward
     * @param path         the computed path (start..goal inclusive), or
     *                     {@code null} if no path exists
     * @param computeNanos nanoseconds spent computing the path, for the
     *                     performance comparison required by the assessment
     */
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

    /** @return true if a usable path was found. */
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