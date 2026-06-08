package solution.Pathfinding;

import java.util.List;

/**
 * A unit of work submitted to the {@link RoutingEngine}.
 *
 * <p>Tasks fall into two priority tiers and several kinds. The tier decides
 * which internal queue the task waits in (return work is drained before
 * outbound work); the kind tells the worker what to do with it.
 *
 * <ul>
 *   <li>{@link Kind#OUTBOUND} - compute a base-&gt;building path (low priority).
 *       Always handled by a stateless one-shot pathfinder on the parallel pool.</li>
 *   <li>{@link Kind#RETURN} - compute a building-&gt;base path (high priority).
 *       Handled either on the parallel pool (one-shot return algorithms) or by
 *       the single dedicated D* Lite worker, depending on the engine's mode.</li>
 *   <li>{@link Kind#EDGE_CHANGE} - a single-arc road status change to feed into
 *       the shared D* Lite instance via {@code updateEdge}. Only used in D* Lite
 *       mode, and only ever runs on the dedicated worker so the instance stays
 *       confined to one thread. Carries the changed arc in {@link #from}/{@link #to}.</li>
 *   <li>{@link Kind#COLLAPSE} - a node-collapse notification to feed into the
 *       shared D* Lite instance via {@code nodeCollapsed}. Distinct from
 *       EDGE_CHANGE because a collapse invalidates every arc incident to the
 *       node, so all of the node's predecessors must be re-evaluated, not just
 *       one arc's tail. Carries the collapsed node in {@link #from}.</li>
 * </ul>
 *
 * <p>Tasks are immutable value carriers; the computed result is delivered
 * separately via the engine's result queue (see {@link PathResult}), never by
 * mutating the task.
 *
 * @author solution
 */
public final class PathTask {

    /** What the worker should do with this task. */
    public enum Kind {
        /** Compute a path from base to a building (low priority). */
        OUTBOUND,
        /** Compute a path from a vehicle's location back to base (high priority). */
        RETURN,
        /** Apply a single-arc road-status change to the shared D* Lite instance. */
        EDGE_CHANGE,
        /** Apply a node collapse to the shared D* Lite instance. */
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

    /**
     * Creates an outbound (base-&gt;building) compute task.
     *
     * @param vehicle the vehicle to dispatch
     * @param start   the vehicle's current location (path start; the base)
     * @param goal    the building to reach
     * @return a new OUTBOUND task
     */
    public static PathTask outbound(int vehicle, String start, String goal) {
        return new PathTask(Kind.OUTBOUND, vehicle, start, goal, null, null);
    }

    /**
     * Creates a return (building-&gt;base) compute task.
     *
     * @param vehicle the vehicle to bring home
     * @param start   the vehicle's current location (path start)
     * @param goal    the base
     * @return a new RETURN task
     */
    public static PathTask returnToBase(int vehicle, String start, String goal) {
        return new PathTask(Kind.RETURN, vehicle, start, goal, null, null);
    }

    /**
     * Creates an edge-change task to feed a single-arc road-status update into
     * the shared D* Lite instance. Used only in D* Lite return mode.
     *
     * @param from the changed arc's source endpoint
     * @param to   the changed arc's target endpoint
     * @return a new EDGE_CHANGE task
     */
    public static PathTask edgeChange(String from, String to) {
        return new PathTask(Kind.EDGE_CHANGE, -1, null, null, from, to);
    }

    /**
     * Creates a collapse task to feed a node collapse into the shared D* Lite
     * instance. Used only in D* Lite return mode.
     *
     * @param node the collapsed node
     * @return a new COLLAPSE task
     */
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