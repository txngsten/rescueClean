package solution.Pathfinding.Algorithms;

import solution.Graph;

import java.util.List;

/**
 * An incremental, replanning pathfinder (e.g. D* Lite) that maintains search
 * state across edge-cost changes so that successive queries to the same fixed
 * goal can reuse prior computation.
 *
 * <p>The contract is: {@link #initialize(Graph, String)} once; then any mix of
 * {@link #updateEdge(String, String)} / {@link #nodeCollapsed(String)} as the
 * world changes, followed by {@link #replan(String)} to obtain an up-to-date
 * path from a (possibly moved) start. All calls must occur on a single thread
 * because the implementation holds mutable, non-thread-safe search state.
 */
public interface IncrementalPathfinder extends Pathfinder {

    /** Prepares the search for a fixed {@code goal} over {@code graph}. */
    void initialize(Graph graph, String goal);

    /**
     * Notifies the search that the cost of the single arc {@code from -> to} has
     * changed (typically blocked or cleared). Only the vertex whose lookahead
     * depends on that arc is updated.
     */
    void updateEdge(String from, String to);

    /**
     * Notifies the search that {@code node} has collapsed: every arc incident to
     * it (in and out) is now permanently unusable. Unlike a single-edge change,
     * a collapse changes the lookahead of <em>every predecessor</em> of
     * {@code node} (their cheapest successor may have just become unreachable),
     * so all of those vertices must be re-evaluated, not just {@code node}
     * itself. Implementations should update {@code node} and each of its road
     * predecessors.
     */
    void nodeCollapsed(String node);

    /**
     * Recomputes (incrementally) and returns a least-cost path from
     * {@code start} to the fixed goal under the current costs, or {@code null}
     * if the goal is currently unreachable.
     */
    List<String> replan(String start);

    /** @return the fixed goal this instance was initialised with. */
    String goal();
}