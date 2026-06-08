package solution.Pathfinding.Algorithms;

import solution.Graph;

import java.util.List;

/**
 * Extension of Pathfinder for incremental, replanning algorithms (D* Lite)
 * that maintain search state across edge-cost changes. The contract is:
 * initialize() once, then any mix of updateEdge()/nodeCollapsed() as the world
 * changes, followed by replan() to get an up-to-date path. All calls must be on
 * a single thread since the implementation holds mutable, non-thread-safe state.
 */
public interface IncrementalPathfinder extends Pathfinder {

    void initialize(Graph graph, String goal);

    // Only the vertex whose lookahead depends on the changed arc is updated.
    void updateEdge(String from, String to);

    // Unlike a single-edge change, a collapse invalidates every arc incident to
    // the node, so all predecessors must be re-evaluated, not just the node itself.
    void nodeCollapsed(String node);

    List<String> replan(String start);

    String goal();
}