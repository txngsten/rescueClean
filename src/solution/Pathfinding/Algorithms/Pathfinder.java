package solution.Pathfinding.Algorithms;

import solution.Graph;
import java.util.List;

/**
 * Common interface for all one-shot pathfinding algorithms. Each implementation
 * computes a path from start to goal on the current graph state without retaining
 * search state between calls.
 */
public interface Pathfinder {
    List<String> findPath(Graph roadMap, String start, String goal);
    String name();
}
