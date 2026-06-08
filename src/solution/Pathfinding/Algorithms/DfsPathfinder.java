package solution.Pathfinding.Algorithms;

import solution.Graph;
import solution.Edge;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Depth-first search returning <em>some</em> path from {@code start} to
 * {@code goal} (not necessarily short by hops or by cost).
 *
 * <p>DFS makes no optimality guarantee whatsoever: it commits to one branch and
 * follows it as deep as possible before backtracking, so the path it returns
 * can be wildly longer than necessary. Its only virtues are speed-to-first-path
 * on well-connected graphs and a small, predictable memory footprint (one
 * explicit stack rather than a cost-ordered heap). It is included as the
 * lower-quality extreme of the comparison: useful to show how much path quality
 * the weighted searches actually buy.
 *
 * <p>Implemented iteratively with an explicit {@link Deque} rather than
 * recursively, so that deep graphs (the 100k-node map) cannot overflow the call
 * stack. Uses {@link Graph#getEdges(String)} so blocked roads and collapsed
 * nodes are never traversed.
 *
 * @author solution
 */
public final class DfsPathfinder implements Pathfinder {

    @Override
    public List<String> findPath(Graph graph, String start, String goal) {
        if (start == null || goal == null) {
            return null;
        }
        if (start.equals(goal)) {
            List<String> trivial = new ArrayList<>(1);
            trivial.add(start);
            return trivial;
        }

        Deque<String> stack = new ArrayDeque<>();
        Set<String> visited = new HashSet<>();
        HashMap<String, String> prev = new HashMap<>();

        stack.push(start);
        visited.add(start);

        while (!stack.isEmpty()) {
            String u = stack.pop();

            if (u.equals(goal)) {
                return reconstruct(prev, start, goal);
            }

            for (Edge e : graph.getEdges(u)) {
                String v = e.to();
                if (visited.add(v)) {
                    prev.put(v, u);
                    if (v.equals(goal)) {
                        return reconstruct(prev, start, goal);
                    }
                    stack.push(v);
                }
            }
        }

        return null; // goal unreachable under current graph state
    }

    private List<String> reconstruct(HashMap<String, String> prev, String start, String goal) {
        ArrayList<String> path = new ArrayList<>();
        for (String cur = goal; cur != null; cur = prev.get(cur)) {
            path.add(cur);
            if (cur.equals(start)) {
                break;
            }
        }
        Collections.reverse(path);
        return path;
    }

    @Override
    public String name() {
        return "DFS";
    }
}
