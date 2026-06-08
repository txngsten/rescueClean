package solution.Pathfinding.Algorithms;

import solution.Edge;
import solution.Graph;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Queue;
import java.util.Set;

/**
 * Breadth-first search returning a path with the <em>fewest road segments</em>
 * (hops) from {@code start} to {@code goal}.
 *
 * <p><b>Important:</b> BFS treats every edge as unit cost and therefore ignores
 * the {@code weight} field entirely. On the artificial maps (edge weights drawn
 * around a mean of 100 with nonzero variance) the fewest-hops path is generally
 * <em>not</em> the least-distance path, so the route it returns can cost
 * meaningfully more travel time than Dijkstra's. It is included precisely so
 * that this trade-off can be measured: BFS explores far less structure than a
 * cost-ordered search and doubles as a fast "is the goal reachable at all?"
 * test, which is useful for pruning hopeless rescues before paying for a
 * weighted search.
 *
 * <p>Uses {@link Graph#getEdges(String)} so blocked roads and collapsed nodes
 * are never traversed.
 *
 * @author solution
 */
public final class BfsPathfinder implements Pathfinder {

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

        Queue<String> frontier = new ArrayDeque<>();
        Set<String> visited = new HashSet<>();
        HashMap<String, String> prev = new HashMap<>();

        frontier.add(start);
        visited.add(start);

        while (!frontier.isEmpty()) {
            String u = frontier.poll();

            for (Edge e : graph.getEdges(u)) {
                String v = e.to();
                if (visited.add(v)) {            // add() returns false if already present
                    prev.put(v, u);
                    if (v.equals(goal)) {
                        return reconstruct(prev, start, goal);
                    }
                    frontier.add(v);
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
        return "BFS";
    }
}