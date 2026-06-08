package solution.Pathfinding;

import solution.Edge;
import solution.Graph;
import util.Pair;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

/**
 * One-shot Dijkstra shortest-path search.
 *
 * <p>This is the optimal baseline: it always returns a minimum-cost path when
 * one exists, against which the faster-but-suboptimal strategies (BFS by hop
 * count, the greedy descent of D* Lite under inflation, etc.) can be compared.
 * It explores in nondecreasing order of cost from {@code start} and terminates
 * as soon as {@code goal} is settled, so on a single point-to-point query it
 * does strictly less work than the all-destinations precompute in
 * {@code MyDisasterResponder}.
 *
 * <p>Reads go through {@link Graph#getEdges(String)}, which already excludes
 * blocked edges and collapsed nodes, so the path returned only uses currently
 * traversable roads. Because the graph is read under its internal read lock per
 * accessor call, a search reflects a consistent-enough snapshot for dispatch;
 * any change arriving mid-search is caught by the Simulator's per-segment
 * validation and handled as a replan.
 *
 * @author solution
 */
public final class DijkstraPathfinder implements Pathfinder {

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

        // Best known cost to each settled/seen node, and the predecessor that
        // achieved it, for path reconstruction.
        HashMap<String, Double> dist = new HashMap<>();
        HashMap<String, String> prev = new HashMap<>();

        // Min-heap keyed on tentative cost. We use the lazy-deletion variant:
        // stale entries are skipped on poll rather than decrease-key'd.
        PriorityQueue<Pair<String, Double>> open =
                new PriorityQueue<>(Comparator.comparing(Pair::second));

        dist.put(start, 0.0);
        open.add(new Pair<>(start, 0.0));

        while (!open.isEmpty()) {
            Pair<String, Double> top = open.poll();
            String u = top.first();
            double d = top.second();

            // Skip stale heap entries (a better cost was found after this was
            // enqueued).
            Double best = dist.get(u);
            if (best == null || d > best) {
                continue;
            }

            // Goal settled: reconstruct and return immediately.
            if (u.equals(goal)) {
                return reconstruct(prev, start, goal);
            }

            for (Edge e : graph.getEdges(u)) {
                String v = e.to();
                double nd = d + e.weight();
                Double known = dist.get(v);
                if (known == null || nd < known) {
                    dist.put(v, nd);
                    prev.put(v, u);
                    open.add(new Pair<>(v, nd));
                }
            }
        }

        // Goal never settled: unreachable under the current graph state.
        return null;
    }

    /**
     * Walks predecessor links from {@code goal} back to {@code start} and
     * reverses to produce a forward path. Assumes a path exists (caller settled
     * the goal before calling).
     */
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
        return "Dijkstra";
    }
}