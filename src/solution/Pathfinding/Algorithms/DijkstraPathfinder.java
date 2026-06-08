package solution.Pathfinding.Algorithms;

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

        HashMap<String, Double> dist = new HashMap<>();
        HashMap<String, String> prev = new HashMap<>();

        PriorityQueue<Pair<String, Double>> open =
                new PriorityQueue<>(Comparator.comparing(Pair::second));

        dist.put(start, 0.0);
        open.add(new Pair<>(start, 0.0));

        while (!open.isEmpty()) {
            Pair<String, Double> top = open.poll();
            String u = top.first();
            double d = top.second();

            Double best = dist.get(u);
            if (best == null || d > best) {
                continue;
            }

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

        return null;
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
        return "Dijkstra";
    }
}