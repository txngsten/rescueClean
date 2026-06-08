package solution.Pathfinding.Algorithms;

import solution.Graph;
import solution.Edge;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * D* Lite incremental shortest-path search (optimized k_m variant) after Koenig
 * & Likhachev (AAAI 2002). Searches backward from a fixed goal (the
 * fleet base), so g(s) estimates cost from s to goal. A single instance is
 * shared across all returning vehicles: one repair after a road change benefits
 * all of them.
 * Not thread-safe; the routing layer confines all calls to one dedicated worker.
 */
public final class DStarLite implements IncrementalPathfinder {

    private static final double INF = Double.POSITIVE_INFINITY;

    private static final class Entry {
        final String s;
        final double k1;
        final double k2;

        Entry(String s, double k1, double k2) {
            this.s = s;
            this.k1 = k1;
            this.k2 = k2;
        }
    }

    private static int compareKeys(double a1, double a2, double b1, double b2) {
        if (a1 < b1) return -1;
        if (a1 > b1) return 1;
        if (a2 < b2) return -1;
        if (a2 > b2) return 1;
        return 0;
    }

    private Graph graph;
    private String goal;
    private String start;
    private String last;
    private double km;

    private Map<String, Double> g;
    private Map<String, Double> rhs;

    // Static reverse adjacency; costs are read live against the graph.
    private Map<String, List<String>> predecessors;
    private Map<String, List<String>> successors;

    private PriorityQueue<Entry> open;
    // Current key per queued vertex; used for lazy-deletion validity checks.
    private Map<String, double[]> queued;


    @Override
    public void initialize(Graph graph, String goal) {
        this.graph = graph;
        this.goal = goal;
        this.start = goal;       // will be reset on first replan
        this.last = goal;
        this.km = 0.0;

        this.g = new HashMap<>();
        this.rhs = new HashMap<>();
        this.predecessors = new HashMap<>();
        this.successors = new HashMap<>();
        this.open = new PriorityQueue<>((x, y) -> compareKeys(x.k1, x.k2, y.k1, y.k2));
        this.queued = new HashMap<>();

        buildReverseAdjacency();

        rhs.put(goal, 0.0);
        double[] k = calculateKey(goal);
        insert(goal, k[0], k[1]);
    }

    private void buildReverseAdjacency() {
        for (String u : graph.nodes()) {
            successors.computeIfAbsent(u, k -> new ArrayList<>());
            for (Edge e : graph.allEdges(u)) {
                String v = e.to();
                successors.get(u).add(v);
                predecessors.computeIfAbsent(v, k -> new ArrayList<>()).add(u);
                successors.computeIfAbsent(v, k -> new ArrayList<>());
            }
        }
    }



    private double getG(String s) {
        Double v = g.get(s);
        return v == null ? INF : v;
    }

    private double getRhs(String s) {
        Double v = rhs.get(s);
        return v == null ? INF : v;
    }

    // Zero heuristic: no coordinates available, so 0 is the only admissible choice.
    private double h(String a, String b) {
        return 0.0;
    }

    private double[] calculateKey(String s) {
        double min = Math.min(getG(s), getRhs(s));
        double k1 = (min == INF) ? INF : min + h(start, s) + km;
        double k2 = min;
        return new double[] { k1, k2 };
    }

    private double cost(String u, String v) {
        if (!graph.isEdgeUsable(u, v)) {
            return INF;
        }
        double w = graph.getWeight(u, v);
        return (w < 0) ? INF : w;
    }

    private void insert(String s, double k1, double k2) {
        open.add(new Entry(s, k1, k2));
        queued.put(s, new double[] { k1, k2 });
    }

    private void remove(String s) {
        queued.remove(s);
    }

    private boolean isQueued(String s) {
        return queued.containsKey(s);
    }

    private void updateVertex(String u) {
        if (!u.equals(goal)) {
            double best = INF;
            List<String> succs = successors.get(u);
            if (succs != null) {
                for (String v : succs) {
                    double c = cost(u, v);
                    if (c == INF) {
                        continue;
                    }
                    double val = c + getG(v);
                    if (val < best) {
                        best = val;
                    }
                }
            }
            rhs.put(u, best);
        }

        if (isQueued(u)) {
            remove(u);
        }
        if (getG(u) != getRhs(u)) {
            double[] k = calculateKey(u);
            insert(u, k[0], k[1]);
        }
    }

    private void computeShortestPath() {
        while (!open.isEmpty()) {
            Entry top = peekValid();
            if (top == null) {
                break;
            }

            double[] startKey = calculateKey(start);
            boolean topPrecedesStart =
                    compareKeys(top.k1, top.k2, startKey[0], startKey[1]) < 0;
            boolean startInconsistent = getRhs(start) != getG(start);

            if (!topPrecedesStart && !startInconsistent) {
                break;
            }

            // Pop the validated top.
            open.poll();
            String u = top.s;

            double[] kNew = calculateKey(u);
            if (compareKeys(top.k1, top.k2, kNew[0], kNew[1]) < 0) {
                insert(u, kNew[0], kNew[1]);
                continue;
            }

            double gu = getG(u);
            double rhsu = getRhs(u);

            if (gu > rhsu) {
                g.put(u, rhsu);
                remove(u);
                List<String> preds = predecessors.get(u);
                if (preds != null) {
                    for (String p : preds) {
                        updateVertex(p);
                    }
                }
            } else {
                g.put(u, INF);
                List<String> preds = predecessors.get(u);
                if (preds != null) {
                    for (String p : preds) {
                        updateVertex(p);
                    }
                }
                updateVertex(u);
            }
        }
    }

    private Entry peekValid() {
        while (!open.isEmpty()) {
            Entry top = open.peek();
            double[] cur = queued.get(top.s);
            if (cur != null && cur[0] == top.k1 && cur[1] == top.k2) {
                return top;
            }
            open.poll();
        }
        return null;
    }

    // ---------------------------------------------------------------- public API

    @Override
    public void updateEdge(String from, String to) {
        if (predecessors.isEmpty() && successors.isEmpty()) {
            return;
        }
        updateVertex(from);
    }

    @Override
    public void nodeCollapsed(String node) {
        if (predecessors.isEmpty() && successors.isEmpty()) {
            return;
        }
        List<String> preds = predecessors.get(node);
        if (preds != null) {
            for (String p : preds) {
                updateVertex(p);
            }
        }
        updateVertex(node);
    }

    @Override
    public List<String> replan(String start) {
        if (start == null) {
            return null;
        }
        if (!start.equals(this.start)) {
            this.km += h(this.last, start);
            this.last = start;
            this.start = start;
        }

        computeShortestPath();

        if (getG(start) == INF) {
            return null;
        }
        return greedyPath(start);
    }

    private List<String> greedyPath(String from) {
        ArrayList<String> path = new ArrayList<>();
        Set<String> guard = new HashSet<>();
        String s = from;
        path.add(s);
        guard.add(s);

        while (!s.equals(goal)) {
            List<String> succs = successors.get(s);
            if (succs == null || succs.isEmpty()) {
                return null;
            }
            String bestNext = null;
            double best = INF;
            for (String v : succs) {
                double c = cost(s, v);
                if (c == INF) {
                    continue;
                }
                double val = c + getG(v);
                if (val < best) {
                    best = val;
                    bestNext = v;
                }
            }
            if (bestNext == null || best == INF) {
                return null;
            }
            if (!guard.add(bestNext)) {
                return null;
            }
            path.add(bestNext);
            s = bestNext;
        }
        return path;
    }

    // One-shot fallback, reinit if needed, then replan.
    @Override
    public List<String> findPath(Graph graph, String start, String goal) {
        if (this.graph != graph || this.goal == null || !this.goal.equals(goal)) {
            initialize(graph, goal);
        }
        return replan(start);
    }

    @Override
    public String goal() {
        return goal;
    }

    @Override
    public String name() {
        return "D* Lite";
    }
}