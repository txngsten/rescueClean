package solution.Pathfinding.Algorithms;

import solution.Graph;
import solution.Edge;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

/**
 * D* Lite incremental shortest-path search (optimized, {@code k_m} variant)
 * after Koenig &amp; Likhachev, <i>D* Lite</i> (AAAI 2002), Figure 4.
 *
 * <p>The search runs <b>backward from a fixed goal</b>, so {@code g(s)} is an
 * estimate of the cost to travel from {@code s} to the goal. In this system the
 * goal is the fleet base and a single instance is shared across every returning
 * vehicle: one repair after a road change benefits all of them, and a vehicle
 * moving toward base is handled as a moving start without reordering the queue
 * (that is what the {@code k_m} key offset buys).
 *
 * <h2>Mapping to this graph</h2>
 * <ul>
 *   <li><b>Predecessors.</b> Backward search expands the <em>successors in the
 *       search graph</em>, which are the <em>predecessors in the road graph</em>
 *       (edges <em>into</em> a vertex). {@link Graph} stores out-edges only, so
 *       a reverse adjacency is built once at {@link #initialize} from
 *        + {@link Graph#allEdges(String)} in one O(E)
 *       pass.</li>
 *   <li><b>Live costs.</b> The reverse structure is static, but each arc cost
 *       {@code c(u,v)} is read live via
 *       -- a blocked or collapsed arc reads as {@code +infinity}. This is why
 *       D* Lite needs the unfiltered {@code allEdges}: it must see the arc in
 *       order to notice its cost became infinite.</li>
 *   <li><b>Heuristic.</b> The GraphML nodes carry no coordinates
 *       ({@code <node id="..."/>} only), so the only heuristic we can prove
 *       admissible is {@code h == 0}. With a zero heuristic D* Lite is still
 *       correct and still reuses computation incrementally; it simply is not
 *       heuristically focused and behaves like an incremental uniform-cost
 *       search. The heuristic is isolated in {@link #h(String, String)} so a
 *       coordinate-based admissible heuristic can be substituted later if node
 *       positions ever become available, with no other change.</li>
 * </ul>
 *
 * <h2>Concurrency</h2>
 * This class holds mutable search state and is <b>not</b> safe for concurrent
 * use. All calls on one instance must be confined to a single thread; the
 * routing layer dedicates one worker to the shared base-rooted instance.
 *
 * @author solution
 */
public final class DStarLite implements IncrementalPathfinder {

    /** Positive infinity sentinel for unreachable / blocked costs. */
    private static final double INF = Double.POSITIVE_INFINITY;

    /**
     * A priority-queue entry pairing a vertex with the key it was inserted
     * under. D* Lite uses lazy deletion: an entry is ignored on pop if the
     * vertex is no longer in {@link #queued} with this exact key.
     */
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

    /** Lexicographic ordering on keys: by k1, then k2. */
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
    private String last;        // s_last: start when k_m was last accumulated
    private double km;          // key modifier accumulating heuristic drift

    private Map<String, Double> g;
    private Map<String, Double> rhs;

    // Reverse adjacency: predecessors.get(v) lists (u, originalWeight) for every
    // road edge u -> v. Costs are re-read live, so the stored weight is only the
    // map's nominal weight; usability is checked separately.
    private Map<String, List<String>> predecessors;   // v -> list of u with edge u->v
    private Map<String, List<String>> successors;     // u -> list of v with edge u->v (real out-edges)

    private PriorityQueue<Entry> open;
    // Tracks the key each queued vertex currently holds, for lazy-deletion
    // validity checks and for membership tests.
    private Map<String, double[]> queued;

    // ---------------------------------------------------------------- lifecycle

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

        // Initialize: rhs(goal) = 0, everything else (g, rhs) defaults to INF
        // lazily via getG / getRhs. Insert the goal.
        rhs.put(goal, 0.0);
        double[] k = calculateKey(goal);
        insert(goal, k[0], k[1]);
    }

    /**
     * Builds predecessor and successor adjacency once from the current graph
     * structure. Runs in O(V + E). Called from {@link #initialize}; the
     * structure is assumed static thereafter (roads are blocked/cleared, not
     * created or destroyed structurally), while costs are read live.
     */
    private void buildReverseAdjacency() {
        for (String u : graph.nodes()) {
            successors.computeIfAbsent(u, k -> new ArrayList<>());
            for (Edge e : graph.allEdges(u)) {
                String v = e.to();
                successors.get(u).add(v);
                predecessors.computeIfAbsent(v, k -> new ArrayList<>()).add(u);
                // Ensure v has a successor entry too (may have no out-edges).
                successors.computeIfAbsent(v, k -> new ArrayList<>());
            }
        }
    }

    // ---------------------------------------------------------------- core math

    private double getG(String s) {
        Double v = g.get(s);
        return v == null ? INF : v;
    }

    private double getRhs(String s) {
        Double v = rhs.get(s);
        return v == null ? INF : v;
    }

    /**
     * Admissible heuristic estimate of cost between {@code a} and {@code b}.
     * Returns 0 -- see the class comment: no node coordinates exist, so zero is
     * the only provably admissible choice. Isolated here for future
     * substitution.
     */
    private double h(String a, String b) {
        return 0.0;
    }

    /**
     * key(s) = [ min(g,rhs) + h(start, s) + km ,  min(g,rhs) ].
     */
    private double[] calculateKey(String s) {
        double min = Math.min(getG(s), getRhs(s));
        double k1 = (min == INF) ? INF : min + h(start, s) + km;
        double k2 = min;
        return new double[] { k1, k2 };
    }

    /** Cost of road arc u -> v: live weight if usable, else +infinity. */
    private double cost(String u, String v) {
        if (!graph.isEdgeUsable(u, v)) {
            return INF;
        }
        double w = graph.getWeight(u, v);
        // getWeight returns -1 if the arc is absent; usable already excludes
        // that, but guard defensively.
        return (w < 0) ? INF : w;
    }

    private void insert(String s, double k1, double k2) {
        open.add(new Entry(s, k1, k2));
        queued.put(s, new double[] { k1, k2 });
    }

    private void remove(String s) {
        queued.remove(s);
        // Entry is left in the heap and skipped lazily on pop.
    }

    private boolean isQueued(String s) {
        return queued.containsKey(s);
    }

    /**
     * UpdateVertex(u): recompute rhs(u) for a non-goal vertex as the best
     * one-step lookahead over its successors in the search graph (its real
     * out-edges u -> v: cost(u,v) + g(v)), then fix its queue membership.
     */
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

    /**
     * ComputeShortestPath(): expand locally inconsistent vertices in key order
     * until the start vertex is consistent and no queued key precedes it.
     */
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

            // If its stored key is stale relative to a freshly computed key,
            // reinsert with the corrected key (Figure 4, lines 12-14).
            double[] kNew = calculateKey(u);
            if (compareKeys(top.k1, top.k2, kNew[0], kNew[1]) < 0) {
                insert(u, kNew[0], kNew[1]);
                continue;
            }

            double gu = getG(u);
            double rhsu = getRhs(u);

            if (gu > rhsu) {
                // Overconsistent: tighten g down to rhs, propagate to preds.
                g.put(u, rhsu);
                remove(u);
                List<String> preds = predecessors.get(u);
                if (preds != null) {
                    for (String p : preds) {
                        updateVertex(p);
                    }
                }
            } else {
                // Underconsistent: reset g to INF, propagate to preds and self.
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

    /**
     * Returns the valid top entry (one whose stored key still matches
     * {@link #queued}), discarding stale lazy-deletion leftovers. Returns
     * {@code null} if the queue holds no valid entries.
     */
    private Entry peekValid() {
        while (!open.isEmpty()) {
            Entry top = open.peek();
            double[] cur = queued.get(top.s);
            if (cur != null && cur[0] == top.k1 && cur[1] == top.k2) {
                return top;
            }
            open.poll(); // stale entry, discard
        }
        return null;
    }

    // ---------------------------------------------------------------- public API

    @Override
    public void updateEdge(String from, String to) {
        // A change to arc (from -> to) affects the rhs of 'from' (its successor
        // 'to' may now be cheaper/costlier or unreachable). Recompute the
        // affected endpoint. Because search is backward, the vertex whose
        // lookahead depends on arc (from->to) is 'from'.
        if (predecessors.isEmpty() && successors.isEmpty()) {
            return; // not initialized
        }
        updateVertex(from);
    }

    @Override
    public List<String> replan(String start) {
        if (start == null) {
            return null;
        }
        // Accumulate km by the heuristic distance the start moved, then update
        // s_last (Figure 4, lines 38-39). With h == 0 this is a no-op on km,
        // but kept structurally correct for a future heuristic.
        if (!start.equals(this.start)) {
            this.km += h(this.last, start);
            this.last = start;
            this.start = start;
        }

        computeShortestPath();

        if (getG(start) == INF) {
            return null; // goal unreachable from start under current costs
        }
        return greedyPath(start);
    }

    /**
     * Extracts the path by greedy descent on g: from each vertex move to the
     * successor minimising cost(s, s') + g(s'), until the goal is reached.
     * Returns {@code null} if descent stalls (should not happen when
     * g(start) is finite, but guarded against cycles).
     */
    private List<String> greedyPath(String from) {
        ArrayList<String> path = new ArrayList<>();
        java.util.HashSet<String> guard = new java.util.HashSet<>();
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
                return null; // revisited a node: no progress, bail
            }
            path.add(bestNext);
            s = bestNext;
        }
        return path;
    }

    @Override
    public List<String> findPath(Graph graph, String start, String goal) {
        // One-shot use: (re)initialise to this goal if needed, then replan.
        // Lets D* Lite stand in wherever a plain Pathfinder is expected.
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