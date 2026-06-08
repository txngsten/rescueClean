package solution;

import org.jdom2.JDOMException;
import sim.Message;
import util.ConfigurationInfo;
import util.Pair;
import solution.Pathfinding.Algorithms.Pathfinder;
import solution.Pathfinding.Algorithms.DijkstraPathfinder;
import solution.Pathfinding.Algorithms.BfsPathfinder;
import solution.Pathfinding.Algorithms.DfsPathfinder;
import solution.Pathfinding.Algorithms.DStarLite;
import solution.Pathfinding.RoutingEngine;
import solution.Pathfinding.PathTask;
import solution.Pathfinding.PathResult;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Concrete {@link DisasterResponder} that dispatches a fleet of vehicles to
 * rescue trapped evacuees on a dynamic road network.
 *
 * <h2>Architecture</h2>
 * The comms thread (via {@link #handle}) does no heavy work: it parses each
 * incoming {@link Message}, updates the {@link VehicleManager}, and submits any
 * path computation to the {@link RoutingEngine}. Workers in the engine compute
 * paths and publish {@link PathResult}s, which a dedicated <em>sender thread</em>
 * drains and turns into PATH / HALT messages on the (capacity-bounded) outgoing
 * queue. Keeping the potentially-blocking {@code put} on its own thread means
 * neither path computation nor message receipt can stall on a full queue.
 *
 * <h2>Outbound precompute</h2>
 * During {@link #setup} a single-source Dijkstra from the base is computed and
 * cached (base-&gt;building distances and predecessors). Outbound dispatches use
 * this cache directly while it is valid - it is the fastest possible outbound
 * route source and costs O(V) memory. The cache is invalidated only when a
 * block or collapse actually lands on the predecessor tree the cache encodes
 * (see {@link #invalidatePrecomputeIfAffected}); an unrelated block elsewhere on
 * the map leaves it valid. Returns and reroutes <em>never</em> use the
 * precompute - they always go through the configured algorithm, which keeps them
 * correct on the one-way Manhattan map and makes the per-algorithm performance
 * comparison clean.
 *
 * <h2>Collapse policy</h2>
 * When a location collapses we proactively halt only those moving vehicles whose
 * <em>imminent</em> next step (the very next unconfirmed segment) leads into the
 * collapsed node. A collapse many hops ahead on a vehicle's route is left to the
 * Simulator's per-segment validation, which will halt the vehicle (via
 * WAYPOINT_INVALID) exactly when the bad segment becomes current - at which
 * point we reroute from the vehicle's real position with fresh information.
 * Halting the whole affected sub-fleet on every collapse produced a reroute
 * storm that serialised behind the single D* Lite worker; narrowing to the
 * imminent step removes that storm without sacrificing any vehicle (a vehicle is
 * only destroyed when it actually enters a collapsed node).
 *
 * <h2>Algorithm selection</h2>
 * The {@link #OUTBOUND_ALGO} and {@link #RETURN_ALGO} constants choose the
 * algorithms, read via the overridable {@link #outboundAlgo()} /
 * {@link #returnAlgo()} accessors. The {@link RoutingEngine}'s threading model
 * follows from the return algorithm: a D* Lite return runs on a single dedicated
 * worker (D* Lite is stateful), while one-shot returns run on the parallel pool.
 * A subclass selects a different {@code DisasterResponder} behaviour simply by
 * overriding the two accessors (see {@code DijkstraResponder}, {@code BfsResponder},
 * {@code HybridResponder}, {@code DStarResponder}); the constants here are the
 * defaults used when nothing is overridden.
 *
 * @author solution
 */
public class MyDisasterResponder extends DisasterResponder {

    // ============================================================ configuration

    /** The available routing algorithms. */
    public enum Algo { DIJKSTRA, BFS, DFS, DSTAR }

    /** Algorithm used for base-&gt;building paths when the precompute is invalid. */
    protected static final Algo OUTBOUND_ALGO = Algo.DIJKSTRA;

    /** Algorithm used for building-&gt;base paths and all rerouting. */
    protected static final Algo RETURN_ALGO = Algo.DSTAR;

    /**
     * Whether to proactively halt and reroute a vehicle when its imminent next
     * segment leads into a node that has collapsed (saving the vehicle if a
     * route around exists). When false, such a vehicle is left to drive into the
     * collapse and be lost.
     */
    protected static final boolean PROACTIVE_COLLAPSE_REROUTE = true;

    /** Size of the parallel worker pool (outbound, and one-shot returns). */
    private static final int POOL_SIZE =
            Math.max(1, Runtime.getRuntime().availableProcessors() - 1);

    // ============================================================ state

    private volatile Graph roadMap;
    private volatile String origin;

    // --- outbound precompute (base -> building) ---
    private final ConcurrentHashMap<String, Double> distFromBase = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, String> parent = new ConcurrentHashMap<>();
    private final AtomicBoolean precomputeDone = new AtomicBoolean(false);
    /** False once a block/collapse on the cached predecessor tree invalidates it. */
    private final AtomicBoolean precomputeValid = new AtomicBoolean(false);

    private VehicleManager fleet;
    private RoutingEngine engine;
    private Thread senderThread;
    private final AtomicBoolean senderRunning = new AtomicBoolean(false);

    // Pending rescue requests we could not immediately serve (no free vehicle,
    // or temporarily unreachable). Kept FIFO; retried when a vehicle frees up.
    private final BlockingQueue<String> pendingRescues = new LinkedBlockingQueue<>();

    // Maps a vehicle to the rescue goal it is currently assigned (so when it
    // reaches the building we know to switch it to a return mission).
    private final ConcurrentHashMap<Integer, String> assignedRescue = new ConcurrentHashMap<>();

    // Vehicles we have proactively halted and intend to reroute once the HALTED
    // confirmation arrives; maps vehicle -> intended new goal (base for returns).
    private final ConcurrentHashMap<Integer, String> rerouteOnHalt = new ConcurrentHashMap<>();

    // ============================================================ setup

    @Override
    protected void setup() {
        String mapFile = ConfigurationInfo.getMapFile(configFile);
        origin = ConfigurationInfo.getOrigin(configFile);

        try {
            roadMap = GraphBuilder.buildFromGraphML(mapFile);
        } catch (JDOMException | IOException e) {
            throw new RuntimeException("Failed to load road map: " + mapFile, e);
        }

        fleet = new VehicleManager(origin, ConfigurationInfo.NUMBER_OF_VEHICLES);

        // Build the routing engine in the mode dictated by the return algorithm.
        Pathfinder outbound = makePathfinder(outboundAlgo());
        Pathfinder ret = makePathfinder(returnAlgo());
        engine = new RoutingEngine(roadMap, origin, outbound, ret, POOL_SIZE);
        engine.start();

        startSender();

        // Kick off the outbound precompute on the inherited single-thread executor.
        final Graph g = roadMap;
        final String src = origin;
        executor.submit(() -> dijkstraFromBase(g, src));
    }

    // Algorithm selection goes through these so subclasses can override the
    // choice (static final constants can't be overridden). Defaults return the
    // constants above, so MyDisasterResponder's own behaviour is unchanged.
    protected Algo outboundAlgo() {
        return OUTBOUND_ALGO;
    }

    protected Algo returnAlgo() {
        return RETURN_ALGO;
    }

    /** Instantiates a fresh pathfinder for the given algorithm choice. */
    private Pathfinder makePathfinder(Algo algo) {
        switch (algo) {
            case DIJKSTRA: return new DijkstraPathfinder();
            case BFS:      return new BfsPathfinder();
            case DFS:      return new DfsPathfinder();
            case DSTAR:    return new DStarLite();
            default:       return new DijkstraPathfinder();
        }
    }

    /**
     * Single-source Dijkstra from the base. Publishes base-&gt;building distances
     * and predecessor links, then flags the precompute ready and valid. Runs on
     * the inherited executor so it does not block setup.
     */
    private void dijkstraFromBase(Graph graph, String src) {
        ConcurrentHashMap<String, Double> dist = new ConcurrentHashMap<>();
        ConcurrentHashMap<String, String> prev = new ConcurrentHashMap<>();
        PriorityQueue<Pair<String, Double>> pq =
                new PriorityQueue<>(Comparator.comparing(Pair::second));

        dist.put(src, 0.0);
        pq.add(new Pair<>(src, 0.0));

        while (!pq.isEmpty()) {
            Pair<String, Double> node = pq.poll();
            Double best = dist.get(node.first());
            if (best == null || node.second() > best) {
                continue;
            }
            for (Edge e : graph.getEdges(node.first())) {
                String dst = e.to();
                double weight = node.second() + e.weight();
                Double known = dist.get(dst);
                if (known == null || weight < known) {
                    dist.put(dst, weight);
                    prev.put(dst, node.first());
                    pq.add(new Pair<>(dst, weight));
                }
            }
        }

        distFromBase.clear();
        distFromBase.putAll(dist);
        parent.clear();
        parent.putAll(prev);
        precomputeDone.set(true);
        precomputeValid.set(true);
    }

    /**
     * Reconstructs the base-&gt;target outbound path from the precompute, or
     * {@code null} if the precompute is invalid/not ready or the target is
     * unreachable in it.
     */
    private List<String> outboundFromPrecompute(String target) {
        if (!precomputeDone.get() || !precomputeValid.get()) {
            return null;
        }
        if (!origin.equals(target) && !parent.containsKey(target)) {
            return null;
        }
        ArrayList<String> path = new ArrayList<>();
        for (String cur = target; cur != null; cur = parent.get(cur)) {
            path.add(cur);
            if (cur.equals(origin)) {
                break;
            }
        }
        if (path.isEmpty() || !path.get(path.size() - 1).equals(origin)) {
            return null;
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * Invalidates the precompute only if the changed node actually participates
     * in the cached base-rooted shortest-path tree as a <em>tree node</em> -
     * i.e. it is the origin, or it appears as a parent of some node in the tree.
     *
     * <p>The earlier version also invalidated whenever the node merely had a
     * distance entry, but single-source Dijkstra settles essentially every
     * reachable node, so that condition fired on almost any block anywhere on
     * the map and killed the outbound fast-path on the first damage event. A
     * node that is only a <em>leaf</em> of the tree (never anyone's parent) being
     * blocked does not change any other node's cached route, so the cache stays
     * usable for every other destination. If a blocked node is an interior tree
     * node, the routes that pass through it are stale, so we drop the cache and
     * fall back to the engine for outbound from then on.
     */
    private void invalidatePrecomputeIfAffected(String node) {
        if (!precomputeValid.get()) {
            return;
        }
        if (origin.equals(node)) {
            precomputeValid.set(false);
            return;
        }
        // Interior tree node: some settled node has 'node' as its parent.
        // (parent is the predecessor map: child -> parent.) If 'node' is a value
        // in that map, routes descend through it and may now be stale.
        if (parent.containsValue(node)) {
            precomputeValid.set(false);
        }
        // Otherwise 'node' is a leaf (or absent): no cached route passes through
        // it, so the cache remains valid for all other destinations.
    }

    // ============================================================ message handling

    @Override
    protected void handle(Message s) {
        if (s == null || s.text == null) {
            return;
        }
        String[] b = s.text.split("\\|");
        if (b.length == 0) {
            return;
        }
        switch (b[0]) {
            case "RESCUE":        handleRescue(b);        break; // RESCUE|LOCATION|loc|PEOPLE|n
            case "ROAD":          handleRoad(b);          break; // ROAD|FROM|a|TO|b|STATUS|st
            case "LOCATION":      handleLocation(b);      break; // LOCATION|loc|COLLAPSED
            case "WAYPOINT_INVALID": handleWaypointInvalid(b); break;
            case "PATH_INVALID":  handlePathInvalid(b);   break;
            case "VEHICLE":       handleVehicle(b);       break; // various VEHICLE|n|...
            default:
                // Unknown message; ignore.
                break;
        }
    }

    // ---- RESCUE|LOCATION|loc|PEOPLE|n ----
    private void handleRescue(String[] b) {
        if (b.length < 5) return;
        String loc = b[2];
        // P5: if the location has already collapsed, drop the request.
        if (roadMap.isCollapsed(loc)) {
            return;
        }
        dispatchOrQueue(loc);
    }

    /** Assigns a free vehicle to a rescue location, or queues it if none free. */
    private void dispatchOrQueue(String loc) {
        Integer v = fleet.findAvailableVehicle();
        if (v == null) {
            pendingRescues.add(loc); // no vehicle now; retry when one frees up
            return;
        }
        assignedRescue.put(v, loc);
        // Outbound: precompute fast-path, else engine.
        List<String> pre = outboundFromPrecompute(loc);
        if (pre != null) {
            // Dispatch directly via the sender (mark dispatched, emit PATH).
            fleet.onDispatched(v, pre, loc, false);
            emitPath(v, pre);
        } else {
            engine.submitOutbound(v, origin, loc);
        }
    }

    // ---- ROAD|FROM|a|TO|b|STATUS|CLEAR|BLOCKED ----
    private void handleRoad(String[] b) {
        if (b.length < 6) return;
        String from = b[2], to = b[4], status = b[5];
        boolean open = "CLEAR".equals(status);

        roadMap.setEdgeOpen(from, to, open);
        engine.reportEdgeChange(from, to);   // no-op unless D* mode

        if (!open) {
            invalidatePrecomputeIfAffected(from);
            invalidatePrecomputeIfAffected(to);
            // Reactive policy for blocks: we do NOT proactively halt. If this
            // road is on a moving vehicle's remaining path, the vehicle will
            // halt itself at the block and we reroute then (WAYPOINT_INVALID +
            // VEHICLE HALTED). Nothing to do here beyond updating the graph.
        } else {
            // A road re-opened: retry any pending rescues that might now be
            // reachable. (Returns that previously failed are treated as lost
            // per P4, so we do not revive them.)
            retryPending();
        }
    }

    // ---- LOCATION|loc|COLLAPSED ----
    private void handleLocation(String[] b) {
        if (b.length < 3 || !"COLLAPSED".equals(b[2])) return;
        String node = b[1];
        roadMap.markCollapsed(node);
        engine.reportCollapse(node);            // propagate to all predecessors in D* mode
        invalidatePrecomputeIfAffected(node);

        // P3: a vehicle DESTROYED is handled elsewhere; here we try to SAVE any
        // vehicle whose IMMINENT next step would drive into the collapsed node,
        // by proactively halting it and rerouting once it stops. We deliberately
        // do NOT scan whole remaining paths: a collapse several hops ahead is
        // caught by the Simulator's per-segment validation when the vehicle
        // actually reaches that segment, and halting the whole affected
        // sub-fleet at once floods the single D* worker with reroutes.
        if (PROACTIVE_COLLAPSE_REROUTE) {
            for (VehicleManager.VehicleState vs : fleet.all()) {
                if (vs.state() == VehicleManager.MissionState.LOST) continue;
                boolean moving = vs.state() == VehicleManager.MissionState.DISPATCHED_OUT
                        || vs.state() == VehicleManager.MissionState.RETURNING;
                if (moving && fleet.nextStepEntersNode(vs.number(), node)) {
                    String goal = (vs.state() == VehicleManager.MissionState.RETURNING)
                            ? origin
                            : assignedRescue.getOrDefault(vs.number(), origin);
                    requestHaltForReroute(vs.number(), goal);
                }
            }
        }
    }

    // ---- WAYPOINT_INVALID|VEHICLE|n|FROM|a|TO|b|ROAD|reason ----
    private void handleWaypointInvalid(String[] b) {
        if (b.length < 8) return;
        int v = parseInt(b[2]);
        String from = b[4], to = b[6], reason = b[7];
        if (v < 0) return;

        // Update graph knowledge from what we just learned.
        if ("BLOCKED".equals(reason)) {
            roadMap.setEdgeOpen(from, to, false);
            engine.reportEdgeChange(from, to);
            invalidatePrecomputeIfAffected(from);
            invalidatePrecomputeIfAffected(to);
        }
        // The vehicle will auto-halt at 'from' (last good waypoint). We react on
        // the ensuing VEHICLE HALTED, so just record the intended goal: continue
        // toward whatever it was heading to.
        VehicleManager.VehicleState vs = fleet.get(v);
        if (vs == null || vs.state() == VehicleManager.MissionState.LOST) return;
        String goal = (vs.state() == VehicleManager.MissionState.RETURNING)
                ? origin
                : assignedRescue.getOrDefault(v, origin);
        rerouteOnHalt.put(v, goal);
        // No HALT needed from us; the vehicle halts itself.
    }

    // ---- PATH_INVALID|VEHICLE|n|reason ----
    private void handlePathInvalid(String[] b) {
        if (b.length < 4) return;
        int v = parseInt(b[2]);
        String reason = b[3];
        if (v < 0) return;
        // STILL_MOVING: our PATH raced a not-yet-halted vehicle. Re-issue a halt
        // and reroute when it stops. Others (DESTROYED/INVALID_NUMBER/
        // INVALID_STARTING_POINT) we cannot easily recover; leave the vehicle.
        if ("STILL_MOVING".equals(reason)) {
            VehicleManager.VehicleState vs = fleet.get(v);
            if (vs != null && vs.state() != VehicleManager.MissionState.LOST) {
                String goal = (vs.state() == VehicleManager.MissionState.RETURNING)
                        ? origin
                        : assignedRescue.getOrDefault(v, origin);
                requestHaltForReroute(v, goal);
            }
        }
    }

    // ---- VEHICLE|n|<EVENT>|... ----
    private void handleVehicle(String[] b) {
        if (b.length < 3) return;
        int v = parseInt(b[1]);
        if (v < 0) return;
        String event = b[2];

        switch (event) {
            case "ARRIVED": {            // VEHICLE|n|ARRIVED|LOCATION|loc
                if (b.length < 5) return;
                String loc = b[4];
                fleet.onArrived(v, loc);
                break;
            }
            case "HALTED": {             // VEHICLE|n|HALTED|LOCATION|loc
                if (b.length < 5) return;
                String loc = b[4];
                fleet.onHalted(v, loc);
                onVehicleStopped(v, loc);
                break;
            }
            case "RETURNED": {           // VEHICLE|n|RETURNED|RESCUED|x
                fleet.onReturned(v);
                assignedRescue.remove(v);
                rerouteOnHalt.remove(v);
                retryPending();          // a vehicle just freed up
                break;
            }
            case "DESTROYED": {          // VEHICLE|n|DESTROYED|LOCATION|loc|PEOPLE|x
                fleet.onDestroyed(v);    // P3 + P4: vehicle (and any cargo) lost
                assignedRescue.remove(v);
                rerouteOnHalt.remove(v);
                break;
            }
            default:
                break;
        }
    }

    /**
     * Called when a vehicle has come to a stop (VEHICLE HALTED). Decides the
     * follow-up mission:
     *  - if it halted at its outbound rescue goal, switch to a return mission;
     *  - if it stopped early (block / collapse reroute), compute a new path to
     *    its intended goal via the engine.
     */
    private void onVehicleStopped(int v, String loc) {
        VehicleManager.VehicleState vs = fleet.get(v);
        if (vs == null || vs.state() == VehicleManager.MissionState.LOST) return;

        String rescueGoal = assignedRescue.get(v);

        // Case 1: reached the rescue building -> begin a return mission.
        if (rescueGoal != null && loc.equals(rescueGoal)
                && vs.state() == VehicleManager.MissionState.DISPATCHED_OUT) {
            fleet.setAtRescue(v);
            // Submit a return-to-base computation from the building.
            engine.submitReturn(v, loc);
            return;
        }

        // Case 2: an intended reroute is pending (block/collapse/still-moving).
        String pending = rerouteOnHalt.remove(v);
        if (pending != null) {
            if (pending.equals(origin)) {
                // Reroute home via the configured return algorithm.
                engine.submitReturn(v, loc);
            } else {
                // Reroute toward the (outbound) rescue goal from current location.
                dispatchRerouteToGoal(v, loc, pending);
            }
            return;
        }

        // Case 3: stopped at base as end of a return path but RETURNED not yet
        // seen, or an idle halt - nothing to do; VehicleManager holds location.
    }

    /**
     * Reroutes a vehicle toward a non-base goal (an outbound building) after it
     * was halted mid-outbound. Uses the precompute if valid (it encodes
     * base-&gt;building, not loc-&gt;building, so only usable when loc==origin),
     * otherwise submits an outbound engine task from the current location.
     */
    private void dispatchRerouteToGoal(int v, String loc, String goal) {
        // The outbound precompute is rooted at base, so it only helps when the
        // vehicle is back at base. Mid-route, go through the engine outbound.
        if (loc.equals(origin)) {
            List<String> pre = outboundFromPrecompute(goal);
            if (pre != null) {
                fleet.onDispatched(v, pre, goal, false);
                emitPath(v, pre);
                return;
            }
        }
        engine.submitOutbound(v, loc, goal);
    }

    /**
     * Sends a HALT for a vehicle and records the goal to reroute toward once the
     * VEHICLE HALTED confirmation arrives.
     */
    private void requestHaltForReroute(int v, String goal) {
        if (fleet.get(v) != null
                && fleet.get(v).state() != VehicleManager.MissionState.HALTING) {
            rerouteOnHalt.put(v, goal);
            fleet.onHaltRequested(v);
            emitHalt(v);
        }
    }

    /** Re-attempts queued rescue requests while vehicles are available. */
    private void retryPending() {
        while (fleet.availableCount() > 0) {
            String loc = pendingRescues.poll();
            if (loc == null) {
                break;
            }
            if (roadMap.isCollapsed(loc)) {
                continue; // P5: dropped
            }
            dispatchOrQueue(loc);
        }
    }

    // ============================================================ sender thread

    /**
     * Starts the dedicated sender: drains computed {@link PathResult}s and emits
     * PATH messages (and triggers follow-up state). Isolating the blocking
     * {@code outMessageQueue.put} here keeps it off the comms and worker threads.
     */
    private void startSender() {
        if (!senderRunning.compareAndSet(false, true)) {
            return;
        }
        senderThread = new Thread(() -> {
            BlockingQueue<PathResult> results = engine.results();
            try {
                while (senderRunning.get()) {
                    PathResult r = results.take();
                    handleResult(r);
                }
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }, "MyDisasterResponder-Sender");
        senderThread.setDaemon(true);
        senderThread.start();
    }

    /** Turns a computed path result into a dispatch (or records unreachability). */
    private void handleResult(PathResult r) {
        int v = r.vehicle();
        VehicleManager.VehicleState vs = fleet.get(v);
        if (vs == null || vs.state() == VehicleManager.MissionState.LOST) {
            return;
        }
        if (!r.hasPath()) {
            // P4 / unreachable: leave the vehicle parked; treat as lost for now.
            return;
        }
        boolean returning = r.kind() == PathTask.Kind.RETURN
                && r.goal() != null && r.goal().equals(origin);
        fleet.onDispatched(v, r.path(), r.goal(), returning);
        emitPath(v, r.path());
    }

    // ============================================================ message emit

    /** Builds and enqueues a PATH message for a vehicle along {@code waypoints}. */
    private void emitPath(int vehicle, List<String> waypoints) {
        if (waypoints == null || waypoints.size() < 2) {
            // A single-node or empty path is not a valid dispatch (the vehicle is
            // already there). Nothing to send.
            return;
        }
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < waypoints.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(waypoints.get(i));
        }
        send(new Message("PATH|VEHICLE|" + vehicle + "|WAYPOINTS|" + sb));
    }

    /** Builds and enqueues a HALT message for a vehicle. */
    private void emitHalt(int vehicle) {
        send(new Message("HALT|VEHICLE|" + vehicle));
    }

    /** Puts a message on the outgoing queue (may block; only called on sender/comms). */
    private void send(Message m) {
        try {
            outMessageQueue.put(m);
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }

    // ============================================================ shutdown

    @Override
    public void shutdown() {
        senderRunning.set(false);
        if (senderThread != null) {
            senderThread.interrupt();
        }
        if (engine != null) {
            engine.shutdown();
        }
        super.shutdown();
    }

    // ============================================================ helpers

    private static int parseInt(String s) {
        try {
            return Integer.parseInt(s);
        } catch (NumberFormatException e) {
            return -1;
        }
    }
}