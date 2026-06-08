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
 * Concrete DisasterResponder that dispatches a fleet of vehicles to rescue
 * evacuees on a dynamic road network. The comms thread parses
 * messages, updates the VehicleManager, and delegates path computation to a
 * RoutingEngine. A dedicated sender thread drains computed results into PATH/HALT
 * messages, keeping the potentially blocking outbound put off both the comms and
 * worker threads. A Dijkstra precompute from base is cached at startup for fast
 * outbound dispatch, it's invalidated only when damage hits an interior node of
 * the cached shortest-path tree. Algorithm selection goes through overridable
 * outboundAlgo()/returnAlgo() accessors so subclasses can swap strategies.
 */
public class MyDisasterResponder extends DisasterResponder {

    public enum Algo { DIJKSTRA, BFS, DFS, DSTAR }

    // Used for base->building paths when the precompute is invalid.
    protected static final Algo OUTBOUND_ALGO = Algo.DIJKSTRA;

    // Used for building->base paths and all rerouting.
    protected static final Algo RETURN_ALGO = Algo.DSTAR;

    // When true, proactively halt a vehicle whose imminent next step leads into
    // a collapsed node. When false, the vehicle drives in and is destroyed.
    protected static final boolean PROACTIVE_COLLAPSE_REROUTE = true;

    private static final int POOL_SIZE =
            Math.max(1, Runtime.getRuntime().availableProcessors() - 1);

    private volatile Graph roadMap;
    private volatile String origin;

    private final ConcurrentHashMap<String, Double> distFromBase = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, String> parent = new ConcurrentHashMap<>();
    private final AtomicBoolean precomputeDone = new AtomicBoolean(false);
    private final AtomicBoolean precomputeValid = new AtomicBoolean(false);

    private VehicleManager fleet;
    private RoutingEngine engine;
    private Thread senderThread;
    private final AtomicBoolean senderRunning = new AtomicBoolean(false);

    // FIFO of rescue requests we could not serve yet; retried when a vehicle frees up.
    private final BlockingQueue<String> pendingRescues = new LinkedBlockingQueue<>();

    // Vehicle -> its current rescue goal (to know when to switch to return).
    private final ConcurrentHashMap<Integer, String> assignedRescue = new ConcurrentHashMap<>();

    // Vehicle -> goal to reroute toward once the HALTED confirmation arrives.
    private final ConcurrentHashMap<Integer, String> rerouteOnHalt = new ConcurrentHashMap<>();

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

    // Overridable so subclasses can swap algorithms (static finals can't be overridden).
    protected Algo outboundAlgo() {
        return OUTBOUND_ALGO;
    }

    protected Algo returnAlgo() {
        return RETURN_ALGO;
    }

    private Pathfinder makePathfinder(Algo algo) {
        switch (algo) {
            case DIJKSTRA: return new DijkstraPathfinder();
            case BFS:      return new BfsPathfinder();
            case DFS:      return new DfsPathfinder();
            case DSTAR:    return new DStarLite();
            default:       return new DijkstraPathfinder();
        }
    }

    // Runs on the inherited executor so it does not block setup.
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

    // Returns null if the precompute is invalid/not ready or the target is unreachable.
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

    // Only invalidates if the node is an interior node of the cached tree (i.e.
    // it is someone's parent). A leaf being blocked doesn't affect other routes.
    // Without this distinction, any block anywhere kills the cache on the first
    // damage event because single-source Dijkstra settles almost every node.
    private void invalidatePrecomputeIfAffected(String node) {
        if (!precomputeValid.get()) {
            return;
        }
        if (origin.equals(node)) {
            precomputeValid.set(false);
            return;
        }
        if (parent.containsValue(node)) {
            precomputeValid.set(false);
        }
    }

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
            case "RESCUE":        handleRescue(b);        break;
            case "ROAD":          handleRoad(b);          break;
            case "LOCATION":      handleLocation(b);      break;
            case "WAYPOINT_INVALID": handleWaypointInvalid(b); break;
            case "PATH_INVALID":  handlePathInvalid(b);   break;
            case "VEHICLE":       handleVehicle(b);       break;
            default:
                break;
        }
    }

    private void handleRescue(String[] b) {
        if (b.length < 5) return;
        String loc = b[2];
        if (roadMap.isCollapsed(loc)) {
            return;
        }
        dispatchOrQueue(loc);
    }

    private void dispatchOrQueue(String loc) {
        Integer v = fleet.findAvailableVehicle();
        if (v == null) {
            pendingRescues.add(loc);
            return;
        }
        assignedRescue.put(v, loc);
        List<String> pre = outboundFromPrecompute(loc);
        if (pre != null) {
            fleet.onDispatched(v, pre, loc, false);
            emitPath(v, pre);
        } else {
            engine.submitOutbound(v, origin, loc);
        }
    }

    private void handleRoad(String[] b) {
        if (b.length < 6) return;
        String from = b[2], to = b[4], status = b[5];
        boolean open = "CLEAR".equals(status);

        roadMap.setEdgeOpen(from, to, open);
        engine.reportEdgeChange(from, to);

        if (!open) {
            invalidatePrecomputeIfAffected(from);
            invalidatePrecomputeIfAffected(to);
        } else {
            retryPending();
        }
    }

    private void handleLocation(String[] b) {
        if (b.length < 3 || !"COLLAPSED".equals(b[2])) return;
        String node = b[1];
        roadMap.markCollapsed(node);
        engine.reportCollapse(node);
        invalidatePrecomputeIfAffected(node);

        // Only halt vehicles whose imminent next step enters the collapsed node.
        // Collapses further ahead are caught reactively via WAYPOINT_INVALID.
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

    private void handleWaypointInvalid(String[] b) {
        if (b.length < 8) return;
        int v = parseInt(b[2]);
        String from = b[4], to = b[6], reason = b[7];
        if (v < 0) return;

        if ("BLOCKED".equals(reason)) {
            roadMap.setEdgeOpen(from, to, false);
            engine.reportEdgeChange(from, to);
            invalidatePrecomputeIfAffected(from);
            invalidatePrecomputeIfAffected(to);
        }
        // Vehicle auto-halts at 'from'; we reroute on the ensuing VEHICLE HALTED.
        VehicleManager.VehicleState vs = fleet.get(v);
        if (vs == null || vs.state() == VehicleManager.MissionState.LOST) return;
        String goal = (vs.state() == VehicleManager.MissionState.RETURNING)
                ? origin
                : assignedRescue.getOrDefault(v, origin);
        rerouteOnHalt.put(v, goal);
    }

    private void handlePathInvalid(String[] b) {
        if (b.length < 4) return;
        int v = parseInt(b[2]);
        String reason = b[3];
        if (v < 0) return;
        // STILL_MOVING: our PATH raced a not-yet-halted vehicle; re-halt and reroute.
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

    private void handleVehicle(String[] b) {
        if (b.length < 3) return;
        int v = parseInt(b[1]);
        if (v < 0) return;
        String event = b[2];

        switch (event) {
            case "ARRIVED": {
                if (b.length < 5) return;
                String loc = b[4];
                fleet.onArrived(v, loc);
                break;
            }
            case "HALTED": {
                if (b.length < 5) return;
                String loc = b[4];
                fleet.onHalted(v, loc);
                onVehicleStopped(v, loc);
                break;
            }
            case "RETURNED": {
                fleet.onReturned(v);
                assignedRescue.remove(v);
                rerouteOnHalt.remove(v);
                retryPending();
                break;
            }
            case "DESTROYED": {
                fleet.onDestroyed(v);
                assignedRescue.remove(v);
                rerouteOnHalt.remove(v);
                break;
            }
            default:
                break;
        }
    }

    private void onVehicleStopped(int v, String loc) {
        VehicleManager.VehicleState vs = fleet.get(v);
        if (vs == null || vs.state() == VehicleManager.MissionState.LOST) return;

        String rescueGoal = assignedRescue.get(v);

        // Reached the rescue building -> begin a return mission.
        if (rescueGoal != null && loc.equals(rescueGoal)
                && vs.state() == VehicleManager.MissionState.DISPATCHED_OUT) {
            fleet.setAtRescue(v);
            // Submit a return-to-base computation from the building.
            engine.submitReturn(v, loc);
            return;
        }

        // Reroute pending from a block/collapse/still-moving event.
        String pending = rerouteOnHalt.remove(v);
        if (pending != null) {
            if (pending.equals(origin)) {
                engine.submitReturn(v, loc);
            } else {
                dispatchRerouteToGoal(v, loc, pending);
            }
            return;
        }

    }

    // The outbound precompute is rooted at base, so it only helps when loc==origin.
    private void dispatchRerouteToGoal(int v, String loc, String goal) {
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

    private void requestHaltForReroute(int v, String goal) {
        if (fleet.get(v) != null
                && fleet.get(v).state() != VehicleManager.MissionState.HALTING) {
            rerouteOnHalt.put(v, goal);
            fleet.onHaltRequested(v);
            emitHalt(v);
        }
    }

    private void retryPending() {
        while (fleet.availableCount() > 0) {
            String loc = pendingRescues.poll();
            if (loc == null) {
                break;
            }
            if (roadMap.isCollapsed(loc)) {
                continue;
            }
            dispatchOrQueue(loc);
        }
    }

    // Isolates the blocking outMessageQueue.put off the comms and worker threads.
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

    private void handleResult(PathResult r) {
        int v = r.vehicle();
        VehicleManager.VehicleState vs = fleet.get(v);
        if (vs == null || vs.state() == VehicleManager.MissionState.LOST) {
            return;
        }
        if (!r.hasPath()) {
            return;
        }
        boolean returning = r.kind() == PathTask.Kind.RETURN
                && r.goal() != null && r.goal().equals(origin);
        fleet.onDispatched(v, r.path(), r.goal(), returning);
        emitPath(v, r.path());
    }

    private void emitPath(int vehicle, List<String> waypoints) {
        if (waypoints == null || waypoints.size() < 2) {
            return;
        }
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < waypoints.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(waypoints.get(i));
        }
        send(new Message("PATH|VEHICLE|" + vehicle + "|WAYPOINTS|" + sb));
    }

    private void emitHalt(int vehicle) {
        send(new Message("HALT|VEHICLE|" + vehicle));
    }

    private void send(Message m) {
        try {
            outMessageQueue.put(m);
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }

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

    private static int parseInt(String s) {
        try {
            return Integer.parseInt(s);
        } catch (NumberFormatException e) {
            return -1;
        }
    }
}