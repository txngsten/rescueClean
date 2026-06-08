package solution.Pathfinding;

import solution.Graph;
import solution.Pathfinding.Algorithms.IncrementalPathfinder;
import solution.Pathfinding.Algorithms.Pathfinder;

import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Manages all path computation off the comms thread, split across two priority
 * tiers (return-to-base before outbound) and one of two threading models. In D*
 * Lite mode a single dedicated worker owns the stateful incremental search and
 * drains returns plus map updates; outbound goes to a parallel pool. In one-shot
 * mode (Dijkstra/BFS/DFS) everything runs on the parallel pool, high queue first.
 * Workers publish PathResults to a result queue that the responder's sender
 * drains; they never touch the outgoing message queue directly.
 */
public final class RoutingEngine {

    private final Graph graph;
    private final String base;

    private final Pathfinder outboundAlgo;
    private final Pathfinder returnAlgo;
    // Non-null iff the return algorithm is incremental (D* mode).
    private final IncrementalPathfinder incrementalReturn;

    private final boolean dStarMode;
    private final int poolSize;

    private final BlockingQueue<PathTask> highQueue = new LinkedBlockingQueue<>();
    private final BlockingQueue<PathTask> lowQueue = new LinkedBlockingQueue<>();
    private final BlockingQueue<PathResult> results = new LinkedBlockingQueue<>();

    private final AtomicBoolean running = new AtomicBoolean(false);
    private Thread[] poolThreads;
    private Thread dedicatedReturnThread;

    private static final PathTask POISON =
            PathTask.edgeChange("__POISON__", "__POISON__");

    public RoutingEngine(Graph graph, String base,
                         Pathfinder outboundAlgo, Pathfinder returnAlgo,
                         int poolSize) {
        this.graph = graph;
        this.base = base;
        this.outboundAlgo = outboundAlgo;
        this.returnAlgo = returnAlgo;
        this.poolSize = Math.max(1, poolSize);

        if (returnAlgo instanceof IncrementalPathfinder) {
            this.dStarMode = true;
            this.incrementalReturn = (IncrementalPathfinder) returnAlgo;
        } else {
            this.dStarMode = false;
            this.incrementalReturn = null;
        }
    }

    public boolean isDStarMode() {
        return dStarMode;
    }

    public BlockingQueue<PathResult> results() {
        return results;
    }

    public void start() {
        if (!running.compareAndSet(false, true)) {
            return;
        }

        if (dStarMode) {
            dedicatedReturnThread = new Thread(this::dedicatedReturnLoop,
                    "RoutingEngine-DStarReturnWorker");
            dedicatedReturnThread.setDaemon(true);
            dedicatedReturnThread.start();

            poolThreads = new Thread[poolSize];
            for (int i = 0; i < poolSize; i++) {
                poolThreads[i] = new Thread(this::outboundOnlyLoop,
                        "RoutingEngine-Outbound-" + i);
                poolThreads[i].setDaemon(true);
                poolThreads[i].start();
            }
        } else {
            poolThreads = new Thread[poolSize];
            for (int i = 0; i < poolSize; i++) {
                poolThreads[i] = new Thread(this::bothQueuesLoop,
                        "RoutingEngine-Worker-" + i);
                poolThreads[i].setDaemon(true);
                poolThreads[i].start();
            }
        }
    }


    public void submitOutbound(int vehicle, String start, String goal) {
        lowQueue.add(PathTask.outbound(vehicle, start, goal));
    }

    public void submitReturn(int vehicle, String start) {
        highQueue.add(PathTask.returnToBase(vehicle, start, base));
    }

    // No-op in one-shot mode; graph already mutated by caller.
    public void reportEdgeChange(String from, String to) {
        if (dStarMode) {
            highQueue.add(PathTask.edgeChange(from, to));
        }
    }

    // No-op in one-shot mode; graph already marks the node collapsed.
    public void reportCollapse(String node) {
        if (dStarMode) {
            highQueue.add(PathTask.collapse(node));
        }
    }


    // Drains all queued map updates before each replan so a burst of road changes
    // folds into a single repair. All D* Lite access is confined to this thread.
    private void dedicatedReturnLoop() {
        incrementalReturn.initialize(graph, base);
        try {
            while (running.get()) {
                PathTask t = highQueue.take();
                if (t == POISON) {
                    break;
                }
                if (applyIfMapUpdate(t)) {
                    drainPendingMapUpdates();
                    continue;
                }
                if (t.kind() == PathTask.Kind.RETURN) {
                    drainPendingMapUpdates();
                    long t0 = System.nanoTime();
                    List<String> path = incrementalReturn.replan(t.start());
                    long dt = System.nanoTime() - t0;
                    results.add(new PathResult(PathTask.Kind.RETURN,
                            t.vehicle(), base, path, dt));
                }
            }
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }

    private boolean applyIfMapUpdate(PathTask t) {
        if (t.kind() == PathTask.Kind.EDGE_CHANGE) {
            incrementalReturn.updateEdge(t.from(), t.to());
            return true;
        }
        if (t.kind() == PathTask.Kind.COLLAPSE) {
            incrementalReturn.nodeCollapsed(t.from());
            return true;
        }
        return false;
    }

    // Snapshots the high queue, applies map updates, re-enqueues the rest.
    private void drainPendingMapUpdates() {
        java.util.List<PathTask> snapshot = new java.util.ArrayList<>();
        highQueue.drainTo(snapshot);
        for (PathTask t : snapshot) {
            if (!applyIfMapUpdate(t)) {
                highQueue.add(t);
            }
        }
    }

    private void outboundOnlyLoop() {
        try {
            while (running.get()) {
                PathTask t = lowQueue.take();
                if (t == POISON) {
                    break;
                }
                runOneShot(t, outboundAlgo);
            }
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }

    // Prefer high queue, fall back to low with a short poll so returns drain first.
    private void bothQueuesLoop() {
        try {
            while (running.get()) {
                PathTask t = highQueue.poll();
                if (t == null) {
                    t = lowQueue.poll(50, java.util.concurrent.TimeUnit.MILLISECONDS);
                    if (t == null) {
                        continue;
                    }
                }
                if (t == POISON) {
                    break;
                }
                Pathfinder algo = (t.kind() == PathTask.Kind.RETURN)
                        ? returnAlgo : outboundAlgo;
                runOneShot(t, algo);
            }
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }

    private void runOneShot(PathTask t, Pathfinder algo) {
        if (t.kind() == PathTask.Kind.EDGE_CHANGE || t.kind() == PathTask.Kind.COLLAPSE) {
            return;
        }
        long t0 = System.nanoTime();
        List<String> path = algo.findPath(graph, t.start(), t.goal());
        long dt = System.nanoTime() - t0;
        results.add(new PathResult(t.kind(), t.vehicle(), t.goal(), path, dt));
    }

    public void shutdown() {
        running.set(false);
        if (dedicatedReturnThread != null) {
            highQueue.add(POISON);
        }
        if (poolThreads != null) {
            for (int i = 0; i < poolThreads.length; i++) {
                highQueue.add(POISON);
                lowQueue.add(POISON);
            }
        }
    }
}