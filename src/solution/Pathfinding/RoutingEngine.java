package solution.Pathfinding;

import solution.Graph;
import solution.Pathfinding.Algorithms.IncrementalPathfinder;
import solution.Pathfinding.Algorithms.Pathfinder;

import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Manages all path computation off the comms thread, using two priority tiers
 * and one of two threading models chosen at construction.
 *
 * <h2>Two tiers</h2>
 * Work is split across two FIFO queues:
 * <ul>
 *   <li><b>high</b> - return-to-base (and, in D* Lite mode, edge-change /
 *       collapse) tasks. A stranded vehicle that cannot get home is more urgent
 *       than dispatching a fresh one, and map updates must be applied before the
 *       returns that depend on them.</li>
 *   <li><b>low</b> - outbound base-&gt;building tasks.</li>
 * </ul>
 * The number of people per rescue is constant across the simulation, so there
 * is nothing to prioritise <em>within</em> a tier; plain FIFO queues with a
 * strict high-before-low drain give the required "return beats outbound"
 * guarantee without the overhead of a comparator-based priority queue.
 *
 * <h2>Two modes (chosen at construction)</h2>
 * <ul>
 *   <li><b>D* Lite return mode</b> (when the configured return strategy is an
 *       {@link IncrementalPathfinder}): the shared base-rooted D* Lite instance
 *       is stateful and not thread-safe, so a <em>single dedicated worker</em>
 *       owns it and drains the high queue (returns + edge-changes + collapses).
 *       Outbound work still runs on the parallel pool. The comms thread never
 *       touches D* Lite; map changes are submitted as EDGE_CHANGE / COLLAPSE
 *       tasks so the dedicated worker applies them on its own thread.
 *
 *       <p>Crucially, the worker drains <em>all</em> immediately-available map
 *       updates before servicing a return: D* Lite is designed to absorb a batch
 *       of edge-cost changes and then run a single {@code computeShortestPath}
 *       (which {@code replan} triggers). Applying every queued update first means
 *       one repair, not one repair per change.</li>
 *   <li><b>One-shot return mode</b> (Dijkstra / BFS / DFS returns): the return
 *       algorithm is stateless, so returns are parallelisable. There is no
 *       dedicated worker; the parallel pool drains <em>both</em> queues,
 *       preferring high over low.</li>
 * </ul>
 *
 * <h2>Results</h2>
 * Workers never call {@code outMessageQueue.put} (it can block: the Simulator's
 * inbound queue is bounded). They publish a {@link PathResult} to the result
 * queue, which the responder's dedicated sender drains.
 *
 * @author solution
 */
public final class RoutingEngine {

    private final Graph graph;
    private final String base;

    // Strategies. Outbound is always a plain one-shot Pathfinder. Return is
    // either an IncrementalPathfinder (D* Lite mode) or a one-shot Pathfinder.
    private final Pathfinder outboundAlgo;
    private final Pathfinder returnAlgo;
    private final IncrementalPathfinder incrementalReturn; // non-null iff D* mode

    private final boolean dStarMode;
    private final int poolSize;

    private final BlockingQueue<PathTask> highQueue = new LinkedBlockingQueue<>();
    private final BlockingQueue<PathTask> lowQueue = new LinkedBlockingQueue<>();
    private final BlockingQueue<PathResult> results = new LinkedBlockingQueue<>();

    private final AtomicBoolean running = new AtomicBoolean(false);
    private Thread[] poolThreads;
    private Thread dedicatedReturnThread; // non-null iff D* mode

    // Poison sentinels to unblock workers on shutdown.
    private static final PathTask POISON =
            PathTask.edgeChange("__POISON__", "__POISON__");

    /**
     * Builds a routing engine. The threading model is selected from the type of
     * {@code returnAlgo}: if it is an {@link IncrementalPathfinder}, the engine
     * runs in D* Lite return mode (dedicated single return worker); otherwise it
     * runs in one-shot return mode (returns share the parallel pool).
     *
     * @param graph        the shared road network (already populated)
     * @param base         the fleet base node (D* Lite's fixed goal)
     * @param outboundAlgo stateless one-shot pathfinder for base-&gt;building
     * @param returnAlgo   pathfinder for building-&gt;base; if it implements
     *                     {@link IncrementalPathfinder} the engine enters D* mode
     * @param poolSize     number of parallel worker threads (>= 1)
     */
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

    /** @return true if running in D* Lite return mode. */
    public boolean isDStarMode() {
        return dStarMode;
    }

    /** @return the queue from which the responder's sender drains results. */
    public BlockingQueue<PathResult> results() {
        return results;
    }

    /**
     * Starts worker threads. In D* mode this initialises the shared D* Lite
     * instance (rooted at base) on the dedicated return thread and starts that
     * thread plus the outbound pool. In one-shot mode it starts only the pool.
     */
    public void start() {
        if (!running.compareAndSet(false, true)) {
            return; // already started
        }

        if (dStarMode) {
            // Dedicated worker: owns and initialises the D* Lite instance, then
            // drains the high queue (returns + edge-changes + collapses) forever.
            dedicatedReturnThread = new Thread(this::dedicatedReturnLoop,
                    "RoutingEngine-DStarReturnWorker");
            dedicatedReturnThread.setDaemon(true);
            dedicatedReturnThread.start();

            // Outbound pool drains only the low queue.
            poolThreads = new Thread[poolSize];
            for (int i = 0; i < poolSize; i++) {
                poolThreads[i] = new Thread(this::outboundOnlyLoop,
                        "RoutingEngine-Outbound-" + i);
                poolThreads[i].setDaemon(true);
                poolThreads[i].start();
            }
        } else {
            // One-shot mode: pool drains both queues, high first.
            poolThreads = new Thread[poolSize];
            for (int i = 0; i < poolSize; i++) {
                poolThreads[i] = new Thread(this::bothQueuesLoop,
                        "RoutingEngine-Worker-" + i);
                poolThreads[i].setDaemon(true);
                poolThreads[i].start();
            }
        }
    }

    // ---------------------------------------------------------------- submission

    /** Submits an outbound (base-&gt;building) compute request. */
    public void submitOutbound(int vehicle, String start, String goal) {
        lowQueue.add(PathTask.outbound(vehicle, start, goal));
    }

    /** Submits a return-to-base compute request (high priority). */
    public void submitReturn(int vehicle, String start) {
        highQueue.add(PathTask.returnToBase(vehicle, start, base));
    }

    /**
     * Reports a single-arc road-status change. In D* mode this becomes a
     * high-priority EDGE_CHANGE task so the dedicated worker applies
     * {@code updateEdge} on its own thread. In one-shot mode there is no
     * incremental state to update, so this is a no-op (the one-shot searches
     * read the live graph each time).
     *
     * @param from changed arc source
     * @param to   changed arc target
     */
    public void reportEdgeChange(String from, String to) {
        if (dStarMode) {
            highQueue.add(PathTask.edgeChange(from, to));
        }
        // one-shot mode: nothing to do; graph already mutated by caller.
    }

    /**
     * Reports a node collapse. In D* mode this becomes a high-priority COLLAPSE
     * task so the dedicated worker applies {@code nodeCollapsed} on its own
     * thread, re-evaluating every predecessor of the collapsed node. In one-shot
     * mode it is a no-op (the live graph already excludes the collapsed node).
     *
     * @param node the collapsed node
     */
    public void reportCollapse(String node) {
        if (dStarMode) {
            highQueue.add(PathTask.collapse(node));
        }
        // one-shot mode: nothing to do; graph already marks the node collapsed.
    }

    // ---------------------------------------------------------------- worker loops

    /**
     * Dedicated D* Lite worker: initialises the instance, then processes the
     * high queue. EDGE_CHANGE / COLLAPSE tasks update the instance; RETURN tasks
     * call {@code replan}. All D* Lite access is confined to this single thread.
     *
     * <p>Before answering a RETURN, the worker drains every map update that is
     * already waiting (without blocking for more), so a burst of road changes is
     * folded into a single repair when {@code replan} runs.
     */
    private void dedicatedReturnLoop() {
        incrementalReturn.initialize(graph, base);
        try {
            while (running.get()) {
                PathTask t = highQueue.take();
                if (t == POISON) {
                    break;
                }
                if (applyIfMapUpdate(t)) {
                    // Opportunistically apply any further map updates queued
                    // right behind this one, so we repair once for the batch.
                    drainPendingMapUpdates();
                    continue;
                }
                if (t.kind() == PathTask.Kind.RETURN) {
                    // Make sure all currently-known map updates are applied
                    // before we repair, then replan once.
                    drainPendingMapUpdates();
                    long t0 = System.nanoTime();
                    List<String> path = incrementalReturn.replan(t.start());
                    long dt = System.nanoTime() - t0;
                    results.add(new PathResult(PathTask.Kind.RETURN,
                            t.vehicle(), base, path, dt));
                }
                // OUTBOUND never lands here.
            }
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Applies a task to the D* Lite instance if it is a map update (EDGE_CHANGE
     * or COLLAPSE) and returns true; returns false for anything else.
     */
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

    /**
     * Applies every map update (EDGE_CHANGE / COLLAPSE) currently in the high
     * queue and re-enqueues all other tasks (RETURN / POISON) in their original
     * relative order, without blocking.
     *
     * <p>The queue is snapshotted once via {@code drainTo}, so this terminates
     * even if it is all returns; map updates are applied to the D* Lite instance
     * and the remaining tasks are added back to the tail preserving their order.
     * A one-position shift of returns relative to a freshly-arriving update is
     * harmless: the next {@code replan} re-drains before computing.
     */
    private void drainPendingMapUpdates() {
        java.util.List<PathTask> snapshot = new java.util.ArrayList<>();
        highQueue.drainTo(snapshot);
        for (PathTask t : snapshot) {
            if (!applyIfMapUpdate(t)) {
                // Preserve non-update tasks (returns, poison) for processing.
                highQueue.add(t);
            }
        }
    }

    /** Outbound-only pool loop (D* mode): drains the low queue. */
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

    /**
     * One-shot mode pool loop: prefer the high queue, fall back to low. Uses a
     * short poll on high so a worker that grabbed nothing high doesn't ignore
     * low work, while still draining returns ahead of outbound when both are
     * available.
     */
    private void bothQueuesLoop() {
        try {
            while (running.get()) {
                PathTask t = highQueue.poll();
                if (t == null) {
                    // No return work right now; block briefly on low, but wake to
                    // re-check high so returns can't starve behind a long low queue.
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

    /** Runs a one-shot compute and publishes the result. */
    private void runOneShot(PathTask t, Pathfinder algo) {
        if (t.kind() == PathTask.Kind.EDGE_CHANGE || t.kind() == PathTask.Kind.COLLAPSE) {
            return; // one-shot mode ignores map-update tasks
        }
        long t0 = System.nanoTime();
        List<String> path = algo.findPath(graph, t.start(), t.goal());
        long dt = System.nanoTime() - t0;
        results.add(new PathResult(t.kind(), t.vehicle(), t.goal(), path, dt));
    }

    // ---------------------------------------------------------------- shutdown

    /**
     * Signals all workers to stop and wakes any blocked on queue takes. Safe to
     * call from the responder's shutdown path.
     */
    public void shutdown() {
        running.set(false);
        // Wake blocked workers with poison sentinels.
        if (dedicatedReturnThread != null) {
            highQueue.add(POISON);
        }
        if (poolThreads != null) {
            for (int i = 0; i < poolThreads.length; i++) {
                // Each pool worker may be blocked on either queue depending on
                // mode; add a poison to both to be safe.
                highQueue.add(POISON);
                lowQueue.add(POISON);
            }
        }
    }
}