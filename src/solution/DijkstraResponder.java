package solution;

/**
 * Dijkstra for both outbound and return. The optimal one-shot baseline: always
 * returns a minimum-cost path, recomputed fresh each time, so it's the yardstick
 * for route quality that the faster/suboptimal solutions are judged against.
 * Returns run on the parallel pool (one-shot mode), not the dedicated D* worker.
 */
public class DijkstraResponder extends MyDisasterResponder {

    @Override
    protected Algo outboundAlgo() {
        return Algo.DIJKSTRA;
    }

    @Override
    protected Algo returnAlgo() {
        return Algo.DIJKSTRA;
    }
}