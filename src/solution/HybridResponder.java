package solution;

/**
 * Dijkstra outbound, D* Lite return. The split bet: outbound dispatches are
 * one-shot and benefit from Dijkstra's optimal routes, while returns and
 * reroutes are where road damage hits hardest, so they go through incremental
 * D* Lite. Because the return algo is incremental, the engine runs in D* mode
 * (dedicated return worker); outbound Dijkstra still runs on the parallel pool.
 * Tests whether mixing beats committing to one algorithm for both directions.
 */
public class HybridResponder extends MyDisasterResponder {

    @Override
    protected Algo outboundAlgo() {
        return Algo.DIJKSTRA;
    }

    @Override
    protected Algo returnAlgo() {
        return Algo.DSTAR;
    }
}