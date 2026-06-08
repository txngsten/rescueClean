package solution;

/*
 * BFS for both outbound and return. Searches by hop count and ignores edge
 * weights, so paths are fewest-intersections rather than shortest-distance.
 * Included to show the cost of dropping weights: fast to compute, but the routes
 * it picks may be longer in actual travel time than Dijkstra's.
 */
public class BfsResponder extends MyDisasterResponder {

    @Override
    protected Algo outboundAlgo() {
        return Algo.BFS;
    }

    @Override
    protected Algo returnAlgo() {
        return Algo.BFS;
    }
}