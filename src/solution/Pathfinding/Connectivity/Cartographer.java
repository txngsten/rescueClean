package solution.Pathfinding.Connectivity;

import solution.Graph;

public interface Cartographer {
    boolean canReach(Graph graph, Integer src, Integer dst);
}
