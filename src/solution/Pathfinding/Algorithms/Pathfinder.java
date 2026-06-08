package solution.Pathfinding.Algorithms;

import solution.Graph;
import java.util.List;

public interface Pathfinder {
    List<String> findPath(Graph roadMap, String start, String goal);
    String name();
}
