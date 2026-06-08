package solution.Pathfinding;

import solution.Graph;
import java.util.List;

public interface Pathfinder {
    List<String> findPath(Graph roadMap, String start, String goal);
    String name();
}
