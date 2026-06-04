package solution.Pathfinding;

import solution.Graph;
import java.util.List;
import java.util.Optional;

public interface Pathfinder {
    Optional<List<String>> getPath(Graph roadMap, String start, String end);
}
