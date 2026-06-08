package solution.Pathfinding;

import solution.Graph;

import java.util.List;

public interface IncrementalPathfinder {
    void initialize(Graph graph, String goal);
    void updateEdge(String from, String to);
    List<String> replan(String start);
    String goal();
}
