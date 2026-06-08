package solution.Pathfinding.Algorithms;

import solution.Graph;

import java.util.List;

public interface IncrementalPathfinder extends Pathfinder {
    void initialize(Graph graph, String goal);
    void updateEdge(String from, String to);
    List<String> replan(String start);
    String goal();
}
