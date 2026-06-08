package solution.Pathfinding.Algorithms;

import solution.Edge;
import solution.Graph;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Queue;
import java.util.Set;

/**
 * Breadth-first search returning the path with least amount of edges.
 */
public final class BfsPathfinder implements Pathfinder {

    @Override
    public List<String> findPath(Graph graph, String start, String goal) {
        if (start == null || goal == null) {
            return null;
        }
        if (start.equals(goal)) {
            List<String> trivial = new ArrayList<>(1);
            trivial.add(start);
            return trivial;
        }

        Queue<String> frontier = new ArrayDeque<>();
        Set<String> visited = new HashSet<>();
        HashMap<String, String> prev = new HashMap<>();

        frontier.add(start);
        visited.add(start);

        while (!frontier.isEmpty()) {
            String u = frontier.poll();

            for (Edge e : graph.getEdges(u)) {
                String v = e.to();
                if (visited.add(v)) {            // add() returns false if already present
                    prev.put(v, u);
                    if (v.equals(goal)) {
                        return reconstruct(prev, start, goal);
                    }
                    frontier.add(v);
                }
            }
        }

        return null; // goal unreachable under current graph state
    }

    private List<String> reconstruct(HashMap<String, String> prev, String start, String goal) {
        ArrayList<String> path = new ArrayList<>();
        for (String cur = goal; cur != null; cur = prev.get(cur)) {
            path.add(cur);
            if (cur.equals(start)) {
                break;
            }
        }
        Collections.reverse(path);
        return path;
    }

    @Override
    public String name() {
        return "BFS";
    }
}