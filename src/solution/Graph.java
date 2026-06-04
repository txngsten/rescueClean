package solution;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class Graph {
    private final HashMap<Integer, ArrayList<Edge>> graph;
    private final Set<Integer> collapsed;
    private final ReentrantReadWriteLock lock;

    public Graph() {
        graph = new HashMap<>();
        collapsed = new HashSet<>();
        lock = new ReentrantReadWriteLock();
    }

    public void addNode(Integer node) {
        lock.writeLock().lock();
        try {
            graph.putIfAbsent(node, new ArrayList<>());
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void addEdge(Integer src, Integer dst, Double weight) {
        lock.writeLock().lock();
        try {
            graph.putIfAbsent(src, new ArrayList<>());
            graph.putIfAbsent(dst, new ArrayList<>());
            graph.get(src).add(new Edge(dst, weight));
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void setEdgeOpen(Integer src, Integer dst, boolean open) {
        lock.writeLock().lock();
        try {
            ArrayList<Edge> edges = graph.get(src);
            if (edges == null) {
                return;
            }

            for (Edge e: edges) {
                if (e.to() == dst) {
                    e.setOpen(open);
                }
            }
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void markCollapsed(Integer node) {
        lock.writeLock().lock();
        try {
            collapsed.add(node);
        } finally {
            lock.writeLock().unlock();
        }
    }


}
