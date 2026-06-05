package solution;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class Graph {
    private final HashMap<String, ArrayList<Edge>> graph;
    private final Set<String> collapsed;
    private final ReentrantReadWriteLock lock;

    public Graph() {
        graph = new HashMap<>();
        collapsed = new HashSet<>();
        lock = new ReentrantReadWriteLock();
    }

    public void addNode(String node) {
        lock.writeLock().lock();
        try {
            graph.putIfAbsent(node, new ArrayList<>());
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void addEdge(String src, String dst, Double weight) {
        lock.writeLock().lock();
        try {
            graph.putIfAbsent(src, new ArrayList<>());
            graph.putIfAbsent(dst, new ArrayList<>());
            graph.get(src).add(new Edge(dst, weight));
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void setEdgeOpen(String src, String dst, boolean open) {
        lock.writeLock().lock();
        try {
            ArrayList<Edge> edges = graph.get(src);
            if (edges == null) {
                return;
            }

            for (Edge e : edges) {
                if (e.to().equals(dst)) {
                    e.setOpen(open);
                }
            }
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void markCollapsed(String node) {
        lock.writeLock().lock();
        try {
            collapsed.add(node);
        } finally {
            lock.writeLock().unlock();
        }
    }

    public List<Edge> getEdges(String node) {
        lock.readLock().lock();
        try {
            if (collapsed.contains(node)) {
                return Collections.emptyList();
            }

            ArrayList<Edge> edges = graph.get(node);
            if (edges == null) {
                return Collections.emptyList();
            }

            ArrayList<Edge> result = new ArrayList<>();
            for (Edge e : edges) {
                if (e.isOpen() && !collapsed.contains(e.to())) {
                    result.add(e);
                }
            }
            return result;
        } finally {
            lock.readLock().unlock();
        }
    }

    public double getWeight(String src, String dst) {
        lock.readLock().lock();
        try {
            ArrayList<Edge> edges = graph.get(src);
            if (edges == null) {
                return -1;
            }

            for (Edge e : edges) {
                if (e.to().equals(dst)) {
                    return e.weight();
                }
            }
        } finally {
            lock.readLock().unlock();
        }
        return -1;
    }

    public boolean containsNode(String node) {
        lock.readLock().lock();
        try {
            return graph.containsKey(node);
        } finally {
            lock.readLock().unlock();
        }
    }

    public boolean containsEdge(String src, String dst) {
        lock.readLock().lock();
        try {
            ArrayList<Edge> edges = graph.get(src);
            if (edges == null) {
                return false;
            }

            for (Edge e : edges) {
                if (e.to().equals(dst)) {
                    return true;
                }
            }
        } finally {
            lock.readLock().unlock();
        }
        return false;
    }

    public boolean isCollapsed(String node) {
        lock.readLock().lock();
        try {
            return collapsed.contains(node);
        } finally {
            lock.readLock().unlock();
        }
    }

    public int size() {
        lock.readLock().lock();
        try {
            return graph.size();
        } finally {
            lock.readLock().unlock();
        }
    }
}