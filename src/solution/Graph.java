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

            edges = graph.get(dst);
            if (edges == null) {
                return;
            }

            for  (Edge e: edges) {
                if (e.to() == src) {
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

    public List<Edge> getEdges(Integer node) {
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
            for (Edge e: edges) {
                if (e.isOpen() && !collapsed.contains(e.to())) {
                    result.add(e);
                }
            }
            return result;
        } finally {
            lock.readLock().unlock();
        }
    }

    public double getWeight(Integer src, Integer dst) {
        lock.readLock().lock();
        try {
            ArrayList<Edge> edges = graph.get(src);
            if (edges == null) {
                return -1;
            }

            for (Edge e: edges) {
                if (e.to() == dst) {
                    return e.weight();
                }
            }
        } finally {
            lock.readLock().unlock();
        }
        return -1;
    }

    public boolean containsNode(Integer node) {
        lock.readLock().lock();
        try {
            return graph.containsKey(node);
        } finally {
            lock.readLock().unlock();
        }
    }

    public boolean containsEdge(Integer src, Integer dst) {
        lock.readLock().lock();
        try {
            ArrayList<Edge> edges = graph.get(src);
            if (edges == null) {
                return false;
            }

            for (Edge e: edges) {
                if (e.to() == dst) {
                    return true;
                }
            }
        } finally {
            lock.readLock().unlock();
        }
        return false;
    }

    public boolean isCollapsed(Integer node) {
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
