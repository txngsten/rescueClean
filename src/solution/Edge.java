package solution;

/**
 * A directed, weighted road segment in the graph. The open flag lets roads be
 * blocked and unblocked without removing and re-adding edges. The field is
 * volatile so the Graph's read-lock holders see writes from the write-lock side
 * immediately.
 */

public class Edge {
    private final String dst;
    private final double weight;
    private volatile boolean open;

    public Edge(String dst, double weight) {
        this.dst = dst;
        this.weight = weight;
        open = true;
    }

    public String to() {
        return dst;
    }

    public double weight() {
        return weight;
    }

    public boolean isOpen() {
        return open;
    }

    public void setOpen(boolean open) {
        this.open = open;
    }
}