package solution;

public class Edge {
    private final int dst;
    private final double weight;
    private volatile boolean open;

    public Edge(int dst, double weight) {
        this.dst = dst;
        this.weight = weight;
        open = true;
    }

    public int to() {
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
