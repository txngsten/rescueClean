package solution;

import org.jdom2.JDOMException;
import sim.Message;
import util.ConfigurationInfo;
import util.Pair;

import java.io.IOException;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;

public class MyDisasterResponder extends DisasterResponder {
    private volatile Graph roadMap;
    private String origin;

    private final ConcurrentHashMap<String, Double> distFromBase = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, String> parent = new ConcurrentHashMap<>();
    private final AtomicBoolean done = new AtomicBoolean(false);

    @Override
    protected void handle(Message s) {

    }

    @Override
    protected void setup() {
        String mapFile = ConfigurationInfo.getMapFile(configFile);
        origin = ConfigurationInfo.getOrigin(configFile);

        try {
            roadMap = GraphBuilder.buildFromGraphML(mapFile);
        } catch (JDOMException | IOException e) {
            throw new RuntimeException("Failed to load road map: " + mapFile, e);
        }


    }

    private void dijkstraFromBase(Graph graph, String src) {
        ConcurrentHashMap<String, Double> dist = new ConcurrentHashMap<>();
        ConcurrentHashMap<String, String> prev = new ConcurrentHashMap<>();
        PriorityQueue<Pair<String, Double>> pq = new PriorityQueue<>(Comparator.comparing(Pair::second));

        dist.put(src, 0.0);
        pq.add(new Pair<>(src, 0.0));

        while (!pq.isEmpty()) {
            Pair<String, Double> node = pq.poll();

            Double best = dist.get(node.first());
            if (best == null || node.second() > best) {
                continue;
            }

            for (Edge e : graph.getEdges(node.first())) {
                String dst = e.to();
                double weight = node.second() + e.weight();
                Double knownDist = dist.get(dst);

                if (knownDist == null || weight < knownDist) {
                    dist.put(dst, weight);
                    prev.put(dst, node.first());
                    pq.add(new Pair<>(dst, weight));
                }
            }
        }

        distFromBase.clear();
        distFromBase.putAll(dist);
        parent.clear();
        parent.putAll(prev);
        done.set(true);
    }




}
