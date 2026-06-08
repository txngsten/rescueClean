package solution;

import org.jdom2.*;
import org.jdom2.input.SAXBuilder;

import java.io.File;
import java.io.IOException;
import java.util.List;

/**
 * Parses a GraphML file into a Graph. Edges carry a "d1" data key for distance
 * weight; node elements are present in the file but only edges are needed to
 * build the adjacency structure (addEdge creates both endpoints).
 */
public class GraphBuilder {

    public static Graph buildFromGraphML(String file) throws JDOMException, IOException {
        SAXBuilder jdomBuilder = new SAXBuilder();

        String normalised = file.replace('\\', File.separatorChar);
        Document jdomDocument = jdomBuilder.build(new File(normalised));

        Element graphxml = jdomDocument.getRootElement();
        Namespace ns = graphxml.getNamespace();
        Element graph = graphxml.getChild("graph", ns);

        List<Element> nodes = graph.getChildren("node", ns);

        for (Element e: nodes) {
            String id = e.getAttribute("id").getValue();
        }

        List<Element> edges = graph.getChildren("edge", ns);

        Graph roadMap = new Graph();
        for (Element e : edges) {
            List<Attribute> at = e.getAttributes();
            String nS = e.getAttribute("source").getValue();
            String nD = e.getAttribute("target").getValue();

            List<Element> datas = e.getChildren("data",ns);
            for (Element d: datas) {
                if (d.getAttribute("key").getValue().equals("d1")) {
                    double dist = Double.parseDouble(d.getText());
                    roadMap.addEdge(nS, nD, dist);
                }
            }
        }
        return roadMap;
    }

}

