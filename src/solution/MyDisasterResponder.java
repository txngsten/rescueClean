package solution;

import sim.Message;
import util.ConfigurationInfo;

public class MyDisasterResponder extends DisasterResponder {
    private volatile Graph roadMap;
    private String origin;

    MyDisasterResponder() {
        super();
    }

    @Override
    protected void handle(Message s) {

    }

    @Override
    protected void setup() {
        String mapFile = ConfigurationInfo.getMapFile(configFile);
        origin = ConfigurationInfo.getOrigin(configFile);

        try {
            roadMap = GraphBuilder.buildFromGraphML(mapFile);
        } catch (Exception e) {
            System.err.println(e.getMessage());
            System.exit(1);
        }



    }
}
