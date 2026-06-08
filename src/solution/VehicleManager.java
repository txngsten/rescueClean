package solution;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Tracks the live state of every rescue vehicle: where it is, what mission it is
 * on, the path it is currently following, and how far along that path it has
 * confirmed progress. This is the bookkeeping that lets the responder react to
 * road blocks and collapses without re-deriving a vehicle's situation from
 * scratch each time. All mutating calls come from the single comms thread, so
 * per-vehicle records need no locking.
 */
public final class VehicleManager {

    public enum MissionState {
        IDLE,
        DISPATCHED_OUT,
        AT_RESCUE,
        RETURNING,
        // We sent a HALT and are awaiting VEHICLE HALTED before we may issue a new PATH.
        HALTING,
        LOST
    }

    public static final class VehicleState {
        final int number;
        MissionState state;
        String location;
        List<String> path;       // last dispatched waypoint list (immutable copy)
        int confirmedIndex;      // index in 'path' of last confirmed waypoint
        String goal;
        int peopleOnBoard;

        VehicleState(int number, String startLocation) {
            this.number = number;
            this.state = MissionState.IDLE;
            this.location = startLocation;
            this.path = Collections.emptyList();
            this.confirmedIndex = 0;
            this.goal = null;
            this.peopleOnBoard = 0;
        }

        public int number()            { return number; }
        public MissionState state()    { return state; }
        public String location()       { return location; }
        public List<String> path()     { return path; }
        public int confirmedIndex()    { return confirmedIndex; }
        public String goal()           { return goal; }
        public int peopleOnBoard()     { return peopleOnBoard; }

        public List<String> remainingPath() {
            if (path == null || path.isEmpty() || confirmedIndex >= path.size() - 1) {
                return Collections.emptyList();
            }
            return new ArrayList<>(path.subList(confirmedIndex, path.size()));
        }
    }

    private final Map<Integer, VehicleState> vehicles = new ConcurrentHashMap<>();
    private final String base;

    public VehicleManager(String base, int numberOfVehicles) {
        this.base = base;
        for (int v = 1; v <= numberOfVehicles; v++) {
            vehicles.put(v, new VehicleState(v, base));
        }
    }

    public VehicleState get(int vehicle) {
        return vehicles.get(vehicle);
    }

    // Returns the lowest-numbered IDLE vehicle at base, for deterministic dispatch order.
    public Integer findAvailableVehicle() {
        Integer best = null;
        for (VehicleState vs : vehicles.values()) {
            if (vs.state == MissionState.IDLE && vs.location.equals(base)) {
                if (best == null || vs.number < best) {
                    best = vs.number;
                }
            }
        }
        return best;
    }

    public void onDispatched(int vehicle, List<String> path, String goal, boolean returning) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.state == MissionState.LOST) {
            return;
        }
        vs.path = (path == null) ? Collections.emptyList() : new ArrayList<>(path);
        vs.confirmedIndex = 0;
        vs.goal = goal;
        vs.state = returning ? MissionState.RETURNING : MissionState.DISPATCHED_OUT;
    }

    public void onArrived(int vehicle, String location) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.state == MissionState.LOST) {
            return;
        }
        vs.location = location;
        // Advance the confirmed index if this matches the next waypoint.
        int next = vs.confirmedIndex + 1;
        if (next < vs.path.size() && vs.path.get(next).equals(location)) {
            vs.confirmedIndex = next;
        }
    }

    public void onHalted(int vehicle, String location) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.state == MissionState.LOST) {
            return;
        }
        vs.location = location;
    }

    public void onHaltRequested(int vehicle) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs != null && vs.state != MissionState.LOST) {
            vs.state = MissionState.HALTING;
        }
    }

    public void setAtRescue(int vehicle) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs != null && vs.state != MissionState.LOST) {
            vs.state = MissionState.AT_RESCUE;
        }
    }

    public void onPeopleTransferred(int vehicle, int people) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs != null && vs.state != MissionState.LOST) {
            vs.peopleOnBoard = people;
        }
    }

    public void onReturned(int vehicle) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.state == MissionState.LOST) {
            return;
        }
        vs.state = MissionState.IDLE;
        vs.location = base;
        vs.path = Collections.emptyList();
        vs.confirmedIndex = 0;
        vs.goal = null;
        vs.peopleOnBoard = 0;
    }

    public void onDestroyed(int vehicle) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null) {
            return;
        }
        vs.state = MissionState.LOST;
        vs.peopleOnBoard = 0;
        vs.path = Collections.emptyList();
    }

    public boolean roadOnRemainingPath(int vehicle, String from, String to) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.path == null) {
            return false;
        }
        // Scan from the confirmed index forward.
        for (int i = Math.max(0, vs.confirmedIndex); i + 1 < vs.path.size(); i++) {
            if (vs.path.get(i).equals(from) && vs.path.get(i + 1).equals(to)) {
                return true;
            }
        }
        return false;
    }

    public boolean nodeOnRemainingPath(int vehicle, String node) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.path == null) {
            return false;
        }
        for (int i = Math.max(0, vs.confirmedIndex); i < vs.path.size(); i++) {
            if (vs.path.get(i).equals(node)) {
                return true;
            }
        }
        return false;
    }

    // Checks only the imminent next step, not the whole remaining path. Collapses
    // further ahead are handled reactively via WAYPOINT_INVALID to avoid a reroute
    // storm and let the reroute use the vehicle's real position at that time.
    public boolean nextStepEntersNode(int vehicle, String node) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.path == null || vs.path.isEmpty()) {
            return false;
        }
        int next = vs.confirmedIndex + 1;
        return next < vs.path.size() && vs.path.get(next).equals(node);
    }

    public Iterable<VehicleState> all() {
        return vehicles.values();
    }

    public int availableCount() {
        int n = 0;
        for (VehicleState vs : vehicles.values()) {
            if (vs.state == MissionState.IDLE && vs.location.equals(base)) {
                n++;
            }
        }
        return n;
    }
}