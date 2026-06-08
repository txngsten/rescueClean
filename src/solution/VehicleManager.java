package solution;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Tracks the live state of every rescue vehicle: where it is, what mission it is
 * on, the path it is currently following, and how far along that path it has
 * confirmed progress. This is the bookkeeping that lets the responder react
 * correctly to road blocks and collapses without re-deriving a vehicle's
 * situation from scratch each time.
 *
 * <h2>Why each field is tracked</h2>
 * <ul>
 *   <li><b>state</b> ({@link MissionState}) - drives the protocol handshake. The
 *       Simulator forbids sending a new PATH while one is in operation, so we
 *       must know whether a vehicle is mid-path, awaiting a halt we requested,
 *       idle, or lost before issuing instructions.</li>
 *   <li><b>location</b> - the vehicle's last confirmed node, updated on every
 *       ARRIVED and HALTED. Ground truth for where a recompute must start
 *       from.</li>
 *   <li><b>path + confirmedIndex</b> - the waypoint list we last dispatched and
 *       the index of the last waypoint the vehicle has confirmed reaching.
 *       Together these answer two questions cheaply: (a) "is a newly blocked
 *       road / collapsed node actually on this vehicle's <em>remaining</em>
 *       route, or about to be stepped into?" - so we only proactively halt
 *       vehicles genuinely affected, and (b) "from which node do we recompute?"
 *       - the confirmed location.</li>
 *   <li><b>goal</b> - the vehicle's current destination (a building when
 *       outbound, the base when returning), so a recompute knows where it is
 *       headed.</li>
 *   <li><b>peopleOnBoard</b> - evacuees picked up but not yet delivered; informs
 *       prioritisation (a loaded vehicle that cannot get home loses its
 *       passengers) and lets us confirm RETURNED counts.</li>
 * </ul>
 *
 * <h2>Concurrency</h2>
 * All mutating calls originate from the single comms thread (inside
 * {@code handle}), so the per-vehicle records do not need internal locking.
 * The backing map is a {@link ConcurrentHashMap} purely so that worker threads
 * may safely <em>read</em> snapshots (e.g. a vehicle's current location) when
 * preparing a result, without risking a {@code ConcurrentModificationException}.
 *
 * @author solution
 */
public final class VehicleManager {

    /**
     * The lifecycle state of a single vehicle's mission.
     */
    public enum MissionState {
        /** Available at base, no active mission. */
        IDLE,
        /** En route from base to a rescue building (outbound). */
        DISPATCHED_OUT,
        /** Stopped at the rescue building (people transferred); needs a return path. */
        AT_RESCUE,
        /** En route from a building back to base (returning). */
        RETURNING,
        /** We have sent a HALT and are awaiting the VEHICLE HALTED confirmation
         *  before we may issue a new PATH. */
        HALTING,
        /** Destroyed (entered a collapsed location) - permanently unusable. */
        LOST
    }

    /**
     * Mutable per-vehicle record. Package-private fields, accessed only through
     * the enclosing manager's synchronised-by-single-thread methods.
     */
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

        /** @return the portion of the path not yet confirmed (from the vehicle's
         *  current confirmed position to the end), or empty if none. */
        public List<String> remainingPath() {
            if (path == null || path.isEmpty() || confirmedIndex >= path.size() - 1) {
                return Collections.emptyList();
            }
            return new ArrayList<>(path.subList(confirmedIndex, path.size()));
        }
    }

    private final Map<Integer, VehicleState> vehicles = new ConcurrentHashMap<>();
    private final String base;

    /**
     * @param base            the fleet base node, where all vehicles start
     * @param numberOfVehicles fleet size (vehicles numbered 1..n)
     */
    public VehicleManager(String base, int numberOfVehicles) {
        this.base = base;
        for (int v = 1; v <= numberOfVehicles; v++) {
            vehicles.put(v, new VehicleState(v, base));
        }
    }

    /**
     * @param vehicle the vehicle number
     * @return that vehicle's state record, or {@code null} if the number is
     *         unknown
     */
    public VehicleState get(int vehicle) {
        return vehicles.get(vehicle);
    }

    /**
     * Finds an available vehicle to dispatch: one that is IDLE at base. Returns
     * the lowest-numbered such vehicle for determinism, or {@code null} if none
     * is free.
     *
     * @return a free vehicle's number, or {@code null} if the whole fleet is busy
     */
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

    // ------------------------------------------------------------ state updates

    /**
     * Records that we have dispatched {@code vehicle} along {@code path} toward
     * {@code goal}, picking the appropriate in-transit state from the direction.
     *
     * @param vehicle the dispatched vehicle
     * @param path    the waypoint list sent (start..goal inclusive)
     * @param goal    the destination
     * @param returning true if this is a building-&gt;base path, false if base-&gt;building
     */
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

    /**
     * Updates location and path progress when a vehicle confirms reaching a
     * waypoint (VEHICLE ARRIVED). Advances {@code confirmedIndex} to the
     * arrived node if it matches the next expected waypoint.
     *
     * @param vehicle  the vehicle
     * @param location the node just reached
     */
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

    /**
     * Handles a VEHICLE HALTED. The vehicle is now stationary at {@code location}
     * and may receive a new PATH. The caller (responder) decides the follow-up
     * mission via the dedicated transition helpers; here we just record the stop.
     *
     * @param vehicle  the halted vehicle
     * @param location where it halted (ground truth)
     */
    public void onHalted(int vehicle, String location) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.state == MissionState.LOST) {
            return;
        }
        vs.location = location;
        // Caller (responder) decides the follow-up mission; we just mark it
        // stopped. Leaving state for the responder to set via the dedicated
        // transition helpers below keeps mission policy out of the manager.
    }

    /** Marks a vehicle as awaiting a halt we requested (HALT sent). */
    public void onHaltRequested(int vehicle) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs != null && vs.state != MissionState.LOST) {
            vs.state = MissionState.HALTING;
        }
    }

    /** Marks a vehicle stopped at the rescue building, ready to be returned. */
    public void setAtRescue(int vehicle) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs != null && vs.state != MissionState.LOST) {
            vs.state = MissionState.AT_RESCUE;
        }
    }

    /** Records evacuees transferred into a vehicle. */
    public void onPeopleTransferred(int vehicle, int people) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs != null && vs.state != MissionState.LOST) {
            vs.peopleOnBoard = people;
        }
    }

    /** Handles VEHICLE RETURNED: vehicle is home, free again, passengers delivered. */
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

    /** Handles VEHICLE DESTROYED: vehicle is permanently lost. */
    public void onDestroyed(int vehicle) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null) {
            return;
        }
        vs.state = MissionState.LOST;
        vs.peopleOnBoard = 0;
        vs.path = Collections.emptyList();
    }

    // ------------------------------------------------------------ queries

    /**
     * Determines whether a directed road {@code (from -> to)} lies on a
     * vehicle's <em>remaining</em> (not-yet-traversed) path. Used to decide
     * whether a ROAD BLOCKED message warrants proactively halting and replanning
     * this vehicle.
     *
     * @param vehicle the vehicle to check
     * @param from    the road's source endpoint
     * @param to      the road's target endpoint
     * @return true if the consecutive pair (from,to) appears in the remaining path
     */
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

    /**
     * Determines whether a node lies anywhere on a vehicle's remaining path.
     * Retained for completeness; the responder's collapse policy now uses the
     * narrower {@link #nextStepEntersNode(int, String)} instead, to avoid
     * halting vehicles for collapses many hops ahead.
     *
     * @param vehicle the vehicle to check
     * @param node    the node that collapsed
     * @return true if {@code node} appears in the remaining path
     */
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

    /**
     * Determines whether the vehicle's <em>imminent next step</em> would move it
     * into {@code node}. The imminent step is the segment from the last confirmed
     * waypoint (the vehicle's current position) to the very next waypoint. This
     * is the only case where a collapse demands a proactive halt: the vehicle is
     * about to enter the collapsed node on its next move and the Simulator's
     * per-segment validation would otherwise let it drive in and be destroyed.
     *
     * <p>Collapses further along the route are intentionally ignored here - they
     * are handled reactively when the vehicle actually reaches that segment
     * (WAYPOINT_INVALID), which both avoids a reroute storm and lets the reroute
     * be computed from the vehicle's real position with up-to-date map state.
     *
     * @param vehicle the vehicle to check
     * @param node    the node that collapsed
     * @return true if the next unconfirmed waypoint equals {@code node}
     */
    public boolean nextStepEntersNode(int vehicle, String node) {
        VehicleState vs = vehicles.get(vehicle);
        if (vs == null || vs.path == null || vs.path.isEmpty()) {
            return false;
        }
        int next = vs.confirmedIndex + 1;
        return next < vs.path.size() && vs.path.get(next).equals(node);
    }

    /** @return all vehicle states (live view; treat as read-only). */
    public Iterable<VehicleState> all() {
        return vehicles.values();
    }

    /** @return the number of vehicles currently IDLE at base. */
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