package solution;

/*
 * D* Lite for both outbound and return. Incremental: reuses search state across
 * road blocks/clears and collapses instead of replanning from scratch, so it
 * should hold up best as road damage rises. This is the same behaviour as the
 * MyDisasterResponder default, named explicitly so it can be selected via
 * RESPONDER_CLASS alongside the other solutions for comparison.
 */
public class DStarResponder extends MyDisasterResponder {

    @Override
    protected Algo outboundAlgo() {
        return Algo.DSTAR;
    }

    @Override
    protected Algo returnAlgo() {
        return Algo.DSTAR;
    }
}