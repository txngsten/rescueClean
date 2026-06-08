package solution;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * Simple stat tracking class for figuring out how many routes are successful.
 */
public final class MissionStats {

    private final AtomicInteger outboundAttempts = new AtomicInteger(0);
    private final AtomicInteger outboundSuccess  = new AtomicInteger(0);
    private final AtomicInteger returnAttempts   = new AtomicInteger(0);
    private final AtomicInteger returnSuccess    = new AtomicInteger(0);

    public void recordOutboundAttempt() {
        outboundAttempts.incrementAndGet();
    }

    public void recordOutboundSuccess() {
        outboundSuccess.incrementAndGet();
    }

    public void recordReturnAttempt() {
        returnAttempts.incrementAndGet();
    }

    public void recordReturnSuccess() {
        returnSuccess.incrementAndGet();
    }

    public void printSummary() {
        int obTotal = outboundAttempts.get();
        int obOk    = outboundSuccess.get();
        int obBad   = obTotal - obOk;

        int rtTotal = returnAttempts.get();
        int rtOk    = returnSuccess.get();
        int rtBad   = rtTotal - rtOk;

        System.out.println("=== MISSION STATS ===");
        System.out.println("Base -> Building");
        System.out.println("  total           : " + obTotal);
        System.out.println("  successful      : " + obOk);
        System.out.println("  unsuccessful    : " + obBad);
        System.out.println("  ratio (ok/bad)  : " + ratio(obOk, obBad));
        System.out.println("Building -> Base");
        System.out.println("  total           : " + rtTotal);
        System.out.println("  successful      : " + rtOk);
        System.out.println("  unsuccessful    : " + rtBad);
        System.out.println("  ratio (ok/bad)  : " + ratio(rtOk, rtBad));
        System.out.println("=====================");
    }

    private static String ratio(int ok, int bad) {
        if (bad == 0) {
            return (ok == 0) ? "n/a (0/0)" : "n/a (no failures)";
        }
        return String.format("%.2f", (double) ok / bad);
    }
}