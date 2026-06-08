package solution;

import sim.Message;

import java.util.concurrent.*;

/**
 * Base class for all disaster response strategies. Runs a dedicated comms thread
 * that receives messages from the Simulator via a blocking queue and dispatches
 * them to the subclass's handle() method. The comms thread is separate from the
 * executor so that message receipt never blocks on path computation or sends.
 */
public abstract class DisasterResponder  {
    protected final Thread commsThread;
    protected String configFile;
    protected final ExecutorService executor;

    protected BlockingQueue<Message> inMessageQueue;
    protected BlockingQueue<Message> outMessageQueue;

    public DisasterResponder() {
        inMessageQueue = new LinkedBlockingQueue<Message>();
        this.executor = Executors.newSingleThreadExecutor();
        this.commsThread = new Thread(this::commsLoop, "DisasterResponder-DispatchThread");
        this.commsThread.setUncaughtExceptionHandler((t, e) -> {
            System.err.println("Exception in thread " + t.getName() + ": " + e.getMessage());
        });
    }

    abstract protected void handle(Message s);

    abstract protected void setup() ;

    public final void start(String configFile) {
        this.configFile = configFile;
        this.commsThread.start();
        System.out.println("RESPONDER SETUP BEGINS");
        setup();
        System.out.println("RESPONDER SETUP FINISHES");
    }

    private void commsLoop() {
        while (true) {
            try {
                Message m = inMessageQueue.take();
                if (m == Message.SHUTDOWN) {
                    shutdown();
                    break;
                }
                try {
                    handle(m);
                } catch (Exception e) {
                    System.err.println("Error in message handling code:");
                    e.printStackTrace();
                }
            } catch (InterruptedException ex) {
                throw new RuntimeException(ex);
            }
        }
        System.out.println("RESPONDER COMMS LOOP HAS TERMINATED");
    }

    public void setOutMessageQueue(BlockingQueue queue) {
        outMessageQueue = queue;
    }

    public BlockingQueue getInMessageQueue() {
        return inMessageQueue;
    }

    public void shutdown() {
        executor.shutdown();
        try {
            if (!executor.awaitTermination(5, TimeUnit.SECONDS)) {
                executor.shutdownNow();
            }
        } catch (InterruptedException e) {
            executor.shutdownNow();
        }
        System.out.println("RESPONDER EXECUTOR HAS TERMINATED");
    }

}
