package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Wraps a {@link VisionIO} implementation and runs it on a dedicated daemon thread.
 *
 * <h3>Why this exists</h3>
 * PhotonVision reads go over NetworkTables. With 4 cameras, doing those reads
 * synchronously inside {@code periodic()} adds 4–8 ms to every robot loop —
 * enough to cause overtime. This class moves all camera I/O off the main thread.
 *
 * <h3>Thread safety</h3>
 * {@link VisionIO.VisionIOInputs} uses {@code volatile} fields so reads from the
 * main loop and writes from the camera thread are always consistent without locks.
 * The only shared mutable state is the {@code referencePose} volatile field on
 * {@link VisionIOPhotonVision}, which is a single-word write — always atomic on JVM.
 *
 * <h3>Usage</h3>
 * <pre>
 *   CameraThread cam = new CameraThread("FrontLeft", io, NoiseLevel.LOW, 1.0);
 *   cam.start();                          // call once at robot init
 *
 *   // In periodic() — fast volatile reads, no blocking:
 *   VisionIO.VisionIOInputs snapshot = cam.getInputs();
 *   cam.setReferencePose(currentPose3d);  // feed latest odometry
 * </pre>
 */
public final class CameraThread {

    private static final int    CAMERA_THREAD_HZ     = 20;   // iterations per second
    private static final long   LOOP_PERIOD_NS        = 1_000_000_000L / CAMERA_THREAD_HZ;
    private static final long   RESTART_DELAY_MS      = 2_000; // after unexpected crash

    // ── Identity ──────────────────────────────────────────────────────────
    private final String cameraName;

    // ── Hardware ──────────────────────────────────────────────────────────
    private final VisionIO                visionIO;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    // ── Camera configuration ──────────────────────────────────────────────
    private final VisionEnums.PoseEstimateNoiseLevel baseNoiseLevel;
    private final double                             cameraConfidenceMultiplier;

    // ── Background thread ─────────────────────────────────────────────────
    private final Thread thread;
    private volatile boolean running = false;

    public CameraThread(
            String cameraName,
            VisionIO visionIO,
            VisionEnums.PoseEstimateNoiseLevel baseNoiseLevel,
            double cameraConfidenceMultiplier) {

        this.cameraName                 = cameraName;
        this.visionIO                   = visionIO;
        this.baseNoiseLevel             = baseNoiseLevel;
        this.cameraConfidenceMultiplier = cameraConfidenceMultiplier;

        thread = new Thread(this::cameraLoop, "Vision-" + cameraName);
        thread.setDaemon(true);  // JVM won't wait for this thread to exit
        thread.setPriority(Thread.MIN_PRIORITY + 1); // below robot loop, above idle
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────

    /** Call once during robot initialization. */
    public void start() {
        running = true;
        thread.start();
    }

    /** Optional — call during robot shutdown. */
    public void stop() {
        running = false;
        thread.interrupt();
    }

    // ── Main-loop API (fast volatile reads) ──────────────────────────────

    /**
     * Returns the cached inputs snapshot. This is the object written by the
     * camera thread using volatile fields — reads here are always safe and fast.
     */
    public VisionIO.VisionIOInputs getInputs() {
        return inputs;
    }

    public String getCameraName() {
        return cameraName;
    }

    public VisionEnums.PoseEstimateNoiseLevel getBaseNoiseLevel() {
        return baseNoiseLevel;
    }

    public double getCameraConfidenceMultiplier() {
        return cameraConfidenceMultiplier;
    }

    /**
     * Feeds the latest robot pose to the camera thread so reference-based
     * estimation strategies stay accurate. Volatile write — always fast.
     */
    public void setReferencePose(Pose3d pose) {
        visionIO.setReferencePose(pose);
    }

    public void setDriverMode(boolean enabled) {
        visionIO.setDriverMode(enabled);
    }

    // ── Camera thread loop ────────────────────────────────────────────────

    private void cameraLoop() {
        while (running) {
            long loopStart = System.nanoTime();

            try {
                visionIO.updateInputs(inputs);
            } catch (Exception e) {
                // Log but never crash the thread — camera might reconnect
                DriverStation.reportWarning(
                        "[Vision/" + cameraName + "] updateInputs threw: " + e.getMessage(), false);

                try {
                    Thread.sleep(RESTART_DELAY_MS);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            // Fixed-rate sleep to maintain ~20 Hz
            long elapsed = System.nanoTime() - loopStart;
            long sleepNs  = LOOP_PERIOD_NS - elapsed;
            if (sleepNs > 0) {
                try {
                    Thread.sleep(sleepNs / 1_000_000L, (int)(sleepNs % 1_000_000L));
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }
}