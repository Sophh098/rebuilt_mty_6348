package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Hardware-abstraction layer for one camera.
 * Kept intentionally minimal — all heavy work happens in the camera thread,
 * not in the main robot loop.
 */
public interface VisionIO {

    /**
     * Snapshot produced once per camera-thread iteration (~20 Hz).
     * Written by the camera thread, read by the main loop.
     * All primitives — zero heap allocation on reads.
     */
    class VisionIOInputs {
        // ── Connection ──────────────────────────────────────────────
        public volatile boolean cameraConnected = false;

        // ── Latest aiming data (best target of newest frame) ────────
        public volatile boolean  hasTarget               = false;
        public volatile double   latestTargetYawRadians  = 0.0;
        public volatile double   latestTargetPitchRadians = 0.0;

        // ── Pose observation (at most 1 per camera per cycle) ───────
        // tagCount == 0  →  no valid observation this cycle
        public volatile int    observationTagCount                = 0;
        public volatile double observationTimestampSeconds        = 0.0;
        public volatile Pose3d observationRobotPose               = null;
        public volatile double observationAmbiguity               = 0.0;
        public volatile double observationAverageTagDistanceMeters = 0.0;
        public volatile boolean observationRotationTrusted        = false;
        public volatile boolean observationIsMultiTag             = false;

        // ── Detected tag IDs (sentinel = -1, max 16 tags) ──────────
        public final long[] detectedTagIdentifiers = new long[16];
        public volatile int  detectedTagCount       = 0;

        // ── Diagnostics ─────────────────────────────────────────────
        public volatile long framesPerSecond = 0;
    }

    /** Called once per camera-thread tick to refresh inputs. */
    default void updateInputs(VisionIOInputs inputs) {}

    default void setDriverMode(boolean enabled) {}

    /** Feed current robot pose so reference-based strategies can use it. */
    default void setReferencePose(Pose3d referencePose) {}
}