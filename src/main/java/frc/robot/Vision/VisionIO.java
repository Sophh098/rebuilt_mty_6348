// File: src/main/java/frc/SuperSubsystem/SuperVision/VisionIO.java
package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

    class VisionIOInputs {
        public boolean cameraConnected = false;

        /** Aiming helpers (radians). */
        public double latestTargetYawRadians = 0.0;
        public double latestTargetPitchRadians = 0.0;

        /** Pose estimator enabled inside the IO implementation. */
        public boolean photonPoseEstimatorEnabled = false;

        /** Observations (one per new frame that produced an estimate). */
        public double[] observationTimestampsSeconds = new double[0];
        public Pose3d[] observationRobotPoses = new Pose3d[0];
        public double[] observationAmbiguities = new double[0];
        public long[] observationTagCounts = new long[0];
        public double[] observationAverageTagDistanceMeters = new double[0];
        public boolean[] observationRotationTrusted = new boolean[0];
        public long[] observationTypeOrdinals = new long[0];

        /** Union of detected tag ids for the latest frame. */
        public long[] detectedTagIdentifiers = new long[0];

        public long framesPerSecond = 0;
        public boolean hasTarget = false;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void setDriverMode(boolean driverModeEnabled) {}

    /** Output to IO: helps reference-based strategies. */
    default void setReferencePoseForEstimation(Pose3d referencePose) {}
}