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

    /** Observations (reused buffers). */
    public int observationCount = 0;
    public final double[] observationTimestampsSeconds = new double[1];
    public final Pose3d[] observationRobotPoses = new Pose3d[1];
    public final double[] observationAmbiguities = new double[1];
    public final long[] observationTagCounts = new long[1];
    public final double[] observationAverageTagDistanceMeters = new double[1];
    public final boolean[] observationRotationTrusted = new boolean[1];
    public final long[] observationTypeOrdinals = new long[1];

    /** Detected tag identifiers for newest frame (reused buffer). */
    public int detectedTagIdentifierCount = 0;
    public final long[] detectedTagIdentifiers = new long[64];

    public long framesPerSecond = 0;
    public boolean hasTarget = false;
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void setDriverMode(boolean driverModeEnabled) {}

  /** Output to IO: helps reference-based strategies (optional). */
  default void setReferencePoseForEstimation(Pose3d referencePose) {}
}