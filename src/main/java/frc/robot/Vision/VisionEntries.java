package frc.robot.Vision;

import java.util.Objects;

import edu.wpi.first.math.geometry.Transform3d;

/** Immutable data records used by the Vision subsystem. */
public final class VisionEntries {
    private VisionEntries() {}

    /**
     * All configuration needed to instantiate one camera.
     *
     * @param cameraName                 NetworkTables name of the PhotonCamera.
     * @param robotToCameraTransform     Transform from robot origin to camera.
     *                                   <b>Must be ROBOT → CAMERA</b> (PhotonLib convention).
     * @param baseNoiseLevel             Base noise preset for this camera.
     * @param cameraConfidenceMultiplier Multiplier applied to computed std devs.
     *                                   Values > 1.0 make the odometry trust this camera less.
     */
    public record CameraSpecifications(
            String cameraName,
            Transform3d robotToCameraTransform,
            VisionEnums.PoseEstimateNoiseLevel baseNoiseLevel,
            double cameraConfidenceMultiplier
    ) {
        public CameraSpecifications {
            Objects.requireNonNull(cameraName,              "cameraName must not be null");
            Objects.requireNonNull(robotToCameraTransform,  "robotToCameraTransform must not be null");
            Objects.requireNonNull(baseNoiseLevel,          "baseNoiseLevel must not be null");
            if (cameraConfidenceMultiplier <= 0.0)
                throw new IllegalArgumentException("cameraConfidenceMultiplier must be > 0");
        }
    }
}