// File: src/main/java/frc/SuperSubsystem/SuperVision/LoggedVisionCamera.java
package frc.robot.Vision;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose3d;

public final class LoggedVisionCamera {

    private final String cameraName;
    private final VisionIO visionHardwareInterface;

    private final VisionIO.VisionIOInputs visionInputs = new VisionIO.VisionIOInputs();

    private final VisionEnums.PoseEstimateNoiseLevel baseNoiseLevel;
    private final double cameraConfidenceMultiplier;

    public LoggedVisionCamera(
        String cameraName,
        VisionIO visionHardwareInterface,
        VisionEnums.PoseEstimateNoiseLevel baseNoiseLevel,
        double cameraConfidenceMultiplier
    ) {
        this.cameraName = Objects.requireNonNull(cameraName);
        this.visionHardwareInterface = Objects.requireNonNull(visionHardwareInterface);
        this.baseNoiseLevel = Objects.requireNonNull(baseNoiseLevel);
        this.cameraConfidenceMultiplier = cameraConfidenceMultiplier;
    }

    public void update(Pose3d referencePoseForEstimation) {
        visionHardwareInterface.setReferencePoseForEstimation(referencePoseForEstimation);
        visionHardwareInterface.updateInputs(visionInputs);
    }

    public VisionIO.VisionIOInputs getVisionInputs() {
        return visionInputs;
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
}