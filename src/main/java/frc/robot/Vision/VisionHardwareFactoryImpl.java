// File: src/main/java/frc/robot/Vision/VisionHardwareFactoryImpl.java
package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public final class VisionHardwareFactoryImpl implements VisionSubsystem.VisionHardwareFactory {

    @SuppressWarnings("unused")
    private final boolean isSimulation;

    public VisionHardwareFactoryImpl(boolean isSimulation) {
        this.isSimulation = isSimulation;
    }

    @Override
    public VisionIO createVisionHardware(
        VisionEntries.CameraSpecifications cameraSpecifications,
        AprilTagFieldLayout aprilTagFieldLayout
    ) {
        return new VisionIOPhotonVision(
            cameraSpecifications.cameraName(),
            cameraSpecifications.getRobotToCameraTransform3d(),
            aprilTagFieldLayout
        );
    }
}