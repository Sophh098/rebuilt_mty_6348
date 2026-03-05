package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/**
 * Creates the correct {@link VisionIO} implementation depending on whether we're
 * running on real hardware or in simulation.
 *
 * <p>For simulation, swap {@link VisionIOPhotonVision} for a
 * {@code VisionIOSim} implementation that uses WPILib's simulation framework.
 */
public final class VisionHardwareFactoryImpl implements VisionSubsystem.VisionHardwareFactory {

    private final boolean isSimulation;

    public VisionHardwareFactoryImpl(boolean isSimulation) {
        this.isSimulation = isSimulation;
    }

    @Override
    public VisionIO createVisionHardware(
            VisionEntries.CameraSpecifications specs,
            AprilTagFieldLayout fieldLayout) {

        if (isSimulation) {
            // TODO: return new VisionIOSim(specs.cameraName(), specs.robotToCameraTransform(), fieldLayout);
            // For now, fall through to real implementation (PhotonVision also supports sim)
        }

        return new VisionIOPhotonVision(
                specs.cameraName(),
                specs.robotToCameraTransform(),
                fieldLayout);
    }
}