// File: src/main/java/frc/robot/Vision/VisionSubsystem.java
package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldCosntants;

public class VisionSubsystem extends SubsystemBase {

  public interface VisionPoseMeasurementConsumer {
    void addVisionMeasurement(
        Pose2d visionRobotPose,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStandardDeviations
    );
  }

  public interface VisionHardwareFactory {
    VisionIO createVisionHardware(
        VisionEntries.CameraSpecifications cameraSpecifications,
        AprilTagFieldLayout aprilTagFieldLayout
    );
  }

  private static final VisionEnums.PoseObservationType[] poseObservationTypesByOrdinal =
      VisionEnums.PoseObservationType.values();

  private final List<LoggedVisionCamera> visionCameraList = new ArrayList<>();

  private final Supplier<Pose2d> currentRobotPoseSupplier;
  private final Supplier<Double> yawRateRadiansPerSecondSupplier;
  private final VisionPoseMeasurementConsumer visionPoseMeasurementConsumer;

  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final VisionStandardDeviationModel visionStandardDeviationModel;

  private final double fieldLengthMeters;
  private final double fieldWidthMeters;

  private final boolean[] isShootingTagAllowedByFiducialIdentifier;

  private boolean visionEnabled = true;
  private boolean shootingTargetValidThisCycle = false;

  public VisionSubsystem(
      AprilTagFieldLayout aprilTagFieldLayout,
      double fieldLengthMeters,
      double fieldWidthMeters,
      Supplier<Pose2d> currentRobotPoseSupplier,
      Supplier<Double> yawRateRadiansPerSecondSupplier,
      VisionPoseMeasurementConsumer visionPoseMeasurementConsumer,
      VisionStandardDeviationModel visionStandardDeviationModel,
      List<VisionEntries.CameraSpecifications> cameraSpecificationsList,
      VisionHardwareFactory visionHardwareFactory
  ) {
    this.aprilTagFieldLayout = aprilTagFieldLayout;
    this.fieldLengthMeters = fieldLengthMeters;
    this.fieldWidthMeters = fieldWidthMeters;
    this.currentRobotPoseSupplier = currentRobotPoseSupplier;
    this.yawRateRadiansPerSecondSupplier = yawRateRadiansPerSecondSupplier;
    this.visionPoseMeasurementConsumer = visionPoseMeasurementConsumer;
    this.visionStandardDeviationModel = visionStandardDeviationModel;

    long[] shootingTagIdentifiers = FieldCosntants.getShootingValidTagIdentifiers();
    this.isShootingTagAllowedByFiducialIdentifier = buildShootingTagLookupTable(shootingTagIdentifiers);

    for (VisionEntries.CameraSpecifications cameraSpecifications : cameraSpecificationsList) {
      VisionIO visionHardwareInterface =
          visionHardwareFactory.createVisionHardware(cameraSpecifications, aprilTagFieldLayout);

      visionCameraList.add(
          new LoggedVisionCamera(
              cameraSpecifications.cameraName(),
              visionHardwareInterface,
              cameraSpecifications.baseNoiseLevel(),
              cameraSpecifications.cameraConfidenceMultiplier()
          )
      );
    }
  }

  @Override
  public void periodic() {
    if (!visionEnabled || visionCameraList.isEmpty()) {
      shootingTargetValidThisCycle = false;
      return;
    }

    if (aprilTagFieldLayout == null) {
      shootingTargetValidThisCycle = false;
      return;
    }

    Pose2d currentRobotPose2d = currentRobotPoseSupplier.get();
    Pose3d referenceRobotPose3d = new Pose3d(currentRobotPose2d);

    double currentRobotTimestampSeconds = Timer.getFPGATimestamp();
    double yawRateRadiansPerSecond = yawRateRadiansPerSecondSupplier.get();

    boolean foundValidShootingTagThisCycle = false;

    for (LoggedVisionCamera visionCamera : visionCameraList) {
      visionCamera.update(referenceRobotPose3d);

      VisionIO.VisionIOInputs visionInputs = visionCamera.getVisionInputs();

      if (!foundValidShootingTagThisCycle) {
        foundValidShootingTagThisCycle =
            containsAnyAllowedShootingTag(
                visionInputs.detectedTagIdentifiers,
                isShootingTagAllowedByFiducialIdentifier
            );
      }

      if (!visionInputs.photonPoseEstimatorEnabled) {
        continue;
      }

      int observationSlotCount = visionInputs.observationTimestampsSeconds.length;

      for (int observationIndex = 0; observationIndex < observationSlotCount; observationIndex++) {
        int tagCount = (int) visionInputs.observationTagCounts[observationIndex];
        if (tagCount <= 0) {
          continue;
        }

        Pose3d robotPose = visionInputs.observationRobotPoses[observationIndex];
        if (robotPose == null) {
          continue;
        }

        int observationTypeOrdinal = (int) visionInputs.observationTypeOrdinals[observationIndex];
        int clampedOrdinal =
            Math.max(0, Math.min(observationTypeOrdinal, poseObservationTypesByOrdinal.length - 1));
        VisionEnums.PoseObservationType observationType = poseObservationTypesByOrdinal[clampedOrdinal];

        VisionEntries.VisionObservation visionObservation =
            new VisionEntries.VisionObservation(
                visionCamera.getCameraName(),
                visionInputs.observationTimestampsSeconds[observationIndex],
                robotPose,
                visionInputs.observationAmbiguities[observationIndex],
                tagCount,
                visionInputs.observationAverageTagDistanceMeters[observationIndex],
                visionInputs.observationRotationTrusted[observationIndex],
                observationType
            );

        VisionEnums.VisionRejectReason rejectReason =
            visionStandardDeviationModel.getRejectReasonOrNull(
                visionObservation,
                currentRobotTimestampSeconds,
                yawRateRadiansPerSecond,
                fieldLengthMeters,
                fieldWidthMeters
            );

        if (rejectReason != null) {
          continue;
        }

        Matrix<N3, N1> baseStandardDeviationMatrix =
            visionCamera.getBaseNoiseLevel().getBaseStandardDeviationMatrix();

        Matrix<N3, N1> measurementStandardDeviations =
            visionStandardDeviationModel.calculateStandardDeviations(
                visionObservation,
                baseStandardDeviationMatrix,
                visionCamera.getCameraConfidenceMultiplier()
            );

        visionPoseMeasurementConsumer.addVisionMeasurement(
            robotPose.toPose2d(),
            visionObservation.timestampSeconds(),
            measurementStandardDeviations
        );
      }
    }

    shootingTargetValidThisCycle = foundValidShootingTagThisCycle;
  }

  private static boolean[] buildShootingTagLookupTable(long[] shootingTagIdentifiers) {
    int maximumFiducialIdentifier = 0;

    if (shootingTagIdentifiers != null) {
      for (long fiducialIdentifier : shootingTagIdentifiers) {
        if (fiducialIdentifier >= 0 && fiducialIdentifier < Integer.MAX_VALUE) {
          maximumFiducialIdentifier = Math.max(maximumFiducialIdentifier, (int) fiducialIdentifier);
        }
      }
    }

    boolean[] lookupTable = new boolean[Math.max(1, maximumFiducialIdentifier + 1)];

    if (shootingTagIdentifiers != null) {
      for (long fiducialIdentifier : shootingTagIdentifiers) {
        if (fiducialIdentifier >= 0 && fiducialIdentifier < lookupTable.length) {
          lookupTable[(int) fiducialIdentifier] = true;
        }
      }
    }

    return lookupTable;
  }

  private static boolean containsAnyAllowedShootingTag(
      long[] detectedTagIdentifiers,
      boolean[] isShootingTagAllowedByFiducialIdentifier
  ) {
    if (detectedTagIdentifiers == null || isShootingTagAllowedByFiducialIdentifier == null) {
      return false;
    }

    for (long fiducialIdentifierLong : detectedTagIdentifiers) {
      if (fiducialIdentifierLong < 0) {
        break; // sentinel
      }

      if (fiducialIdentifierLong >= isShootingTagAllowedByFiducialIdentifier.length) {
        continue;
      }

      if (isShootingTagAllowedByFiducialIdentifier[(int) fiducialIdentifierLong]) {
        return true;
      }
    }

    return false;
  }

  public boolean hasTarget() {
    for (LoggedVisionCamera visionCamera : visionCameraList) {
      if (visionCamera.getVisionInputs().hasTarget) {
        return true;
      }
    }
    return false;
  }

  public double getLatestTargetYawRadians() {
    if (visionCameraList.isEmpty()) {
      return 0.0;
    }
    return visionCameraList.get(0).getVisionInputs().latestTargetYawRadians;
  }

  public void setVisionEnabled(boolean visionEnabled) {
    this.visionEnabled = visionEnabled;
  }

  public boolean itsAValidShootingTarget() {
    return shootingTargetValidThisCycle;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return aprilTagFieldLayout;
  }
}