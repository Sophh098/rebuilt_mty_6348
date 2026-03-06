package frc.robot.Drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Shooting.ShootingHelper;
import frc.robot.Vision.VisionSubsystem;

public final class DriveCommands {

  private DriveCommands() {}

  public static Command joystickDriveWithVisionAim(
      CommandSwerveDrivetrain drivetrain,
      SwerveRequest.FieldCentric fieldCentricDriveRequest,
      VisionSubsystem visionSubsystem,
      ShootingHelper shootingHelper,
      BooleanSupplier isAutoAimActiveSupplier,
      DoubleSupplier translationXInputSupplier,
      DoubleSupplier translationYInputSupplier,
      DoubleSupplier rotationInputSupplier,
      double maximumSpeedMetersPerSecond,
      double maximumAngularRateRadiansPerSecond) {

    ProfiledPIDController headingProfiledController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                maximumAngularRateRadiansPerSecond,
                maximumAngularRateRadiansPerSecond * 2.0));

    headingProfiledController.enableContinuousInput(-Math.PI, Math.PI);

    final boolean[] wasAutoAimActiveInPreviousLoop = new boolean[] {false};

    return drivetrain
        .applyRequest(
            () -> {
              SwerveDrivetrain.SwerveDriveState drivetrainState = drivetrain.getState();
              Pose2d robotPose = drivetrainState.Pose;

              // CTRE: Speeds is robot-centric. Convert to field-relative for your helper (like your old code).
              ChassisSpeeds robotRelativeSpeeds = drivetrainState.Speeds;
              ChassisSpeeds fieldRelativeSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

              shootingHelper.update(robotPose, fieldRelativeSpeeds);

              Translation2d normalizedLinearVelocity =
                  getNormalizedLinearVelocityFromJoysticks(
                      translationXInputSupplier.getAsDouble(),
                      translationYInputSupplier.getAsDouble());

              boolean isAutoAimButtonPressed = isAutoAimActiveSupplier.getAsBoolean();
              boolean isPossibleToShoot = shootingHelper.isPossibleToShoot();
              boolean isAValidShootingTarget = visionSubsystem.isShootingTargetValid();

              boolean shouldAutoAim = isAutoAimButtonPressed && isPossibleToShoot && isAValidShootingTarget;

              double requestedAngularRateRadiansPerSecond;

              double currentHeadingRadians = robotPose.getRotation().getRadians();

              if (shouldAutoAim) {
                double desiredHeadingRadians = shootingHelper.getDesiredChassisHeadingRadians();

                if (!wasAutoAimActiveInPreviousLoop[0]) {
                  headingProfiledController.reset(currentHeadingRadians);
                }

                requestedAngularRateRadiansPerSecond =
                    headingProfiledController.calculate(currentHeadingRadians, desiredHeadingRadians);

              } else {
                double manualRotationInput =
                    MathUtil.applyDeadband(rotationInputSupplier.getAsDouble(), DriveConstants.DEADBAND);

                manualRotationInput =
                    Math.copySign(manualRotationInput * manualRotationInput, manualRotationInput);

                requestedAngularRateRadiansPerSecond = manualRotationInput * maximumAngularRateRadiansPerSecond;

                if (wasAutoAimActiveInPreviousLoop[0]) {
                  headingProfiledController.reset(currentHeadingRadians);
                }
              }

              wasAutoAimActiveInPreviousLoop[0] = shouldAutoAim;

              return fieldCentricDriveRequest
                  .withVelocityX(normalizedLinearVelocity.getX() * maximumSpeedMetersPerSecond)
                  .withVelocityY(normalizedLinearVelocity.getY() * maximumSpeedMetersPerSecond)
                  .withRotationalRate(requestedAngularRateRadiansPerSecond);
            })
        .beforeStarting(() -> headingProfiledController.reset(drivetrain.getState().Pose.getRotation().getRadians()));
  }

  private static Translation2d getNormalizedLinearVelocityFromJoysticks(
      double rawTranslationXInput,
      double rawTranslationYInput) {

    double translationXAfterDeadband = MathUtil.applyDeadband(rawTranslationXInput, DriveConstants.DEADBAND);
    double translationYAfterDeadband = MathUtil.applyDeadband(rawTranslationYInput, DriveConstants.DEADBAND);

    translationXAfterDeadband =
        Math.copySign(translationXAfterDeadband * translationXAfterDeadband, translationXAfterDeadband);
    translationYAfterDeadband =
        Math.copySign(translationYAfterDeadband * translationYAfterDeadband, translationYAfterDeadband);

    double magnitude = Math.hypot(translationXAfterDeadband, translationYAfterDeadband);
    if (magnitude > 1.0) {
      translationXAfterDeadband /= magnitude;
      translationYAfterDeadband /= magnitude;
    }

    return new Translation2d(translationXAfterDeadband, translationYAfterDeadband);
  }
}