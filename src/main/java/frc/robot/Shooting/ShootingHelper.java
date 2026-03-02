package frc.robot.Shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShootingConstants;

public final class ShootingHelper {

    private double hoodAngleRadians;
    private double horizontalVelocityMetersPerSecond;
    private final double verticalVelocityMetersPerSecond;
    private double exitSpeedMetersPerSecond;
    private double desiredChassisHeadingRadians;
    private double distanceToDigitalTargetMeters;

    private double digitalTargetXFieldMeters;
    private double digitalTargetYFieldMeters;

    private boolean isPossibleToShoot;

    private final boolean isAndymarkTarget;

    public ShootingHelper(boolean isAndymarkTarget) {
        this.isAndymarkTarget = isAndymarkTarget;
        this.verticalVelocityMetersPerSecond =
            ShootingConstants.VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND;
    }

    public void update(Pose2d robotPoseField, ChassisSpeeds fieldSpeeds) {

        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        double shooterXFieldMeters = robotPoseField.getX();
        double shooterYFieldMeters = robotPoseField.getY();

        double targetXFieldMeters = getTargetXFieldMeters(currentAlliance);
        double targetYFieldMeters = getTargetYFieldMeters(currentAlliance);

        digitalTargetXFieldMeters =
            targetXFieldMeters - fieldSpeeds.vxMetersPerSecond * ShootingConstants.TOTAL_TIME_SECONDS;

        digitalTargetYFieldMeters =
            targetYFieldMeters - fieldSpeeds.vyMetersPerSecond * ShootingConstants.TOTAL_TIME_SECONDS;

        double deltaXFieldMeters = digitalTargetXFieldMeters - shooterXFieldMeters;
        double deltaYFieldMeters = digitalTargetYFieldMeters - shooterYFieldMeters;

        distanceToDigitalTargetMeters = Math.hypot(deltaXFieldMeters, deltaYFieldMeters);

        if (distanceToDigitalTargetMeters < ShootingConstants.MINIMUM_HORIZONTAL_DISTANCE_METERS) {
            isPossibleToShoot = false;
            return;
        }

        horizontalVelocityMetersPerSecond =
            distanceToDigitalTargetMeters / ShootingConstants.TOTAL_TIME_SECONDS;

        hoodAngleRadians = Math.atan2(
            verticalVelocityMetersPerSecond,
            horizontalVelocityMetersPerSecond
        );

        exitSpeedMetersPerSecond = Math.hypot(
            horizontalVelocityMetersPerSecond,
            verticalVelocityMetersPerSecond
        );

        desiredChassisHeadingRadians = Math.atan2(deltaYFieldMeters, deltaXFieldMeters);

        isPossibleToShoot =
            exitSpeedMetersPerSecond <= ShootingConstants.MAXIMUM_EXIT_SPEED_METERS_PER_SECOND &&
            hoodAngleRadians >= ShootingConstants.MINIMUM_HOOD_ANGLE_RADIANS &&
            hoodAngleRadians <= ShootingConstants.MAXIMUM_HOOD_ANGLE_RADIANS;
    }

    private double getTargetXFieldMeters(Alliance currentAlliance) {
        boolean isRedAlliance = currentAlliance == Alliance.Red;

        if (isAndymarkTarget) {
            return isRedAlliance
                ? ShootingConstants.FUNNEL_POSITION_RED_ALLIANCE_ANDYMARK_X_METERS
                : ShootingConstants.FUNNEL_POSITION_BLUE_ALLIANCE_ANDYMARK_X_METERS;
        }

        return isRedAlliance
            ? ShootingConstants.FUNNEL_POSITION_RED_ALLIANCE_WELDED_X_METERS
            : ShootingConstants.FUNNEL_POSITION_BLUE_ALLIANCE_WELDED_X_METERS;
    }

    private double getTargetYFieldMeters(Alliance currentAlliance) {
        boolean isRedAlliance = currentAlliance == Alliance.Red;

        if (isAndymarkTarget) {
            return isRedAlliance
                ? ShootingConstants.FUNNEL_POSITION_RED_ALLIANCE_ANDYMARK_Y_METERS
                : ShootingConstants.FUNNEL_POSITION_BLUE_ALLIANCE_ANDYMARK_Y_METERS;
        }

        return isRedAlliance
            ? ShootingConstants.FUNNEL_POSITION_RED_ALLIANCE_WELDED_Y_METERS
            : ShootingConstants.FUNNEL_POSITION_BLUE_ALLIANCE_WELDED_Y_METERS;
    }

    public double getHoodAngleRadians() {
        return hoodAngleRadians;
    }

    public double getExitSpeedMetersPerSecond() {
        return exitSpeedMetersPerSecond;
    }

    public double getDesiredChassisHeadingRadians() {
        return desiredChassisHeadingRadians;
    }

    public boolean isPossibleToShoot() {
        return isPossibleToShoot;
    }

    public double getDistanceToDigitalTargetMeters() {
        return distanceToDigitalTargetMeters;
    }
}