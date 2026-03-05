package frc.robot;

import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Vision.VisionEntries;
import frc.robot.Vision.VisionEnums;

public class Constants {

    public static final double BATTERY_VOLTAGE = 12.0;

    public static final class DriveConstants {
        public static final Mode SIM_MODE = Mode.SIM;
        public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

        public static enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }

        // -----------------------
        // Updated with your values
        // -----------------------

        // Robot mass
        public static final double ROBOT_MASS_KG = 60.0;

        // Robot footprint: 27.5" x 27.5" (0.6985 m x 0.6985 m)
        // MOI_z ≈ (1/12) * m * (w^2 + l^2)
        // With m=60 kg, w=l=0.6985 m => MOI_z = 4.8790225 kg*m^2
        public static final double ROBOT_MOI = 4.8790225;

        // Coefficient of friction (given)
        public static final double WHEEL_COF = 1.2;

        public static final double STEER_GAINS_KP = 100;
        public static final double STEER_GAINS_KI = 0.0;
        public static final double STEER_GAINS_KD = 0.5;
        public static final double STEER_GAINS_KS = 0.1;
        public static final double STEER_GAINS_KV = 1.91;
        public static final double STEER_GAINS_KA = 0.0;

        public static final double DRIVE_GAINS_KP = 0.1;
        public static final double DRIVE_GAINS_KI = 0.0;
        public static final double DRIVE_GAINS_KD = 0.0;
        public static final double DRIVE_GAINS_KS = 0.0;
        public static final double DRIVE_GAINS_KV = 0.124;
        public static final double DRIVE_GAINS_KA = 0.0;

        public static final Current kSlipCurrent = Amps.of(60.0);
        public static final Current SteeringStatorCurrentLimit = Amps.of(60.0);

        public static final String CAN_BUS = "6348 Horus CANivore";
        public static final String HOOT_FILE_PATH = "./logs/example.hoot";

        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(3.79);

        public static final double K_COUPLE_RATIO = 3.5714285714285716;
        public static final double K_DRIVE_GEAR_RATIO = 8.142857142857142;
        public static final double K_STEER_GEAR_RATIO = 21.428571428571427;
        public static final Distance kWheelRadius = Inches.of(2.0);

        public static final boolean K_INVERT_LEFT_SIDE = false;
        public static final boolean K_INVERT_RIGHT_SIDE = true;
        
        public static final int K_PIGEON_ID = 13;

        // -----------------------
        // Simulation-only values
        // -----------------------

        // kDriveInertia: wheel spin inertia about axle (NOT steer).
        // Each wheel mass is 0.73 lb = 0.33112243 kg
        // Wheel radius is 2.167 in = 0.0550418 m
        // Solid disk approximation: I = 0.5 * m * r^2 = 0.000501584 kg*m^2
        public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

        // kSteerInertia: azimuth rotational inertia of the entire steering mechanism.
        // If this is too tiny, MapleSim can get numerically unstable. Use a conservative value.
        public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);

        // Simulated voltage necessary to overcome friction
        public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final Distance kTrackWidth = Inches.of(23.5);
        public static final Distance kWheelBase = Inches.of(23.5);
        // Front Left
        public static final int K_FRONT_LEFT_DRIVE_MOTOR_ID = 10;
        public static final int K_FRONT_LEFT_STEER_MOTOR_ID = 6;
        public static final int K_FRONT_LEFT_ENCODER_ID = 2;
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.27197265625);
        public static final boolean K_FRONT_LEFT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_FRONT_LEFT_ENCODER_INVERTED = false;

        public static final Distance kFrontLeftXPos = kWheelBase.div(2);
        public static final Distance kFrontLeftYPos = kTrackWidth.div(2);


        // Front Right
        public static final int K_FRONT_RIGHT_DRIVE_MOTOR_ID = 9;
        public static final int K_FRONT_RIGHT_STEER_MOTOR_ID = 5;
        public static final int K_FRONT_RIGHT_ENCODER_ID = 1;
        public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.30810546875);
        public static final boolean K_FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_FRONT_RIGHT_ENCODER_INVERTED = false;

       public static final Distance kFrontRightXPos = kWheelBase.div(2);
        public static final Distance kFrontRightYPos = kTrackWidth.div(-2);
        //hay que modificcar algo extra ahorita para probar sin cámaras?

        // Back Left
        public static final int K_BACK_LEFT_DRIVE_MOTOR_ID = 11;
        public static final int K_BACK_LEFT_STEER_MOTOR_ID = 7;
        public static final int K_BACK_LEFT_ENCODER_ID = 3;
        public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.41455078125);
        public static final boolean K_BACK_LEFT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_BACK_LEFT_ENCODER_INVERTED = false;

        public static final Distance kBackLeftXPos = kWheelBase.div(-2);
        public static final Distance kBackLeftYPos = kTrackWidth.div(2);

        // Back Right
        public static final int K_BACK_RIGHT_DRIVE_MOTOR_ID = 12;
        public static final int K_BACK_RIGHT_STEER_MOTOR_ID = 8;
        public static final int K_BACK_RIGHT_ENCODER_ID = 4;
        public static final Angle kBackRightEncoderOffset = Rotations.of(-0.14013671875);
        public static final boolean K_BACK_RIGHT_STEER_MOTOR_INVERTED = true;
        public static final boolean K_BACK_RIGHT_ENCODER_INVERTED = false;

        public static final Distance kBackRightXPos = kWheelBase.div(-2);
        public static final Distance kBackRightYPos = kTrackWidth.div(-2);

        public static final double DEADBAND = 0.1;
        public static final double ANGLE_KP = 5.0;
        public static final double ANGLE_KD = 0.4;
        public static final double ANGLE_MAX_VELOCITY = 8.0;
        public static final double ANGLE_MAX_ACCELERATION = 20.0;
        public static final double FF_START_DELAY = 2.0; // Secs
        public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
        public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
        public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    }

    public static final class AutoConstants {
        public static final double TRANSLATION_KP = 5.0;
        public static final double TRANSLATION_KI = 0.0;
        public static final double TRANSLATION_KD = 0.0;

        public static final double ROTATION_KP = 5.0;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;
    }

    public static final class ShootingConstants {

        public static final double PASSING_SHOOT_RIGHT_RED_ALLIANCE_WELDED_X_METERS = 14.228191;
        public static final double PASSING_SHOOT_RIGHT_RED_ALLIANCE_WELDED_Y_METERS = 1.344887667;

        public static final double PASSING_SHOOT_RIGHT_BLUE_ALLIANCE_WELDED_X_METERS = 2.312797;
        public static final double PASSING_SHOOT_RIGHT_BLUE_ALLIANCE_WELDED_Y_METERS = 1.344887667;

        public static final double PASSING_SHOOT_RIGHT_RED_ALLIANCE_ANDYMARK_X_METERS = 14.207236;
        public static final double PASSING_SHOOT_RIGHT_RED_ALLIANCE_ANDYMARK_Y_METERS = 1.340442667;

        public static final double PASSING_SHOOT_RIGHT_BLUE_ALLIANCE_ANDYMARK_X_METERS = 2.305812;
        public static final double PASSING_SHOOT_RIGHT_BLUE_ALLIANCE_ANDYMARK_Y_METERS = 1.340442667;

        public static final double PASSING_SHOOT_LEFT_RED_ALLIANCE_WELDED_X_METERS = 14.228191;
        public static final double PASSING_SHOOT_LEFT_RED_ALLIANCE_WELDED_Y_METERS = 6.724438333;

        public static final double PASSING_SHOOT_LEFT_BLUE_ALLIANCE_WELDED_X_METERS = 2.312797;
        public static final double PASSING_SHOOT_LEFT_BLUE_ALLIANCE_WELDED_Y_METERS = 6.724438333;
        
        public static final double PASSING_SHOOT_LEFT_RED_ALLIANCE_ANDYMARK_X_METERS = 14.207236;
        public static final double PASSING_SHOOT_LEFT_RED_ALLIANCE_ANDYMARK_Y_METERS = 6.702213333;

        public static final double PASSING_SHOOT_LEFT_BLUE_ALLIANCE_ANDYMARK_X_METERS = 2.305812;
        public static final double PASSING_SHOOT_LEFT_BLUE_ALLIANCE_ANDYMARK_Y_METERS = 6.702213333;
        
        public static final double FUNNEL_POSITION_RED_ALLIANCE_WELDED_X_METERS = 11.915394;
        public static final double FUNNEL_POSITION_RED_ALLIANCE_WELDED_Y_METERS = 4.034663;

        public static final double FUNNEL_POSITION_BLUE_ALLIANCE_WELDED_X_METERS = 4.625594;
        public static final double FUNNEL_POSITION_BLUE_ALLIANCE_WELDED_Y_METERS = 4.034663;

        public static final double FUNNEL_POSITION_RED_ALLIANCE_ANDYMARK_X_METERS = 11.901424;
        public static final double FUNNEL_POSITION_RED_ALLIANCE_ANDYMARK_Y_METERS = 4.034663;

        public static final double FUNNEL_POSITION_BLUE_ALLIANCE_ANDYMARK_X_METERS = 4.611624;
        public static final double FUNNEL_POSITION_BLUE_ALLIANCE_ANDYMARK_Y_METERS = 4.034663;

        public static final double FUNNEL_HEIGHT_METERS = 1.8288;
        public static final double SHOOTER_EXIT_HEIGHT_METERS = 22 * 0.0254;

        public static final double GRAVITY_METERS_PER_SECOND_SQUARED = 9.81;
        public static final double APEX_MAX_HEIGHT_METERS = 2.4;

        public static final double APEX_RELATIVE_TO_SHOOTER_METERS =
            APEX_MAX_HEIGHT_METERS - SHOOTER_EXIT_HEIGHT_METERS;

        public static final double APEX_TO_FUNNEL_METERS =
            APEX_MAX_HEIGHT_METERS - FUNNEL_HEIGHT_METERS;

        public static final double VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND =
            Math.sqrt(2.0 * GRAVITY_METERS_PER_SECOND_SQUARED * APEX_RELATIVE_TO_SHOOTER_METERS);

        public static final double TIME_GOING_UP_SECONDS =
            VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND / GRAVITY_METERS_PER_SECOND_SQUARED;

        public static final double TIME_GOING_DOWN_SECONDS =
            Math.sqrt(2.0 * APEX_TO_FUNNEL_METERS / GRAVITY_METERS_PER_SECOND_SQUARED);

        public static final double TOTAL_TIME_SECONDS =
            TIME_GOING_UP_SECONDS + TIME_GOING_DOWN_SECONDS;

        public static final double DEFAULT_HORIZONTAL_DISTANCE_METERS = 2.0;

        public static final double DEFAULT_HORIZONTAL_VELOCITY_METERS_PER_SECOND =
            DEFAULT_HORIZONTAL_DISTANCE_METERS / TOTAL_TIME_SECONDS;

        // MODEL angle (same definition as ShootingHelper.getHoodAngleRadians()).
        public static final double DEFAULT_HOOD_ANGLE_RADIANS =
            Math.atan2(
                VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND,
                DEFAULT_HORIZONTAL_VELOCITY_METERS_PER_SECOND
            );

        // Keep alias to avoid breaking other code paths; name is misleading.
        public static final double DEFAULT_HOOD_ANGLE_ROT = DEFAULT_HOOD_ANGLE_RADIANS;

        public static final double DEFAULT_EXIT_SPEED_METERS_PER_SECOND =
            Math.hypot(
                DEFAULT_HORIZONTAL_VELOCITY_METERS_PER_SECOND,
                VERTICAL_LAUNCH_VELOCITY_METERS_PER_SECOND
            );

        public static final double MAX_RPM = 6000.0;
        public static final double WHEEL_DIAMETER_METERS = 4.0 * 0.0254;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double EXIT_SPEED_CONVERSION = WHEEL_CIRCUMFERENCE / 60.0;

        public static final double MAXIMUM_EXIT_SPEED_METERS_PER_SECOND =
            MAX_RPM * EXIT_SPEED_CONVERSION * 0.60;

        public static final double MINIMUM_HOOD_ANGLE_RADIANS = Math.toRadians(24);
        public static final double MAXIMUM_HOOD_ANGLE_RADIANS = Math.toRadians(98);

        public static final double MINIMUM_HORIZONTAL_DISTANCE_METERS = 0.1;
    }

    public static final class VisionConstants {

        public static final List<VisionEntries.CameraSpecifications> cameraSpecificationsList =
            List.of(
                new VisionEntries.CameraSpecifications(
                    "FRONTRIGHT_CAM",
                    new Transform3d(
                        new Translation3d(0.32, -0.32, 0.1778),
                        new Rotation3d(0, 0, 0)
                    ),
                    VisionEnums.PoseEstimateNoiseLevel.MEDIUM,
                    1.0
                ),
                new VisionEntries.CameraSpecifications(
                    "FRONT_LEFT_CAM",
                    new Transform3d(
                        new Translation3d(0.32, 0.32, 0.1778),
                        new Rotation3d(0, 0, 0)
                    ),
                    VisionEnums.PoseEstimateNoiseLevel.HIGH,
                    0.8
                ),
                new VisionEntries.CameraSpecifications(
                    "LEFT_CAM",
                    new Transform3d(
                        new Translation3d(0.32, 0.32, 0.1778),
                        new Rotation3d(0, 0, Math.toRadians(90))
                    ),
                    VisionEnums.PoseEstimateNoiseLevel.HIGH,
                    0.8
                ),
                new VisionEntries.CameraSpecifications(
                    "RIGHT_CAM",
                    new Transform3d(
                        new Translation3d(0.32, -0.32, 0.1778),
                        new Rotation3d(0, 0, Math.toRadians(-90))
                    ),
                    VisionEnums.PoseEstimateNoiseLevel.HIGH,
                    0.8
                )
            );

        public static final double MAXIMUM_AMBIGUITY_FOR_SINGLE_TAG = 0.20;
        public static final double MAXIMUM_Z_ERROR_METERS = 0.25;
        public static final double MAXIMUM_OBSERVATION_AGE_SECONDS = 0.35;

        public static final double MAXIMUM_DISTANCE_FOR_SINGLE_TAG_METERS = 4.0;
        public static final double MAXIMUM_DISTANCE_FOR_MULTI_TAG_METERS = 6.0;
        public static final double MAXIMUM_YAW_RATE_RADIANS_PER_SECOND = 4.0;

        public static final double MAXIMUM_LINEAR_STANDARD_DEVIATION_METERS = 2.0;
        public static final double MAXIMUM_ANGULAR_STANDARD_DEVIATION_RADIANS = 3.0;
    }

    public static final class FieldCosntants {

        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final double FIELD_LENGTH_METERS = 16.54175;
        public static final double FIELD_WIDTH_METERS = 8.0137;

        public static final boolean IS_ANDYMARK_FIELD = true;

        public static long[] getShootingValidTagIdentifiers() {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
            return alliance == Alliance.Red
                ? new long[] {2, 5, 8, 9, 10, 11}
                : new long[] {18, 21, 24, 25, 26, 27};
        }
    }

    public static final class HoodConstants {

        public static final int HOOD_ANGLE_TALON_ID = 15;
        public static final int RIGHT_HOOD_PROPULSION_TALON_ID = 16;
        public static final int LEFT_HOOD_PROPULSION_TALON_ID = 17;
        public static final int INDEXER_TALON_ID = 22;

        public static final Slot0Configs HOOD_ANGLE_SLOT_CONFIGS = new Slot0Configs()
            .withKS(0.1)
            .withKG(0.0)
            .withKV(0.12)
            .withKA(0.0)
            .withKP(0.11)
            .withKI(0.0)
            .withKD(0.0);

        public static final Slot0Configs RIGHT_HOOD_PROPULSION_SLOT_CONFIGS = new Slot0Configs()
            .withKS(0.1)
            .withKG(0.0)
            .withKV(0.12)
            .withKA(0.0)
            .withKP(0.11)
            .withKI(0.0)
            .withKD(0.0);

        public static final Slot0Configs LEFT_HOOD_PROPULSION_SLOT_CONFIGS = new Slot0Configs()
            .withKS(0.1)
            .withKG(0.0)
            .withKV(0.12)
            .withKA(0.0)
            .withKP(0.11)
            .withKI(0.0)
            .withKD(0.0);

        public static final Slot0Configs INDEXER_SLOT_CONFIGS = new Slot0Configs()
            .withKS(0.1)
            .withKG(0.0)
            .withKV(0.12)
            .withKA(0.0)
            .withKP(0.11)
            .withKI(0.0)
            .withKD(0.0);

        public static final double HOOD_SHOOTING_GEAR_RATIO = 1.0;
        public static final double HOOD_ANGLE_GEAR_RATIO = 1.0;

        public static final double CONVERSION_RATIO_FROM_METERS_TO_RPS =
            HOOD_SHOOTING_GEAR_RATIO / ShootingConstants.WHEEL_CIRCUMFERENCE;

        public static final double INDEXER_SPEED = 0.95;

        public static final double HOOD_ANGLE_TOLERANCE_ROT = 0.01;
        public static final double HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS = 0.05;

        public static final double CONVERSION_RATIO_RAD_TO_ROT =
            HOOD_ANGLE_GEAR_RATIO / (2.0 * Math.PI);

        public static final double COMPLEMENTARY_ANGLE = Math.toRadians(90.0);

        // Home position for relative encoder systems (assume robot boots at home).
        public static final double HOOD_HOME_POSITION_ROTATIONS = 0.0;

        // Offset in radians applied AFTER complementary conversion and BEFORE rotations.
        public static final double HOOD_ANGLE_OFFSET_RADIANS = 0.0;

        // Post-shot behavior: cut wheels + indexer first, then return hood to home.
        public static final double POST_SHOT_COAST_MINIMUM_TIME_SECONDS = 0.10;
        public static final double POST_SHOT_COAST_MAXIMUM_TIME_SECONDS = 0.75;

        public static final double POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND = 1.0;

        // Shooter / hood simulation parameters (must match real mechanism ratios)

        // Rotor rotations per shooter wheel rotation (example: 2.0 means rotor spins 2x wheel)
        public static final double SHOOTER_WHEEL_ROTOR_TO_WHEEL_GEAR_RATIO = 1.0;

        // Rotor rotations per hood arm rotation (example: 100.0 means rotor spins 100x arm)
        public static final double HOOD_ANGLE_ROTOR_TO_ARM_GEAR_RATIO = 1.0;

        // Rough inertias (tune if you care about realism; otherwise just keep stable values)
        public static final double SHOOTER_WHEEL_MOMENT_OF_INERTIA_KG_METERS_SQUARED = 0.001;
        public static final double HOOD_ARM_MOMENT_OF_INERTIA_KG_METERS_SQUARED = 0.004;

        // Hood arm geometry (meters) for SingleJointedArmSim (tune)
        public static final double HOOD_ARM_LENGTH_METERS = 0.33;
    }

    public static final class IntakeConstants {

        public static final int INTAKE_MOTOR_ID = 23;
        public static final int PIVOT_INTAKE__RIGHT_MOTOR_ID = 19;
        public static final int PIVOT_INTAKE_LEFT_MOTOR_ID = 18;

        public static final double INTAKE_ACTIVATION_VOLTAGE_VOLTS = 12.0 * 0.6;

        // ---------------------------
        // Your calibration-friendly space (normalized rotations 0..1)
        // 0.0 = fully retracted, 1.0 = fully deployed
        // ---------------------------
        public static final double PIVOT_RETRACT_POSITION_ROTATIONS = 0.0;
        public static final double PIVOT_DEPLOY_POSITION_ROTATIONS = 1.0;

        // Safety clamp (still in normalized rotations)
        public static final double PIVOT_SOFT_LIMIT_MINIMUM_ROTATIONS = -0.10;
        public static final double PIVOT_SOFT_LIMIT_MAXIMUM_ROTATIONS = 1.10;

        public static final double PIVOT_POSITION_TOLERANCE_ROTATIONS = 0.02;
        public static final double PIVOT_POSITION_HYSTERESIS_ROTATIONS = 0.02;

        // Simple PID gains (RIO PID in subsystem)
        public static final double PIVOT_POSITION_PROPORTIONAL_GAIN = 0.5;
        public static final double PIVOT_POSITION_INTEGRAL_GAIN = 0.0;
        public static final double PIVOT_POSITION_DERIVATIVE_GAIN = 0.0;

        public static final double PIVOT_CONTROL_MAXIMUM_ABSOLUTE_VOLTAGE_VOLTS = 12.0;

        // ---------------------------
        // REAL robot calibration placeholders (raw relative encoder rotations)
        // normalized = (raw - retractRaw) / (deployRaw - retractRaw)
        // ---------------------------
        public static final double PIVOT_RAW_ENCODER_RETRACT_ROTATIONS = 0.0;
        public static final double PIVOT_RAW_ENCODER_DEPLOY_ROTATIONS = 25.0;

        // If you always boot with the intake physically retracted, this makes life easy.
        public static final boolean PIVOT_ZERO_ENCODER_ON_BOOT_TO_RETRACT = true;

        // ---------------------------
        // SIM calibration (SingleJointedArmSim angles in radians)
        // normalized = (angle - retractAngle) / (deployAngle - retractAngle)
        // ---------------------------
        public static final double PIVOT_SIM_RETRACT_ANGLE_RADIANS = 0.0;
        public static final double PIVOT_SIM_DEPLOY_ANGLE_RADIANS = 1.57; // ~90 degrees, tweak later if you want

        // MapleSim intake geometry (bounding box)
        public static final double MAPLESIM_INTAKE_WIDTH_METERS = 0.70;
        public static final double MAPLESIM_INTAKE_EXTENSION_METERS = 0.20;
        public static final int MAPLESIM_INTAKE_CAPACITY = 1;

        // Pivot arm simulation model
        public static final double PIVOT_SIM_GEAR_REDUCTION = 60.0;
        public static final double PIVOT_SIM_MOMENT_OF_INERTIA = 0.002;
        public static final double PIVOT_SIM_ARM_LENGTH_METERS = 0.25;
        public static final double PIVOT_SIM_MIN_ANGLE_RADIANS = -0.2;
        public static final double PIVOT_SIM_MAX_ANGLE_RADIANS = 1.8;
        public static final double PIVOT_SIM_START_ANGLE_RADIANS = 0.0;
        public static final int ROLLER_MOTOR_ID = 23;
        public static final int PIVOT_INTAKE_RIGHT_MOTOR = 19;
        public static final int PIVOT_INTAKE_LEFT_MOTOR = 18;
        public static final boolean PIVOT_RIGHT_MOTOR_INVERTED = false;


    }

    public static final class ClimberConstants {

        public static final int LEFT_CLIMBER_MOTOR_ID = 20;
        public static final int RIGHT_CLIMBER_MOTOR_ID = 21;

        public static final double CLIMBER_MAX_DUTY_CYCLE = 0.8;

        // Positions (rotations)
        public static final double CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS = 0.0;
        public static final double CLIMBER_EXTENDED_POSITION_ROTATIONS = 0.0; // <-- AJUSTA esto (ahorita tienes 0)

        // “Ya llegó” tolerance (rotations)
        public static final double CLIMBER_POSITION_TOLERANCE_ROTATIONS = 0.05;

        // Manual voltage scaling
        public static final double CLIMBER_MANUAL_VOLTAGE_VOLTS =
            Constants.BATTERY_VOLTAGE * CLIMBER_MAX_DUTY_CYCLE;

        // PID gains
        public static final double CLIMBER_POSITION_PROPORTIONAL_GAIN = 6.0;
        public static final double CLIMBER_POSITION_INTEGRAL_GAIN = 0.0;
        public static final double CLIMBER_POSITION_DERIVATIVE_GAIN = 0.2;

        // Integrator clamp (volts)
        public static final double CLIMBER_PID_INTEGRATOR_MINIMUM_VOLTS = -Constants.BATTERY_VOLTAGE;
        public static final double CLIMBER_PID_INTEGRATOR_MAXIMUM_VOLTS = Constants.BATTERY_VOLTAGE;

        // Output clamp (volts)
        public static final double CLIMBER_OUTPUT_MINIMUM_VOLTS = -Constants.BATTERY_VOLTAGE;
        public static final double CLIMBER_OUTPUT_MAXIMUM_VOLTS = Constants.BATTERY_VOLTAGE;

        // Averaging factor (keep it explicit even if it’s “mathy”)
        public static final double CLIMBER_AVERAGE_MULTIPLIER = 0.5;

        private ClimberConstants() {
            // Prevent instantiation
        }
    }
}