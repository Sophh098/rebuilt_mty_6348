// File: src/main/java/frc/robot/Drive/generated/TunerConstants.java
package frc.robot.Drive.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drive.CommandSwerveDrivetrain;

/**
 * Centralized constants/configuration for a CTRE Phoenix 6 swerve drivetrain.
 *
 * What this class provides:
 * - PID + feedforward gains (Slot0) for steer and drive motors.
 * - Electrical constraints (current limits, slip current).
 * - Physical parameters (gear ratios, wheel radius, inertia, friction voltage).
 * - CAN bus configuration and Pigeon2 configuration.
 * - Per-module constants: CAN device IDs, absolute encoder offsets, module positions, inversion flags.
 * - Factory methods to create a {@link CommandSwerveDrivetrain} instance with the configured modules.
 *
 * Important:
 * - This file lives under a "generated" package path, which usually implies it may be overwritten by tooling
 *   (CTRE Tuner X or a generator). If your workflow regenerates it, keep changes minimal and prefer changing
 *   values in {@link DriveConstants}.
 */
public class TunerConstants {

    // -------------------- Gains --------------------

    /**
     * Steer motor Slot0 gains (PID + feedforward).
     * Feedforward sign is tied to the closed-loop sign to keep KS direction consistent.
     */
    private static final Slot0Configs STEER_SLOT0_GAINS_CONFIGURATION = new Slot0Configs()
        .withKP(DriveConstants.STEER_GAINS_KP)
        .withKI(DriveConstants.STEER_GAINS_KI)
        .withKD(DriveConstants.STEER_GAINS_KD)
        .withKS(DriveConstants.STEER_GAINS_KS)
        .withKV(DriveConstants.STEER_GAINS_KV)
        .withKA(DriveConstants.STEER_GAINS_KA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    /**
     * Drive motor Slot0 gains (PID + feedforward).
     */
    private static final Slot0Configs DRIVE_SLOT0_GAINS_CONFIGURATION = new Slot0Configs()
        .withKP(DriveConstants.DRIVE_GAINS_KP)
        .withKI(DriveConstants.DRIVE_GAINS_KI)
        .withKD(DriveConstants.DRIVE_GAINS_KD)
        .withKS(DriveConstants.DRIVE_GAINS_KS)
        .withKV(DriveConstants.DRIVE_GAINS_KV)
        .withKA(DriveConstants.DRIVE_GAINS_KA);

    // -------------------- Motor / Feedback Types --------------------

    /**
     * Steer closed-loop output: Voltage means the controller outputs volts directly (not duty cycle).
     */
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

    /**
     * Drive closed-loop output: Voltage means the controller outputs volts directly (not duty cycle).
     */
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

    /**
     * Drive motor arrangement: integrated TalonFX sensors/config assumptions.
     */
    private static final DriveMotorArrangement DRIVE_MOTOR_ARRANGEMENT = DriveMotorArrangement.TalonFX_Integrated;

    /**
     * Steer motor arrangement: integrated TalonFX sensors/config assumptions.
     */
    private static final SteerMotorArrangement STEER_MOTOR_ARRANGEMENT = SteerMotorArrangement.TalonFX_Integrated;

    /**
     * Steering feedback source:
     * FusedCANcoder typically combines integrated motor sensor with absolute CANcoder for better robustness.
     */
    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // -------------------- Electrical / Physical --------------------

    /**
     * Slip current used by CTRE swerve control to help detect/limit traction slip (implementation depends on CTRE).
     */
    private static final Current SLIP_CURRENT = DriveConstants.kSlipCurrent;

    /**
     * Default/initial configuration applied to drive TalonFX controllers.
     * (Currently empty; typically filled by the CTRE module factory + your constants.)
     */
    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGURATION = new TalonFXConfiguration();

    /**
     * Default/initial configuration applied to steer TalonFX controllers.
     * Includes a stator current limit to protect the steering mechanism.
     */
    private static final TalonFXConfiguration STEER_INITIAL_CONFIGURATION = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(DriveConstants.SteeringStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
        );

    /**
     * Default/initial configuration applied to CANcoder absolute encoders.
     */
    private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGURATION = new CANcoderConfiguration();

    /**
     * Pigeon2 configuration block for gyro settings.
     * If null, CTRE will use default configuration.
     */
    private static final Pigeon2Configuration PIGEON_CONFIGURATION = null;

    /**
     * CAN bus used by the drivetrain devices (TalonFX, CANcoder, Pigeon2).
     * The second parameter is commonly a CTRE "hoots" (fault logging) file path depending on your setup.
     */
    public static final CANBus CONTROLLER_AREA_NETWORK_BUS =
        new CANBus(DriveConstants.CAN_BUS, DriveConstants.HOOT_FILE_PATH);

    /**
     * Theoretical/characterized max speed at 12V used by CTRE for feedforward scaling.
     */
    public static final LinearVelocity SPEED_AT_12_VOLTS = DriveConstants.kSpeedAt12Volts;

    /** Mechanical coupling ratio between steer and drive for certain swerve designs (CTRE uses it for compensation). */
    private static final double COUPLING_GEAR_RATIO = DriveConstants.K_COUPLE_RATIO;

    /** Drive gear ratio (motor rotations per wheel rotation, or per CTRE convention). */
    private static final double DRIVE_GEAR_RATIO = DriveConstants.K_DRIVE_GEAR_RATIO;

    /** Steer gear ratio (motor rotations per module rotation, or per CTRE convention). */
    private static final double STEER_GEAR_RATIO = DriveConstants.K_STEER_GEAR_RATIO;

    /** Wheel radius used for kinematics and speed/position conversions. */
    private static final Distance WHEEL_RADIUS = DriveConstants.kWheelRadius;

    /** Whether the left-side drive motors should be inverted (drivetrain-wide convention). */
    private static final boolean LEFT_SIDE_INVERTED = DriveConstants.K_INVERT_LEFT_SIDE;

    /** Whether the right-side drive motors should be inverted (drivetrain-wide convention). */
    private static final boolean RIGHT_SIDE_INVERTED = DriveConstants.K_INVERT_RIGHT_SIDE;

    /** CAN device ID for the Pigeon2 gyro. */
    private static final int PIGEON_DEVICE_IDENTIFIER = DriveConstants.K_PIGEON_ID;

    /** Estimated steer moment of inertia used by CTRE swerve control models. */
    private static final MomentOfInertia STEER_MOMENT_OF_INERTIA = DriveConstants.kSteerInertia;

    /** Estimated drive moment of inertia used by CTRE swerve control models. */
    private static final MomentOfInertia DRIVE_MOMENT_OF_INERTIA = DriveConstants.kDriveInertia;

    /** Steer friction voltage used to model stiction/friction effects. */
    private static final Voltage STEER_FRICTION_VOLTAGE = DriveConstants.kSteerFrictionVoltage;

    /** Drive friction voltage used to model stiction/friction effects. */
    private static final Voltage DRIVE_FRICTION_VOLTAGE = DriveConstants.kDriveFrictionVoltage;

    // -------------------- Drivetrain Constants --------------------

    /**
     * Drivetrain-level CTRE swerve constants:
     * - CAN bus name
     * - Pigeon2 gyro ID
     * - Pigeon2 configuration
     */
    public static final SwerveDrivetrainConstants SWERVE_DRIVETRAIN_CONSTANTS =
        new SwerveDrivetrainConstants()
            .withCANBusName(CONTROLLER_AREA_NETWORK_BUS.getName())
            .withPigeon2Id(PIGEON_DEVICE_IDENTIFIER)
            .withPigeon2Configs(PIGEON_CONFIGURATION);

    /**
     * Factory that builds per-module {@link SwerveModuleConstants} using shared parameters
     * (ratios, gains, inertias, friction models, motor/encoder types, etc.).
     *
     * This reduces duplication and ensures all modules share the same configuration baseline.
     */
    private static final SwerveModuleConstantsFactory<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > SWERVE_MODULE_CONSTANTS_FACTORY =
        new SwerveModuleConstantsFactory<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        >()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withWheelRadius(WHEEL_RADIUS)
            .withSteerMotorGains(STEER_SLOT0_GAINS_CONFIGURATION)
            .withDriveMotorGains(DRIVE_SLOT0_GAINS_CONFIGURATION)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT_TYPE)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT_TYPE)
            .withSlipCurrent(SLIP_CURRENT)
            .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
            .withDriveMotorType(DRIVE_MOTOR_ARRANGEMENT)
            .withSteerMotorType(STEER_MOTOR_ARRANGEMENT)
            .withFeedbackSource(STEER_FEEDBACK_TYPE)
            .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGURATION)
            .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGURATION)
            .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGURATION)
            .withSteerInertia(STEER_MOMENT_OF_INERTIA)
            .withDriveInertia(DRIVE_MOMENT_OF_INERTIA)
            .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

    // -------------------- Modules --------------------

    /**
     * Front-left swerve module constants:
     * - CAN IDs (steer motor, drive motor, CANcoder)
     * - absolute encoder offset
     * - module position on robot (X, Y)
     * - inversion flags (drive side, steer motor, encoder)
     */
    public static final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > FRONT_LEFT_MODULE =
        SWERVE_MODULE_CONSTANTS_FACTORY.createModuleConstants(
            DriveConstants.K_FRONT_LEFT_STEER_MOTOR_ID,
            DriveConstants.K_FRONT_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.K_FRONT_LEFT_ENCODER_ID,
            DriveConstants.kFrontLeftEncoderOffset,
            DriveConstants.kFrontLeftXPos,
            DriveConstants.kFrontLeftYPos,
            LEFT_SIDE_INVERTED,
            DriveConstants.K_FRONT_LEFT_STEER_MOTOR_INVERTED,
            DriveConstants.K_FRONT_LEFT_ENCODER_INVERTED
        );

    /** Front-right swerve module constants. */
    public static final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > FRONT_RIGHT_MODULE =
        SWERVE_MODULE_CONSTANTS_FACTORY.createModuleConstants(
            DriveConstants.K_FRONT_RIGHT_STEER_MOTOR_ID,
            DriveConstants.K_FRONT_RIGHT_DRIVE_MOTOR_ID,
            DriveConstants.K_FRONT_RIGHT_ENCODER_ID,
            DriveConstants.kFrontRightEncoderOffset,
            DriveConstants.kFrontRightXPos,
            DriveConstants.kFrontRightYPos,
            RIGHT_SIDE_INVERTED,
            DriveConstants.K_FRONT_RIGHT_STEER_MOTOR_INVERTED,
            DriveConstants.K_FRONT_RIGHT_ENCODER_INVERTED
        );

    /** Back-left swerve module constants. */
    public static final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > BACK_LEFT_MODULE =
        SWERVE_MODULE_CONSTANTS_FACTORY.createModuleConstants(
            DriveConstants.K_BACK_LEFT_STEER_MOTOR_ID,
            DriveConstants.K_BACK_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.K_BACK_LEFT_ENCODER_ID,
            DriveConstants.kBackLeftEncoderOffset,
            DriveConstants.kBackLeftXPos,
            DriveConstants.kBackLeftYPos,
            LEFT_SIDE_INVERTED,
            DriveConstants.K_BACK_LEFT_STEER_MOTOR_INVERTED,
            DriveConstants.K_BACK_LEFT_ENCODER_INVERTED
        );

    /** Back-right swerve module constants. */
    public static final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > BACK_RIGHT_MODULE =
        SWERVE_MODULE_CONSTANTS_FACTORY.createModuleConstants(
            DriveConstants.K_BACK_RIGHT_STEER_MOTOR_ID,
            DriveConstants.K_BACK_RIGHT_DRIVE_MOTOR_ID,
            DriveConstants.K_BACK_RIGHT_ENCODER_ID,
            DriveConstants.kBackRightEncoderOffset,
            DriveConstants.kBackRightXPos,
            DriveConstants.kBackRightYPos,
            RIGHT_SIDE_INVERTED,
            DriveConstants.K_BACK_RIGHT_STEER_MOTOR_INVERTED,
            DriveConstants.K_BACK_RIGHT_ENCODER_INVERTED
        );

    /**
     * Creates the team's drivetrain implementation using the CTRE swerve constants for all modules.
     *
     * @return a configured {@link CommandSwerveDrivetrain} ready to be used by commands.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            SWERVE_DRIVETRAIN_CONSTANTS,
            FRONT_LEFT_MODULE,
            FRONT_RIGHT_MODULE,
            BACK_LEFT_MODULE,
            BACK_RIGHT_MODULE
        );
    }

    /**
     * CTRE "tuner" drivetrain implementation.
     * This is commonly used by CTRE example code and tooling; it extends {@link SwerveDrivetrain}
     * and supplies constructors for different odometry and covariance configurations.
     *
     * Note: The class name is uppercase in the original; it is kept as-is to match generated patterns.
     */
    public static class TUNER_SWERVE_DRIVETRAIN
        extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

        /**
         * Basic constructor: uses default odometry update frequency.
         *
         * @param drivetrainConstants drivetrain-wide constants (CAN bus, Pigeon ID, etc.)
         * @param modules module constants (one per swerve module)
         */
        public TUNER_SWERVE_DRIVETRAIN(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                drivetrainConstants,
                modules
            );
        }

        /**
         * Constructor with explicit odometry update frequency.
         *
         * @param drivetrainConstants drivetrain-wide constants
         * @param odometryUpdateFrequency update rate for odometry calculations (Hz)
         * @param modules module constants
         */
        public TUNER_SWERVE_DRIVETRAIN(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                drivetrainConstants,
                odometryUpdateFrequency,
                modules
            );
        }

        /**
         * Constructor with odometry frequency and standard deviation matrices for odometry/vision fusion.
         *
         * @param drivetrainConstants drivetrain-wide constants
         * @param odometryUpdateFrequency update rate for odometry calculations (Hz)
         * @param odometryStandardDeviation standard deviations for odometry measurements
         * @param visionStandardDeviation standard deviations for vision measurements
         * @param modules module constants
         */
        public TUNER_SWERVE_DRIVETRAIN(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules
            );
        }
    }
}