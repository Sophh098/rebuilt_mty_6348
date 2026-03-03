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

public class TunerConstants {

    // -------------------- Gains --------------------

    private static final Slot0Configs STEER_SLOT0_GAINS_CONFIGURATION = new Slot0Configs()
        .withKP(DriveConstants.STEER_GAINS_KP)
        .withKI(DriveConstants.STEER_GAINS_KI)
        .withKD(DriveConstants.STEER_GAINS_KD)
        .withKS(DriveConstants.STEER_GAINS_KS)
        .withKV(DriveConstants.STEER_GAINS_KV)
        .withKA(DriveConstants.STEER_GAINS_KA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs DRIVE_SLOT0_GAINS_CONFIGURATION = new Slot0Configs()
        .withKP(DriveConstants.DRIVE_GAINS_KP)
        .withKI(DriveConstants.DRIVE_GAINS_KI)
        .withKD(DriveConstants.DRIVE_GAINS_KD)
        .withKS(DriveConstants.DRIVE_GAINS_KS)
        .withKV(DriveConstants.DRIVE_GAINS_KV)
        .withKA(DriveConstants.DRIVE_GAINS_KA);

    // -------------------- Motor / Feedback Types --------------------

    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement DRIVE_MOTOR_ARRANGEMENT = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement STEER_MOTOR_ARRANGEMENT = SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // -------------------- Electrical / Physical --------------------

    private static final Current SLIP_CURRENT = DriveConstants.kSlipCurrent;

    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGURATION = new TalonFXConfiguration();

    private static final TalonFXConfiguration STEER_INITIAL_CONFIGURATION = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(DriveConstants.SteeringStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
        );

    private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGURATION = new CANcoderConfiguration();

    private static final Pigeon2Configuration PIGEON_CONFIGURATION = null;

    public static final CANBus CONTROLLER_AREA_NETWORK_BUS =
        new CANBus(DriveConstants.CAN_BUS, DriveConstants.HOOT_FILE_PATH);

    public static final LinearVelocity SPEED_AT_12_VOLTS =
        DriveConstants.kSpeedAt12Volts;

    private static final double COUPLING_GEAR_RATIO =
        DriveConstants.K_COUPLE_RATIO;

    private static final double DRIVE_GEAR_RATIO =
        DriveConstants.K_DRIVE_GEAR_RATIO;

    private static final double STEER_GEAR_RATIO =
        DriveConstants.K_STEER_GEAR_RATIO;

    private static final Distance WHEEL_RADIUS =
        DriveConstants.kWheelRadius;

    private static final boolean LEFT_SIDE_INVERTED =
        DriveConstants.K_INVERT_LEFT_SIDE;

    private static final boolean RIGHT_SIDE_INVERTED =
        DriveConstants.K_INVERT_RIGHT_SIDE;

    private static final int PIGEON_DEVICE_IDENTIFIER =
        DriveConstants.K_PIGEON_ID;

    private static final MomentOfInertia STEER_MOMENT_OF_INERTIA =
        DriveConstants.kSteerInertia;

    private static final MomentOfInertia DRIVE_MOMENT_OF_INERTIA =
        DriveConstants.kDriveInertia;

    private static final Voltage STEER_FRICTION_VOLTAGE =
        DriveConstants.kSteerFrictionVoltage;

    private static final Voltage DRIVE_FRICTION_VOLTAGE =
        DriveConstants.kDriveFrictionVoltage;

    // -------------------- Drivetrain Constants --------------------

    public static final SwerveDrivetrainConstants SWERVE_DRIVETRAIN_CONSTANTS =
        new SwerveDrivetrainConstants()
            .withCANBusName(CONTROLLER_AREA_NETWORK_BUS.getName())
            .withPigeon2Id(PIGEON_DEVICE_IDENTIFIER)
            .withPigeon2Configs(PIGEON_CONFIGURATION);

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

    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            SWERVE_DRIVETRAIN_CONSTANTS,
            FRONT_LEFT_MODULE,
            FRONT_RIGHT_MODULE,
            BACK_LEFT_MODULE,
            BACK_RIGHT_MODULE
        );
    }

    public static class TUNER_SWERVE_DRIVETRAIN
        extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

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