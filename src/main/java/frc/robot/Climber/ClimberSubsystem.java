// File: src/main/java/frc/robot/Climber/ClimberSubsystem.java
package frc.robot.Climber;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public final class ClimberSubsystem extends SubsystemBase {

    private enum ClimberControlMode {
        MANUAL_VOLTAGE,
        POSITION_PID
    }

    private final SparkMax leftClimberMotorController;
    private final SparkMax rightClimberMotorController;

    private final PIDController positionPidController =
        new PIDController(
            ClimberConstants.CLIMBER_POSITION_PROPORTIONAL_GAIN,
            ClimberConstants.CLIMBER_POSITION_INTEGRAL_GAIN,
            ClimberConstants.CLIMBER_POSITION_DERIVATIVE_GAIN
        );

    private ClimberControlMode controlMode = ClimberControlMode.MANUAL_VOLTAGE;

    private double requestedManualVoltageVolts = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
    private double requestedTargetPositionRotations =
        ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;

    private double leftPositionRotations = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
    private double rightPositionRotations = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;

    private boolean bootInitializationCompleted = false;

    public ClimberSubsystem() {
        this(
            new SparkMax(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        );
    }

    public ClimberSubsystem(SparkMax leftClimberMotorController, SparkMax rightClimberMotorController) {
        this.leftClimberMotorController = leftClimberMotorController;
        this.rightClimberMotorController = rightClimberMotorController;

        // If one side needs inversion, do it here.
        // this.rightClimberMotorController.setInverted(true);

        positionPidController.setIntegratorRange(
            ClimberConstants.CLIMBER_PID_INTEGRATOR_MINIMUM_VOLTS,
            ClimberConstants.CLIMBER_PID_INTEGRATOR_MAXIMUM_VOLTS
        );
    }

    @Override
    public void periodic() {
        readEncoderPositions();

        if (!bootInitializationCompleted) {
            zeroEncodersAssumingRetracted();
            bootInitializationCompleted = true;
        }

        if (DriverStation.isDisabled()) {
            stopMotors();
            positionPidController.reset();
            return;
        }

        if (controlMode == ClimberControlMode.MANUAL_VOLTAGE) {
            applyManualVoltage();
            return;
        }

        applyPositionPid();
    }

    private void readEncoderPositions() {
        leftPositionRotations = leftClimberMotorController.getEncoder().getPosition();
        rightPositionRotations = rightClimberMotorController.getEncoder().getPosition();
    }

    private void zeroEncodersAssumingRetracted() {
        leftClimberMotorController.getEncoder().setPosition(
            ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS
        );
        rightClimberMotorController.getEncoder().setPosition(
            ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS
        );

        requestedTargetPositionRotations = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
        requestedManualVoltageVolts = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;

        positionPidController.reset();
    }

    private void stopMotors() {
        leftClimberMotorController.setVoltage(ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS);
        rightClimberMotorController.setVoltage(ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS);
    }

    private void applyManualVoltage() {
        double clampedVoltageVolts =
            MathUtil.clamp(
                requestedManualVoltageVolts,
                ClimberConstants.CLIMBER_OUTPUT_MINIMUM_VOLTS,
                ClimberConstants.CLIMBER_OUTPUT_MAXIMUM_VOLTS
            );

        leftClimberMotorController.setVoltage(clampedVoltageVolts);
        rightClimberMotorController.setVoltage(clampedVoltageVolts);
    }

    private void applyPositionPid() {
        double averagePositionRotations = getAverageClimberPositionRotations();

        double feedbackVoltageVolts =
            positionPidController.calculate(averagePositionRotations, requestedTargetPositionRotations);

        double clampedVoltageVolts =
            MathUtil.clamp(
                feedbackVoltageVolts,
                ClimberConstants.CLIMBER_OUTPUT_MINIMUM_VOLTS,
                ClimberConstants.CLIMBER_OUTPUT_MAXIMUM_VOLTS
            );

        leftClimberMotorController.setVoltage(clampedVoltageVolts);
        rightClimberMotorController.setVoltage(clampedVoltageVolts);
    }

    // ---------------- Manual open-loop ----------------

    public void expand() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = ClimberConstants.CLIMBER_MANUAL_VOLTAGE_VOLTS;
    }

    public void retract() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = -ClimberConstants.CLIMBER_MANUAL_VOLTAGE_VOLTS;
    }

    public void stop() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
        stopMotors();
    }

    // ---------------- Auto (PID) ----------------

    public void autoExpand() {
        controlMode = ClimberControlMode.POSITION_PID;
        requestedTargetPositionRotations = ClimberConstants.CLIMBER_EXTENDED_POSITION_ROTATIONS;
    }

    public void autoRetract() {
        controlMode = ClimberControlMode.POSITION_PID;
        requestedTargetPositionRotations = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
    }

    // ---------------- State ----------------

    public double getLeftClimberPositionRotations() {
        return leftPositionRotations;
    }

    public double getRightClimberPositionRotations() {
        return rightPositionRotations;
    }

    public double getAverageClimberPositionRotations() {
        return (leftPositionRotations + rightPositionRotations) * ClimberConstants.CLIMBER_AVERAGE_MULTIPLIER;
    }

    public boolean isClimberExtended() {
        return getAverageClimberPositionRotations() >=
            (ClimberConstants.CLIMBER_EXTENDED_POSITION_ROTATIONS - ClimberConstants.CLIMBER_POSITION_TOLERANCE_ROTATIONS);
    }

    public boolean isClimberRetracted() {
        return getAverageClimberPositionRotations() <=
            (ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS + ClimberConstants.CLIMBER_POSITION_TOLERANCE_ROTATIONS);
    }

    public double getLastRequestedTargetRotations() {
        return requestedTargetPositionRotations;
    }

    public void zeroEncoders() {
        zeroEncodersAssumingRetracted();
    }
}