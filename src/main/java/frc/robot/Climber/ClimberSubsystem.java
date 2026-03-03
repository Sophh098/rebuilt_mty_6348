// File: src/main/java/frc/robot/Climber/ClimberSubsystem.java
package frc.robot.Climber;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Subsystem responsible for controlling a two-motor climber (left and right).
 *
 * Control strategy:
 * - MANUAL_VOLTAGE: open-loop voltage command (expand/retract/stop).
 * - POSITION_PID: closed-loop position control using a WPILib PIDController on the *average* encoder position.
 *
 * Safety behavior:
 * - If the robot is disabled, the subsystem stops motors and resets the PID integrator.
 *
 * Initialization behavior:
 * - On the first periodic() call after boot, encoder positions are zeroed assuming the climber starts retracted.
 *
 * Notes:
 * - This subsystem reads encoder positions every periodic() and caches them into member variables.
 * - The PID controller output is clamped to allowed voltage limits.
 */
public final class ClimberSubsystem extends SubsystemBase {

    /**
     * Defines the active control mode for the climber.
     */
    private enum ClimberControlMode {
        /** Direct voltage control (open-loop). */
        MANUAL_VOLTAGE,
        /** Position control using PID on average position. */
        POSITION_PID
    }

    /** Motor controller for the left climber motor. */
    private final SparkMax leftClimberMotorController;
    /** Motor controller for the right climber motor. */
    private final SparkMax rightClimberMotorController;

    /**
     * PID controller used for closed-loop position control.
     * It drives both motors with the same voltage based on the average encoder position.
     */
    private final PIDController positionPidController =
        new PIDController(
            ClimberConstants.CLIMBER_POSITION_PROPORTIONAL_GAIN,
            ClimberConstants.CLIMBER_POSITION_INTEGRAL_GAIN,
            ClimberConstants.CLIMBER_POSITION_DERIVATIVE_GAIN
        );

    /** Current control mode (defaults to manual voltage). */
    private ClimberControlMode controlMode = ClimberControlMode.MANUAL_VOLTAGE;

    /**
     * The last requested voltage in MANUAL_VOLTAGE mode.
     *
     * IMPORTANT: This value should represent volts, not a position.
     */
    private double requestedManualVoltageVolts = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;

    /** Target position (in rotations) for POSITION_PID mode. */
    private double requestedTargetPositionRotations =
        ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;

    /** Cached left encoder position (rotations). Updated in periodic(). */
    private double leftPositionRotations = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
    /** Cached right encoder position (rotations). Updated in periodic(). */
    private double rightPositionRotations = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;

    /** Ensures boot-time encoder initialization is only performed once. */
    private boolean bootInitializationCompleted = false;

    /**
     * Creates the climber subsystem with new SparkMax instances using constants for IDs and motor type.
     */
    public ClimberSubsystem() {
        this(
            new SparkMax(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        );
    }

    /**
     * Creates the climber subsystem with injected motor controllers (useful for testing).
     *
     * @param leftClimberMotorController motor controller for the left side
     * @param rightClimberMotorController motor controller for the right side
     */
    public ClimberSubsystem(SparkMax leftClimberMotorController, SparkMax rightClimberMotorController) {
        this.leftClimberMotorController = leftClimberMotorController;
        this.rightClimberMotorController = rightClimberMotorController;

        // If one side needs inversion, do it here.
        // this.rightClimberMotorController.setInverted(true);

        // Prevent integral windup by limiting integrator contribution (in volts).
        positionPidController.setIntegratorRange(
            ClimberConstants.CLIMBER_PID_INTEGRATOR_MINIMUM_VOLTS,
            ClimberConstants.CLIMBER_PID_INTEGRATOR_MAXIMUM_VOLTS
        );
    }

    /**
     * Main periodic loop:
     * - Reads encoder positions and caches them.
     * - Performs one-time boot initialization (zeroing encoders assuming retracted).
     * - If disabled: stop motors and reset PID.
     * - Otherwise: apply either manual voltage or PID position control.
     */
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

    /**
     * Reads current encoder positions from both SparkMax controllers and caches them.
     */
    private void readEncoderPositions() {
        leftPositionRotations = leftClimberMotorController.getEncoder().getPosition();
        rightPositionRotations = rightClimberMotorController.getEncoder().getPosition();
    }

    /**
     * Resets encoder positions and internal state, assuming the climber is physically in the retracted position.
     * This is used at boot as a convenience when there is no absolute sensor or limit switch.
     */
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

    /**
     * Immediately commands both motors to stop (voltage = 0).
     */
    private void stopMotors() {
        leftClimberMotorController.setVoltage(ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS);
        rightClimberMotorController.setVoltage(ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS);
    }

    /**
     * Applies an open-loop voltage request to both motors after clamping it to safe limits.
     */
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

    /**
     * Applies closed-loop position control using PID:
     * - Uses average encoder position as the process variable.
     * - Uses requestedTargetPositionRotations as the setpoint.
     * - Clamps PID output to safe voltage limits.
     */
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

    /**
     * Commands the climber to expand using open-loop voltage control.
     */
    public void expand() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = ClimberConstants.CLIMBER_MANUAL_VOLTAGE_VOLTS;
    }

    /**
     * Commands the climber to retract using open-loop voltage control.
     */
    public void retract() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = -ClimberConstants.CLIMBER_MANUAL_VOLTAGE_VOLTS;
    }

    /**
     * Stops the climber and switches to MANUAL_VOLTAGE mode.
     */
    public void stop() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
        stopMotors();
    }

    // ---------------- Auto (PID) ----------------

    /**
     * Commands the climber to move to the extended position using PID position control.
     */
    public void autoExpand() {
        controlMode = ClimberControlMode.POSITION_PID;
        requestedTargetPositionRotations = ClimberConstants.CLIMBER_EXTENDED_POSITION_ROTATIONS;
    }

    /**
     * Commands the climber to move to the retracted position using PID position control.
     */
    public void autoRetract() {
        controlMode = ClimberControlMode.POSITION_PID;
        requestedTargetPositionRotations = ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS;
    }

    // ---------------- State ----------------

    /**
     * @return cached left climber position in rotations
     */
    public double getLeftClimberPositionRotations() {
        return leftPositionRotations;
    }

    /**
     * @return cached right climber position in rotations
     */
    public double getRightClimberPositionRotations() {
        return rightPositionRotations;
    }

    /**
     * @return average of left and right positions in rotations
     */
    public double getAverageClimberPositionRotations() {
        return (leftPositionRotations + rightPositionRotations) * ClimberConstants.CLIMBER_AVERAGE_MULTIPLIER;
    }

    /**
     * @return true if the climber is at or beyond the extended threshold (with tolerance)
     */
    public boolean isClimberExtended() {
        return getAverageClimberPositionRotations() >=
            (ClimberConstants.CLIMBER_EXTENDED_POSITION_ROTATIONS - ClimberConstants.CLIMBER_POSITION_TOLERANCE_ROTATIONS);
    }

    /**
     * @return true if the climber is at or below the retracted threshold (with tolerance)
     */
    public boolean isClimberRetracted() {
        return getAverageClimberPositionRotations() <=
            (ClimberConstants.CLIMBER_RETRACTED_TARGET_POSITION_ROTATIONS + ClimberConstants.CLIMBER_POSITION_TOLERANCE_ROTATIONS);
    }

    /**
     * @return the last requested target position for PID mode (rotations)
     */
    public double getLastRequestedTargetRotations() {
        return requestedTargetPositionRotations;
    }

    /**
     * Re-zeros encoders assuming the climber is currently retracted.
     */
    public void zeroEncoders() {
        zeroEncodersAssumingRetracted();
    }
}