// File: src/main/java/frc/robot/Climber/ClimberSubsystem.java
package frc.robot.Climber;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public final class ClimberSubsystem extends SubsystemBase {

    private enum ClimberControlMode {
        MANUAL_VOLTAGE,
        POSITION_PID
    }

    private static final double retractedTargetPositionRotations = 0.0;

    // Solo para “ya llegó” en auto.
    private static final double positionToleranceRotations = 0.05;

    // Open-loop voltage (igual que tu código)
    private static final double manualVoltageVolts =
        Constants.BATTERY_VOLTAGE * ClimberConstants.CLIMBER_MAX_DUTY_CYCLE;

    // PID simple. Si quieres, luego lo movemos a ClimberConstants.
    private static final double positionProportionalGain = 6.0;
    private static final double positionIntegralGain = 0.0;
    private static final double positionDerivativeGain = 0.2;

    private final SparkMax leftClimberMotorController;
    private final SparkMax rightClimberMotorController;

    private final PIDController positionPidController =
        new PIDController(positionProportionalGain, positionIntegralGain, positionDerivativeGain);

    private ClimberControlMode controlMode = ClimberControlMode.MANUAL_VOLTAGE;

    private double requestedManualVoltageVolts = 0.0;
    private double requestedTargetPositionRotations = retractedTargetPositionRotations;

    private double leftPositionRotations = 0.0;
    private double rightPositionRotations = 0.0;

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

        // Si uno de los lados necesita invertir, hazlo aquí (si aplica en tu robot):
        // rightClimberMotorController.setInverted(true);

        positionPidController.setIntegratorRange(-12.0, 12.0);
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
        leftClimberMotorController.getEncoder().setPosition(0.0);
        rightClimberMotorController.getEncoder().setPosition(0.0);

        requestedTargetPositionRotations = retractedTargetPositionRotations;
        requestedManualVoltageVolts = 0.0;
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;

        positionPidController.reset();
    }

    private void stopMotors() {
        leftClimberMotorController.setVoltage(0.0);
        rightClimberMotorController.setVoltage(0.0);
    }

    private void applyManualVoltage() {
        double clampedVoltageVolts =
            MathUtil.clamp(requestedManualVoltageVolts, -12.0, 12.0);

        leftClimberMotorController.setVoltage(clampedVoltageVolts);
        rightClimberMotorController.setVoltage(clampedVoltageVolts);
    }

    private void applyPositionPid() {
        double averagePositionRotations = getAverageClimberPositionRotations();

        double feedbackVoltageVolts =
            positionPidController.calculate(averagePositionRotations, requestedTargetPositionRotations);

        double clampedVoltageVolts =
            MathUtil.clamp(feedbackVoltageVolts, -12.0, 12.0);

        leftClimberMotorController.setVoltage(clampedVoltageVolts);
        rightClimberMotorController.setVoltage(clampedVoltageVolts);
    }

    // ---------------- Manual open-loop ----------------

    public void expand() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = manualVoltageVolts;
    }

    public void retract() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = -manualVoltageVolts;
    }

    public void stop() {
        controlMode = ClimberControlMode.MANUAL_VOLTAGE;
        requestedManualVoltageVolts = 0.0;
        stopMotors();
    }

    // ---------------- Auto (PID) ----------------

    public void autoExpand() {
        controlMode = ClimberControlMode.POSITION_PID;
        requestedTargetPositionRotations = ClimberConstants.CLIMBER_EXTENDED_POSITION;
    }

    public void autoRetract() {
        controlMode = ClimberControlMode.POSITION_PID;
        requestedTargetPositionRotations = retractedTargetPositionRotations;
    }

    // ---------------- State ----------------

    public double getLeftClimberPositionRotations() {
        return leftPositionRotations;
    }

    public double getRightClimberPositionRotations() {
        return rightPositionRotations;
    }

    public double getAverageClimberPositionRotations() {
        return (leftPositionRotations + rightPositionRotations) * 0.5;
    }

    public boolean isClimberExtended() {
        return getAverageClimberPositionRotations() >=
            (ClimberConstants.CLIMBER_EXTENDED_POSITION - positionToleranceRotations);
    }

    public boolean isClimberRetracted() {
        return getAverageClimberPositionRotations() <=
            (retractedTargetPositionRotations + positionToleranceRotations);
    }

    public double getLastRequestedTargetRotations() {
        return requestedTargetPositionRotations;
    }

    // Si quieres exponer “zero” manual para pit/driver:
    public void zeroEncoders() {
        zeroEncodersAssumingRetracted();
    }
}