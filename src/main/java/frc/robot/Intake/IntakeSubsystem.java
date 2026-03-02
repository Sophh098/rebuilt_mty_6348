// File: src/main/java/frc/robot/Intake/IntakeSubsystem.java
package frc.robot.Intake;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public final class IntakeSubsystem extends SubsystemBase {

    private final SparkMax rollerMotorController;
    private final SparkMax pivotMotorController;

    private final Debouncer rollerMotorConnectedDebouncer =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Debouncer pivotMotorConnectedDebouncer =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private boolean rollerMotorConnected = false;
    private boolean pivotMotorConnected = false;

    private double rollerVelocityRotationsPerSecond = 0.0;
    private double rollerAppliedVolts = 0.0;
    private double rollerCurrentAmps = 0.0;

    private double pivotRawEncoderPositionRotations = 0.0;
    private double pivotRawEncoderVelocityRotationsPerSecond = 0.0;

    // Normalized 0..1 “calibration workflow”
    private double pivotNormalizedPositionRotations = 0.0;
    private double pivotNormalizedVelocityRotationsPerSecond = 0.0;

    // Requests
    private boolean rollerEnabledRequest = false;
    private double pivotTargetNormalizedPositionRotationsRequest =
        IntakeConstants.PIVOT_RETRACT_POSITION_ROTATIONS;

    private final PIDController pivotPositionFeedbackController =
        new PIDController(
            IntakeConstants.PIVOT_POSITION_PROPORTIONAL_GAIN,
            IntakeConstants.PIVOT_POSITION_INTEGRAL_GAIN,
            IntakeConstants.PIVOT_POSITION_DERIVATIVE_GAIN
        );

    private boolean fullyDeployed = false;
    private boolean fullyRetracted = true;

    private boolean bootInitializationCompleted = false;

    // Placeholder until you wire a real sensor
    private boolean fuelDetectedInsideIntake = false;

    public IntakeSubsystem(SparkMax rollerMotorController, SparkMax pivotMotorController) {
        this.rollerMotorController = rollerMotorController;
        this.pivotMotorController = pivotMotorController;

        pivotPositionFeedbackController.setIntegratorRange(-12.0, 12.0);
    }

    @Override
    public void periodic() {
        readHardwareInputs();

        if (!bootInitializationCompleted) {
            performBootInitialization();
            bootInitializationCompleted = true;
        }

        if (DriverStation.isDisabled()) {
            stopAllMotorsAndResetController();
            return;
        }

        updateDeploymentStateFromPosition();
        applyPivotClosedLoopControl();
        applyRollerControl();
    }

    private void readHardwareInputs() {
        boolean rollerHadAnyError = false;
        boolean pivotHadAnyError = false;

        // ---------------- Roller ----------------
        var rollerEncoder = rollerMotorController.getEncoder();

        Double rollerVelocityRaw = readDoubleIfNoError(
            rollerMotorController,
            () -> rollerEncoder.getVelocity()
        );
        if (rollerVelocityRaw != null) {
            rollerVelocityRotationsPerSecond =
                convertSparkVelocityToRotationsPerSecond(rollerVelocityRaw.doubleValue());
        } else {
            rollerHadAnyError = true;
        }

        Double rollerAppliedOutput = readDoubleIfNoError(
            rollerMotorController,
            rollerMotorController::getAppliedOutput
        );
        Double rollerBusVoltage = readDoubleIfNoError(
            rollerMotorController,
            rollerMotorController::getBusVoltage
        );
        if (rollerAppliedOutput != null && rollerBusVoltage != null) {
            rollerAppliedVolts = rollerAppliedOutput.doubleValue() * rollerBusVoltage.doubleValue();
        } else {
            rollerHadAnyError = true;
        }

        Double rollerCurrent = readDoubleIfNoError(
            rollerMotorController,
            rollerMotorController::getOutputCurrent
        );
        if (rollerCurrent != null) {
            rollerCurrentAmps = rollerCurrent.doubleValue();
        } else {
            rollerHadAnyError = true;
        }

        rollerMotorConnected = rollerMotorConnectedDebouncer.calculate(!rollerHadAnyError);

        // ---------------- Pivot ----------------
        var pivotEncoder = pivotMotorController.getEncoder();

        Double pivotPositionRaw = readDoubleIfNoError(
            pivotMotorController,
            () -> pivotEncoder.getPosition()
        );
        if (pivotPositionRaw != null) {
            pivotRawEncoderPositionRotations = pivotPositionRaw.doubleValue();
        } else {
            pivotHadAnyError = true;
        }

        Double pivotVelocityRaw = readDoubleIfNoError(
            pivotMotorController,
            () -> pivotEncoder.getVelocity()
        );
        if (pivotVelocityRaw != null) {
            pivotRawEncoderVelocityRotationsPerSecond =
                convertSparkVelocityToRotationsPerSecond(pivotVelocityRaw.doubleValue());
        } else {
            pivotHadAnyError = true;
        }

        pivotMotorConnected = pivotMotorConnectedDebouncer.calculate(!pivotHadAnyError);

        // Raw -> normalized
        pivotNormalizedPositionRotations =
            getNormalizedPositionRotationsFromRawEncoderRotations(pivotRawEncoderPositionRotations);

        pivotNormalizedVelocityRotationsPerSecond =
            getNormalizedVelocityRotationsPerSecondFromRawEncoderVelocityRotationsPerSecond(
                pivotRawEncoderVelocityRotationsPerSecond
            );

        // No simulation: keep false unless you add a sensor
        fuelDetectedInsideIntake = false;
    }

    private void performBootInitialization() {
        if (IntakeConstants.PIVOT_ZERO_ENCODER_ON_BOOT_TO_RETRACT) {
            setPivotRawEncoderPositionRotations(IntakeConstants.PIVOT_RAW_ENCODER_RETRACT_ROTATIONS);
        }

        pivotTargetNormalizedPositionRotationsRequest =
            IntakeConstants.PIVOT_RETRACT_POSITION_ROTATIONS;

        pivotPositionFeedbackController.reset();
    }

    private void stopAllMotorsAndResetController() {
        rollerMotorController.setVoltage(0.0);
        pivotMotorController.setVoltage(0.0);
        pivotPositionFeedbackController.reset();
    }

    private void applyPivotClosedLoopControl() {
        double measuredNormalizedPositionRotations = pivotNormalizedPositionRotations;

        double clampedTargetNormalizedRotations =
            MathUtil.clamp(
                pivotTargetNormalizedPositionRotationsRequest,
                IntakeConstants.PIVOT_SOFT_LIMIT_MINIMUM_ROTATIONS,
                IntakeConstants.PIVOT_SOFT_LIMIT_MAXIMUM_ROTATIONS
            );

        double feedbackVolts =
            pivotPositionFeedbackController.calculate(
                measuredNormalizedPositionRotations,
                clampedTargetNormalizedRotations
            );

        double totalCommandedVolts =
            MathUtil.clamp(
                feedbackVolts,
                -IntakeConstants.PIVOT_CONTROL_MAXIMUM_ABSOLUTE_VOLTAGE_VOLTS,
                IntakeConstants.PIVOT_CONTROL_MAXIMUM_ABSOLUTE_VOLTAGE_VOLTS
            );

        pivotMotorController.setVoltage(totalCommandedVolts);
    }

    private void applyRollerControl() {
        if (!rollerEnabledRequest) {
            rollerMotorController.setVoltage(0.0);
            return;
        }

        rollerMotorController.setVoltage(IntakeConstants.INTAKE_ACTIVATION_VOLTAGE_VOLTS);
    }

    private void updateDeploymentStateFromPosition() {
        double positionRotations = pivotNormalizedPositionRotations;

        double deployTargetRotations = IntakeConstants.PIVOT_DEPLOY_POSITION_ROTATIONS;
        double retractTargetRotations = IntakeConstants.PIVOT_RETRACT_POSITION_ROTATIONS;

        double toleranceRotations = IntakeConstants.PIVOT_POSITION_TOLERANCE_ROTATIONS;
        double hysteresisRotations = IntakeConstants.PIVOT_POSITION_HYSTERESIS_ROTATIONS;

        boolean isAtDeploy = positionRotations >= (deployTargetRotations - toleranceRotations);
        boolean isAtRetract = positionRotations <= (retractTargetRotations + toleranceRotations);

        if (isAtDeploy) {
            fullyDeployed = true;
            fullyRetracted = false;
        } else if (positionRotations < (deployTargetRotations - toleranceRotations - hysteresisRotations)) {
            fullyDeployed = false;
        }

        if (isAtRetract) {
            fullyRetracted = true;
            fullyDeployed = false;
        } else if (positionRotations > (retractTargetRotations + toleranceRotations + hysteresisRotations)) {
            fullyRetracted = false;
        }
    }

    // ---------------- Requests ----------------

    public void requestDeployIntake() {
        pivotTargetNormalizedPositionRotationsRequest = IntakeConstants.PIVOT_DEPLOY_POSITION_ROTATIONS;
    }

    public void requestRetractIntake() {
        pivotTargetNormalizedPositionRotationsRequest = IntakeConstants.PIVOT_RETRACT_POSITION_ROTATIONS;
    }

    public void setRollerEnabled(boolean enabled) {
        rollerEnabledRequest = enabled;
    }

    // ---------------- State getters ----------------

    public boolean isIntakeDeployed() {
        return fullyDeployed;
    }

    public boolean isIntakeRetracted() {
        return fullyRetracted;
    }

    public boolean isFuelInsideIntake() {
        return fuelDetectedInsideIntake;
    }

    public double getPivotTargetPositionRotations() {
        return pivotTargetNormalizedPositionRotationsRequest;
    }

    // ---------------- Encoder utilities ----------------

    private void setPivotRawEncoderPositionRotations(double rawEncoderPositionRotations) {
        var pivotEncoder = pivotMotorController.getEncoder();
        pivotEncoder.setPosition(rawEncoderPositionRotations);
    }

    private static Double readDoubleIfNoError(SparkBase sparkBase, java.util.function.DoubleSupplier supplier) {
        double value = supplier.getAsDouble();
        REVLibError error = sparkBase.getLastError();
        if (error == REVLibError.kOk) {
            return value;
        }
        return null;
    }

    // Important:
    // Most REV encoder APIs report velocity in RPM. If your API already gives rotations/sec, remove the / 60.0.
    private static double convertSparkVelocityToRotationsPerSecond(double sparkVelocity) {
        return sparkVelocity / 60.0;
    }

    private static double getNormalizedPositionRotationsFromRawEncoderRotations(double rawEncoderRotations) {
        double retractRaw = IntakeConstants.PIVOT_RAW_ENCODER_RETRACT_ROTATIONS;
        double deployRaw = IntakeConstants.PIVOT_RAW_ENCODER_DEPLOY_ROTATIONS;

        double denominator = deployRaw - retractRaw;
        if (Math.abs(denominator) < 1e-9) {
            return 0.0;
        }

        return (rawEncoderRotations - retractRaw) / denominator;
    }

    private static double getNormalizedVelocityRotationsPerSecondFromRawEncoderVelocityRotationsPerSecond(
        double rawEncoderVelocityRotationsPerSecond
    ) {
        double retractRaw = IntakeConstants.PIVOT_RAW_ENCODER_RETRACT_ROTATIONS;
        double deployRaw = IntakeConstants.PIVOT_RAW_ENCODER_DEPLOY_ROTATIONS;

        double denominator = deployRaw - retractRaw;
        if (Math.abs(denominator) < 1e-9) {
            return 0.0;
        }

        return rawEncoderVelocityRotationsPerSecond / denominator;
    }
}