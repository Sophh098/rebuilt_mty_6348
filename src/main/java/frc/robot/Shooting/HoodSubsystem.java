package frc.robot.Shooting;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // ✅ IMPORT FALTANTE
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Drive.generated.TunerConstants;
import frc.robot.Vision.VisionSubsystem;

public class HoodSubsystem extends SubsystemBase {

    private enum HoodOperationalState {
        STOWED,
        AIMING_AND_SPINNING,
        POST_SHOT_COASTING,
        STOWING
    }

    // ========== HARDWARE ==========
    private final TalonFX hoodAngleMotorController;
    private final TalonFX rightWheelMotorController;
    private final TalonFX leftWheelMotorController;
    private final TalonFX indexerMotorController;

    // ========== CONTROL REQUESTS ==========
    private final VelocityTorqueCurrentFOC rightWheelVelocityControlRequest =
        new VelocityTorqueCurrentFOC(0.0);

    private final VelocityTorqueCurrentFOC leftWheelVelocityControlRequest =
        new VelocityTorqueCurrentFOC(0.0);

    private final MotionMagicExpoTorqueCurrentFOC hoodAnglePositionControlRequest =
        new MotionMagicExpoTorqueCurrentFOC(0.0);

    private final DutyCycleOut indexerDutyCycleControlRequest =
        new DutyCycleOut(0.0);

    // ========== STATE MANAGEMENT ==========
    private final Debouncer readyDebouncerSeconds =
        new Debouncer(0.12, Debouncer.DebounceType.kRising);

    private HoodOperationalState hoodOperationalState = HoodOperationalState.STOWED;

    private boolean shootingRequestActive = false;
    private boolean readyToShoot = false;

    // ========== SHOOTING PARAMETERS ==========
    private double desiredExitSpeedMetersPerSecond =
        ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;

    private double desiredHoodAngleRadiansFromModel =
        ShootingConstants.DEFAULT_HOOD_ANGLE_RADIANS;

    // ========== COAST STATE TRACKING ==========
    private double postShotCoastStartTimestampSeconds = 0.0;
    private double hoodHoldPositionRotations = 0.0;

    // ========== GOAL SETPOINTS ==========
    private double goalWheelRotationsPerSecond = 0.0;
    private double goalHoodAnglePositionRotations = 0.0;
    private boolean indexerEnabled = false;

    // ========== SENSOR READINGS ==========
    private double hoodAnglePositionRotations = 0.0;
    private double leftWheelVelocityRotationsPerSecond = 0.0;
    private double rightWheelVelocityRotationsPerSecond = 0.0;

    // ========== CONSTRUCTOR ==========
    public HoodSubsystem() {
        this(TunerConstants.CONTROLLER_AREA_NETWORK_BUS);
    }

    public HoodSubsystem(CANBus canBusName) {
        hoodAngleMotorController = new TalonFX(HoodConstants.HOOD_ANGLE_TALON_ID, canBusName);
        rightWheelMotorController = new TalonFX(HoodConstants.RIGHT_HOOD_PROPULSION_TALON_ID, canBusName);
        leftWheelMotorController = new TalonFX(HoodConstants.LEFT_HOOD_PROPULSION_TALON_ID, canBusName);
        indexerMotorController = new TalonFX(HoodConstants.INDEXER_TALON_ID, canBusName);

        TalonFXConfigurator hoodAngleConfigurator = hoodAngleMotorController.getConfigurator();
        TalonFXConfigurator rightWheelConfigurator = rightWheelMotorController.getConfigurator();
        TalonFXConfigurator leftWheelConfigurator = leftWheelMotorController.getConfigurator();
        TalonFXConfigurator indexerConfigurator = indexerMotorController.getConfigurator();

        hoodAngleConfigurator.apply(HoodConstants.HOOD_ANGLE_SLOT_CONFIGS);
        rightWheelConfigurator.apply(HoodConstants.RIGHT_HOOD_PROPULSION_SLOT_CONFIGS);
        leftWheelConfigurator.apply(HoodConstants.LEFT_HOOD_PROPULSION_SLOT_CONFIGS);
        indexerConfigurator.apply(HoodConstants.INDEXER_SLOT_CONFIGS);
    }

    // ========== MAIN PERIODIC LOOP ==========
    @Override
    public void periodic() {
        // 1. Read all sensor values
        readHardwareSensors();

        // 2. Handle disabled state
        if (DriverStation.isDisabled()) {
            applyDisabledOutputs();
            readyToShoot = false;
            return;
        }

        // 3. Update state machine transitions
        updateOperationalStateTransitions();

        // 4. Execute state-specific logic
        switch (hoodOperationalState) {
            case AIMING_AND_SPINNING -> {
                // Calculate wheel speed goal
                goalWheelRotationsPerSecond =
                    desiredExitSpeedMetersPerSecond * HoodConstants.CONVERSION_RATIO_FROM_METERS_TO_RPS;

                // Calculate hood angle goal
                double complementaryAngleRadians =
                    HoodConstants.COMPLEMENTARY_ANGLE - desiredHoodAngleRadiansFromModel;

                double correctedAngleRadians =
                    complementaryAngleRadians - HoodConstants.HOOD_ANGLE_OFFSET_RADIANS;

                goalHoodAnglePositionRotations =
                    correctedAngleRadians * HoodConstants.CONVERSION_RATIO_RAD_TO_ROT;

                // Check if ready (with debouncer to avoid premature activation)
                boolean rawReady =
                    isWheelVelocityReady(goalWheelRotationsPerSecond) &&
                    isHoodAngleReady(goalHoodAnglePositionRotations);

                readyToShoot = readyDebouncerSeconds.calculate(rawReady);

                // Only enable indexer when ready AND shooting request is active
                indexerEnabled = shootingRequestActive && readyToShoot;
            }

            case POST_SHOT_COASTING -> {
                indexerEnabled = false;
                goalWheelRotationsPerSecond = 0.0;
                goalHoodAnglePositionRotations = hoodHoldPositionRotations;

                double currentTimestampSeconds = Timer.getFPGATimestamp();
                double elapsedSeconds = currentTimestampSeconds - postShotCoastStartTimestampSeconds;

                boolean hasMinimumTimeElapsed =
                    elapsedSeconds >= HoodConstants.POST_SHOT_COAST_MINIMUM_TIME_SECONDS;

                boolean hasMaximumTimeElapsed =
                    elapsedSeconds >= HoodConstants.POST_SHOT_COAST_MAXIMUM_TIME_SECONDS;

                boolean wheelsAreSlowEnough = areWheelsBelowStopThreshold();

                // Transition to stowing when wheels have slowed down enough
                if ((hasMinimumTimeElapsed && wheelsAreSlowEnough) || hasMaximumTimeElapsed) {
                    hoodOperationalState = HoodOperationalState.STOWING;
                }

                readyToShoot = false;
            }

            case STOWING -> {
                indexerEnabled = false;
                goalWheelRotationsPerSecond = 0.0;
                goalHoodAnglePositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;

                readyToShoot = false;

                // Transition to stowed when hood reaches home position
                if (isHoodAngleReady(HoodConstants.HOOD_HOME_POSITION_ROTATIONS)) {
                    hoodOperationalState = HoodOperationalState.STOWED;
                }
            }

            case STOWED -> {
                indexerEnabled = false;
                goalWheelRotationsPerSecond = 0.0;
                goalHoodAnglePositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;

                readyToShoot = false;
            }
        }

        // 5. Send telemetry to dashboard
        logTelemetry();

        // 6. Apply motor outputs
        applyEnabledOutputs();
    }

    // ========== PUBLIC API ==========
    public void setShootingRequestActive(boolean isActive) {
        shootingRequestActive = isActive;
    }

    public void updateShootingSolution(VisionSubsystem visionSubsystem, ShootingHelper shootingHelper) {
        if (!shootingRequestActive) {
            return;
        }

        boolean shouldUseLiveSolution =
            visionSubsystem.hasTarget()
            && visionSubsystem.isShootingTargetValid()
            && shootingHelper.isPossibleToShoot();

        if (shouldUseLiveSolution) {
            desiredExitSpeedMetersPerSecond = shootingHelper.getExitSpeedMetersPerSecond();
            desiredHoodAngleRadiansFromModel = shootingHelper.getHoodAngleRadians();
            return;
        }

        // Fallback to default values if no valid vision solution
        desiredExitSpeedMetersPerSecond = ShootingConstants.DEFAULT_EXIT_SPEED_METERS_PER_SECOND;
        desiredHoodAngleRadiansFromModel = ShootingConstants.DEFAULT_HOOD_ANGLE_RADIANS;
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public void stop() {
        shootingRequestActive = false;
        hoodOperationalState = HoodOperationalState.STOWED;

        goalWheelRotationsPerSecond = 0.0;
        goalHoodAnglePositionRotations = HoodConstants.HOOD_HOME_POSITION_ROTATIONS;
        indexerEnabled = false;

        readyToShoot = false;
        applyDisabledOutputs();
    }

    // ========== PRIVATE HELPERS ==========
    private void updateOperationalStateTransitions() {
        // If shooting is requested, go to aiming state
        if (shootingRequestActive) {
            if (hoodOperationalState != HoodOperationalState.AIMING_AND_SPINNING) {
                hoodOperationalState = HoodOperationalState.AIMING_AND_SPINNING;
                readyDebouncerSeconds.calculate(false); // Reset debouncer
            }
            return;
        }

        // If we were aiming and shooting stopped, begin coasting
        if (hoodOperationalState == HoodOperationalState.AIMING_AND_SPINNING) {
            hoodOperationalState = HoodOperationalState.POST_SHOT_COASTING;
            postShotCoastStartTimestampSeconds = Timer.getFPGATimestamp();
            hoodHoldPositionRotations = hoodAnglePositionRotations;
            readyToShoot = false;
        }
    }

    private void readHardwareSensors() {
        hoodAnglePositionRotations = hoodAngleMotorController.getPosition().getValueAsDouble();
        leftWheelVelocityRotationsPerSecond = leftWheelMotorController.getVelocity().getValueAsDouble();
        rightWheelVelocityRotationsPerSecond = rightWheelMotorController.getVelocity().getValueAsDouble();
    }

    private void applyDisabledOutputs() {
        rightWheelMotorController.setControl(rightWheelVelocityControlRequest.withVelocity(0.0));
        leftWheelMotorController.setControl(leftWheelVelocityControlRequest.withVelocity(0.0));
        indexerMotorController.setControl(indexerDutyCycleControlRequest.withOutput(0.0));

        // Hold current position when disabled
        double currentHoodAnglePositionRotations = hoodAngleMotorController.getPosition().getValueAsDouble();
        hoodAngleMotorController.setControl(
            hoodAnglePositionControlRequest.withPosition(currentHoodAnglePositionRotations)
        );
    }

    private void applyEnabledOutputs() {
        // Note: Right wheel is inverted (negative velocity)
        rightWheelMotorController.setControl(
            rightWheelVelocityControlRequest.withVelocity(-goalWheelRotationsPerSecond)
        );

        leftWheelMotorController.setControl(
            leftWheelVelocityControlRequest.withVelocity(goalWheelRotationsPerSecond)
        );

        hoodAngleMotorController.setControl(
            hoodAnglePositionControlRequest.withPosition(goalHoodAnglePositionRotations)
        );

        double requestedDutyCycle = indexerEnabled ? HoodConstants.INDEXER_SPEED : 0.0;
        indexerMotorController.setControl(indexerDutyCycleControlRequest.withOutput(requestedDutyCycle));
    }

    private boolean isWheelVelocityReady(double goalWheelRotationsPerSecond) {
        double leftMeasuredRotationsPerSecond = leftWheelVelocityRotationsPerSecond;
        double rightMeasuredRotationsPerSecond = rightWheelVelocityRotationsPerSecond;

        double leftError = Math.abs(leftMeasuredRotationsPerSecond - goalWheelRotationsPerSecond);
        double rightError = Math.abs(rightMeasuredRotationsPerSecond - (-goalWheelRotationsPerSecond));

        return leftError <= HoodConstants.HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS
            && rightError <= HoodConstants.HOOD_SHOOTING_VELOCITY_TOLERANCE_RPS;
    }

    private boolean isHoodAngleReady(double goalHoodAnglePositionRotations) {
        double measuredRotations = hoodAnglePositionRotations;
        double errorRotations = Math.abs(measuredRotations - goalHoodAnglePositionRotations);
        return errorRotations <= HoodConstants.HOOD_ANGLE_TOLERANCE_ROT;
    }

    private boolean areWheelsBelowStopThreshold() {
        double leftAbsoluteRotationsPerSecond = Math.abs(leftWheelVelocityRotationsPerSecond);
        double rightAbsoluteRotationsPerSecond = Math.abs(rightWheelVelocityRotationsPerSecond);

        return leftAbsoluteRotationsPerSecond <= HoodConstants.POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND
            && rightAbsoluteRotationsPerSecond <= HoodConstants.POST_SHOT_WHEEL_STOP_THRESHOLD_ROTATIONS_PER_SECOND;
    }

    private void logTelemetry() {
        SmartDashboard.putString("Hood/State", hoodOperationalState.toString());
        SmartDashboard.putBoolean("Hood/ShootingRequestActive", shootingRequestActive);
        SmartDashboard.putBoolean("Hood/ReadyToShoot", readyToShoot);
        SmartDashboard.putBoolean("Hood/IndexerEnabled", indexerEnabled);
        
        SmartDashboard.putNumber("Hood/DesiredAngleRadians", desiredHoodAngleRadiansFromModel);
        SmartDashboard.putNumber("Hood/DesiredAngleDegrees", Math.toDegrees(desiredHoodAngleRadiansFromModel));
        SmartDashboard.putNumber("Hood/CurrentAngleRotations", hoodAnglePositionRotations);
        SmartDashboard.putNumber("Hood/GoalAngleRotations", goalHoodAnglePositionRotations);
        
        SmartDashboard.putNumber("Hood/LeftWheelRPS", leftWheelVelocityRotationsPerSecond);
        SmartDashboard.putNumber("Hood/RightWheelRPS", rightWheelVelocityRotationsPerSecond);
        SmartDashboard.putNumber("Hood/GoalWheelRPS", goalWheelRotationsPerSecond);
        
        SmartDashboard.putNumber("Hood/DesiredExitSpeedMPS", desiredExitSpeedMetersPerSecond);
    }
}