package frc.robot.Drive;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Drive.generated.TunerConstants.TUNER_SWERVE_DRIVETRAIN;

/**
 * Command-based wrapper around CTRE Phoenix 6 swerve drivetrain.
 *
 * Responsibilities:
 * - Exposes drivetrain state in a way that works cleanly with Command-Based (Subsystem interface).
 * - Configures PathPlanner AutoBuilder for holonomic autos.
 * - Provides SysId routines for characterizing translation, steer, and rotation.
 * - Handles simulation loop timing for more realistic PID behavior in sim.
 * - Applies "operator perspective" forward direction based on alliance color.
 *
 * Notes:
 * - This class extends the CTRE generated drivetrain class so it inherits the CTRE control API (setControl, getState, etc.).
 * - This file looks based on CTRE Tuner X generator patterns; be careful if your workflow regenerates it.
 */
public class CommandSwerveDrivetrain extends TUNER_SWERVE_DRIVETRAIN implements Subsystem {

    /**
     * Simulation loop period in seconds.
     * CTRE recommends a faster sim loop so closed-loop behavior is closer to reality.
     */
    private static final double SIMULATION_LOOP_PERIOD_SECONDS = 0.004; // 4 ms

    /** Notifier used to run the drivetrain simulation loop at a fixed period. */
    private Notifier simulationNotifier = null;

    /** Last simulation timestamp (seconds) used to compute delta time for sim updates. */
    private double lastSimulationTimeSeconds;

    /** Blue alliance forward is 0 degrees (toward the red alliance wall). */
    private static final Rotation2d blueAllianceForwardRotation = Rotation2d.kZero;

    /** Red alliance forward is 180 degrees (toward the blue alliance wall). */
    private static final Rotation2d redAllianceForwardRotation = Rotation2d.k180deg;

    /**
     * Tracks whether operator perspective has ever been applied.
     * Used so that if code restarts mid-match, the drivetrain can correct perspective at least once.
     */
    private boolean hasAppliedOperatorPerspective = false;

    // -------------------- SysId Requests --------------------

    /** Swerve request used by SysId to characterize translation (drive motors). */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterizationRequest =
        new SwerveRequest.SysIdSwerveTranslation();

    /** Swerve request used by SysId to characterize steer motor control gains. */
    private final SwerveRequest.SysIdSwerveSteerGains steerGainsCharacterizationRequest =
        new SwerveRequest.SysIdSwerveSteerGains();

    /** Swerve request used by SysId to characterize rotational response (heading controller). */
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterizationRequest =
        new SwerveRequest.SysIdSwerveRotation();

    // -------------------- PathPlanner Request --------------------

    /**
     * Swerve request used by PathPlanner AutoBuilder to apply chassis speeds
     * plus optional wheel force feedforwards.
     */
    private final SwerveRequest.ApplyRobotSpeeds pathPlannerApplyRobotSpeedsRequest =
        new SwerveRequest.ApplyRobotSpeeds();

    // -------------------- Reflection / Compatibility --------------------

    /**
     * Avoid spamming DriverStation errors if we cannot find any compatible pose reset method name.
     * CTRE has historically changed method names across versions (setPose vs seedFieldRelative vs etc.).
     */
    private boolean hasReportedPoseResetMethodMissing = false;

    // -------------------- SysId Routines --------------------

    /**
     * SysId routine for characterizing translation.
     * Typically used to tune drive motor closed-loop and feedforward.
     */
    private final SysIdRoutine systemIdentificationRoutineTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate (1 V/s)
                Volts.of(4),  // Reduced dynamic step voltage to avoid brownout
                null,         // Default timeout (10 s)
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> setControl(translationCharacterizationRequest.withVolts(output)),
                null,
                this
            )
        );

    /**
     * SysId routine for characterizing steer.
     * Typically used to tune steer motor gains (module azimuth control).
     */
    private final SysIdRoutine systemIdentificationRoutineSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate (1 V/s)
                Volts.of(7),  // Higher dynamic voltage for steering system response
                null,         // Default timeout (10 s)
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> setControl(steerGainsCharacterizationRequest.withVolts(volts)),
                null,
                this
            )
        );

    /**
     * SysId routine for characterizing rotation.
     *
     * Important units note:
     * - SysId tooling expects "volts", but this request wants angular rates/accelerations.
     * - This routine intentionally "abuses" volts as a carrier unit so SysId can log/plot it.
     * - The logged values are still useful, as long as you interpret them consistently during analysis.
     */
    private final SysIdRoutine systemIdentificationRoutineRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(Math.PI / 6).per(Second), // treated as rad/s^2 but logged as V/s
                Volts.of(Math.PI),                 // treated as rad/s but logged as V
                null, // Default timeout (10 s)
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> {
                    // "output" is treated as a rotational rate but carried through SysId as volts.
                    setControl(rotationCharacterizationRequest.withRotationalRate(output.in(Volts)));
                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                },
                null,
                this
            )
        );

    /** Currently selected SysId routine to apply when sysIdQuasistatic/sysIdDynamic are called. */
    private SysIdRoutine systemIdentificationRoutineToApply = systemIdentificationRoutineTranslation;

    /** Select translation characterization routine (drive motors). */
    public void useSystemIdentificationTranslationRoutine() {
        systemIdentificationRoutineToApply = systemIdentificationRoutineTranslation;
    }

    /** Select steer characterization routine (steer motors). */
    public void useSystemIdentificationSteerRoutine() {
        systemIdentificationRoutineToApply = systemIdentificationRoutineSteer;
    }

    /** Select rotation characterization routine (heading response). */
    public void useSystemIdentificationRotationRoutine() {
        systemIdentificationRoutineToApply = systemIdentificationRoutineRotation;
    }

    /**
     * Constructs a CTRE swerve drivetrain using the specified constants and modules.
     * Also starts simulation thread (if in sim) and configures PathPlanner AutoBuilder.
     *
     * @param drivetrainConstants drivetrain-wide constants (CAN bus name, Pigeon ID, etc.)
     * @param modules             module constants (one per swerve module)
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimulationThread();
        }
        configureAutoBuilderInternal();
    }

    /**
     * Constructs a CTRE swerve drivetrain with explicit odometry update frequency.
     *
     * @param drivetrainConstants     drivetrain-wide constants
     * @param odometryUpdateFrequency odometry loop frequency (Hz)
     * @param modules                 module constants
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimulationThread();
        }
        configureAutoBuilderInternal();
    }

    /**
     * Constructs a CTRE swerve drivetrain with odometry update frequency and covariance matrices
     * for odometry/vision fusion.
     *
     * @param drivetrainConstants       drivetrain-wide constants
     * @param odometryUpdateFrequency   odometry loop frequency (Hz)
     * @param odometryStandardDeviation odometry standard deviation vector [x, y, theta]^T
     * @param visionStandardDeviation   vision standard deviation vector [x, y, theta]^T
     * @param modules                   module constants
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimulationThread();
        }
        configureAutoBuilderInternal();
    }

    /**
     * Convenience accessor for method references (ex: drivetrain::getPose).
     *
     * @return current estimated robot pose (meters, radians)
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * Convenience accessor for method references (ex: drivetrain::getYawRateRadiansPerSecond).
     *
     * @return current estimated yaw rate in radians per second
     */
    public double getYawRateRadiansPerSecond() {
        return getState().Speeds.omegaRadiansPerSecond;
    }

    /**
     * Public wrapper to configure PathPlanner later as well.
     * Safe to call outside constructors (for example, after constants/config updates).
     */
    public void configureAutoBuilder() {
        configureAutoBuilderInternal();
    }

    /**
     * PathPlanner AutoBuilder configuration.
     *
     * Design goals:
     * - Uses robot config from GUI settings.
     * - Registers pose supplier, pose reset, chassis speed supplier, and output consumer.
     * - Uses holonomic controller with translation and rotation PID constants.
     * - Avoids crashing robot code if the config file is missing or malformed.
     */
    @SuppressWarnings("all")
    private void configureAutoBuilderInternal() {
        try {
            RobotConfig robotConfiguration = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPoseForAutoBuilder,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    pathPlannerApplyRobotSpeedsRequest.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(
                        AutoConstants.TRANSLATION_KP,
                        AutoConstants.TRANSLATION_KI,
                        AutoConstants.TRANSLATION_KD
                    ),
                    new PIDConstants(
                        AutoConstants.ROTATION_KP,
                        AutoConstants.ROTATION_KI,
                        AutoConstants.ROTATION_KD
                    )
                ),
                robotConfiguration,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );

            SignalLogger.writeString("PathPlanner_AutoBuilder", "Configured");
        } catch (IOException | ParseException exception) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", exception.getStackTrace());
            SignalLogger.writeString("PathPlanner_AutoBuilder", "ConfigurationFailed");
        }
    }

    /**
     * Pose reset wrapper used by PathPlanner AutoBuilder.
     *
     * CTRE method names for pose reset have varied across versions, so this function attempts multiple known names
     * via reflection and uses the first one that exists.
     *
     * If none are found, it reports a DriverStation error one time.
     *
     * @param desiredPoseMeters desired pose in field coordinates
     */
    public void resetPoseForAutoBuilder(Pose2d desiredPoseMeters) {
        if (tryInvokePoseResetMethod("setPose", desiredPoseMeters)) {
            return;
        }
        if (tryInvokePoseResetMethod("seedFieldRelative", desiredPoseMeters)) {
            return;
        }
        if (tryInvokePoseResetMethod("seedFieldCentric", desiredPoseMeters)) {
            return;
        }
        if (tryInvokePoseResetMethod("resetPose", desiredPoseMeters)) {
            return;
        }

        if (!hasReportedPoseResetMethodMissing) {
            hasReportedPoseResetMethodMissing = true;
            DriverStation.reportError(
                "AutoBuilder pose reset failed: no pose reset method found (setPose/seedFieldRelative/seedFieldCentric/resetPose).",
                false
            );
        }
    }

    /**
     * Attempts to invoke a pose reset method by name using reflection.
     *
     * @param methodName method name to search for
     * @param desiredPoseMeters pose argument
     * @return true if the method was found and invoked successfully
     */
    @SuppressWarnings("all")
    private boolean tryInvokePoseResetMethod(String methodName, Pose2d desiredPoseMeters) {
        try {
            Method poseResetMethod = getClass().getMethod(methodName, Pose2d.class);
            poseResetMethod.invoke(this, desiredPoseMeters);
            SignalLogger.writeString("AutoBuilder_PoseResetMethodUsed", methodName);
            return true;
        } catch (NoSuchMethodException
            | IllegalAccessException
            | InvocationTargetException
            | SecurityException exception) {
            return false;
        }
    }

    /**
     * Creates a command that continuously applies a swerve request to the drivetrain.
     *
     * Typical use:
     * - drivetrain.applyRequest(() -> driveRequest.with...);
     *
     * @param request supplier that returns the desired CTRE {@link SwerveRequest}
     * @return command that runs until interrupted
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    /**
     * Runs the SysId quasistatic test in the given direction for the currently selected routine.
     *
     * @param direction SysId test direction
     * @return a command that runs the quasistatic test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return systemIdentificationRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId dynamic test in the given direction for the currently selected routine.
     *
     * @param direction SysId test direction
     * @return a command that runs the dynamic test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return systemIdentificationRoutineToApply.dynamic(direction);
    }

    /**
     * Periodic hook:
     * Applies operator perspective forward direction based on alliance.
     *
     * Rationale:
     * - If we have never applied perspective before, do it even if enabled, to recover from mid-match code restart.
     * - Otherwise, only update while disabled so driver controls do not flip unexpectedly during testing.
     */
    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? redAllianceForwardRotation
                        : blueAllianceForwardRotation
                );
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * Starts a faster simulation loop so drivetrain closed-loop control behaves more realistically in sim.
     * Uses WPILib battery voltage as supply voltage input.
     */
    private void startSimulationThread() {
        lastSimulationTimeSeconds = Utils.getCurrentTimeSeconds();

        simulationNotifier = new Notifier(() -> {
            final double currentTimeSeconds = Utils.getCurrentTimeSeconds();
            double deltaTimeSeconds = currentTimeSeconds - lastSimulationTimeSeconds;
            lastSimulationTimeSeconds = currentTimeSeconds;

            updateSimState(deltaTimeSeconds, RobotController.getBatteryVoltage());
        });

        simulationNotifier.startPeriodic(SIMULATION_LOOP_PERIOD_SECONDS);
    }

    /**
     * Adds a vision measurement into the CTRE/WPILib pose estimator.
     * Converts FPGA timestamps into "current time" which CTRE uses internally.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement with measurement standard deviations.
     *
     * @param visionRobotPoseMeters measured robot pose
     * @param timestampSeconds FPGA timestamp seconds
     * @param visionMeasurementStdDevs measurement standard deviations [x, y, theta]^T
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs
        );
    }

    /**
     * Samples the pose estimate at a given timestamp, if the internal buffer is available.
     *
     * @param timestampSeconds FPGA timestamp seconds
     * @return an Optional containing a pose sample if available
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}