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

public class CommandSwerveDrivetrain extends TUNER_SWERVE_DRIVETRAIN implements Subsystem {

    private static final double SIMULATION_LOOP_PERIOD_SECONDS = 0.004;

    private Notifier simulationNotifier = null;
    private double   lastSimulationTimeSeconds;

    private static final Rotation2d blueAllianceForwardRotation = Rotation2d.kZero;
    private static final Rotation2d redAllianceForwardRotation  = Rotation2d.k180deg;

    private boolean hasAppliedOperatorPerspective    = false;
    private boolean hasReportedPoseResetMethodMissing = false;

    // ── SysId requests ────────────────────────────────────────────────────
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterizationRequest =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerGainsCharacterizationRequest =
        new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterizationRequest =
        new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.ApplyRobotSpeeds pathPlannerApplyRobotSpeedsRequest =
        new SwerveRequest.ApplyRobotSpeeds();

    // ── SysId routines ────────────────────────────────────────────────────
    private final SysIdRoutine systemIdentificationRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> setControl(translationCharacterizationRequest.withVolts(output)), null, this));

    private final SysIdRoutine systemIdentificationRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> setControl(steerGainsCharacterizationRequest.withVolts(volts)), null, this));

    private final SysIdRoutine systemIdentificationRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> {
            setControl(rotationCharacterizationRequest.withRotationalRate(output.in(Volts)));
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
        }, null, this));

    private SysIdRoutine systemIdentificationRoutineToApply = systemIdentificationRoutineTranslation;

    // ── Constructors ──────────────────────────────────────────────────────

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimulationThread();
        configureAutoBuilderInternal();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) startSimulationThread();
        configureAutoBuilderInternal();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStdDev,
        Matrix<N3, N1> visionStdDev,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStdDev, visionStdDev, modules);
        if (Utils.isSimulation()) startSimulationThread();
        configureAutoBuilderInternal();
    }

    // ── Public API ────────────────────────────────────────────────────────

    public Pose2d getPose() {
        return getState().Pose;
    }

    public double getYawRateRadiansPerSecond() {
        return getState().Speeds.omegaRadiansPerSecond;
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    public void configureAutoBuilder() {
        configureAutoBuilderInternal();
    }

    public void useSystemIdentificationTranslationRoutine() {
        systemIdentificationRoutineToApply = systemIdentificationRoutineTranslation;
    }

    public void useSystemIdentificationSteerRoutine() {
        systemIdentificationRoutineToApply = systemIdentificationRoutineSteer;
    }

    public void useSystemIdentificationRotationRoutine() {
        systemIdentificationRoutineToApply = systemIdentificationRoutineRotation;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return systemIdentificationRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return systemIdentificationRoutineToApply.dynamic(direction);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // Mismo fix: sin conversión de timestamp.
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // ── Periodic ──────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? redAllianceForwardRotation
                        : blueAllianceForwardRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    // ── PathPlanner ───────────────────────────────────────────────────────

    public void resetPoseForAutoBuilder(Pose2d desiredPoseMeters) {
        if (tryInvokePoseResetMethod("setPose",              desiredPoseMeters)) return;
        if (tryInvokePoseResetMethod("seedFieldRelative",    desiredPoseMeters)) return;
        if (tryInvokePoseResetMethod("seedFieldCentric",     desiredPoseMeters)) return;
        if (tryInvokePoseResetMethod("resetPose",            desiredPoseMeters)) return;

        if (!hasReportedPoseResetMethodMissing) {
            hasReportedPoseResetMethodMissing = true;
            DriverStation.reportError(
                "AutoBuilder pose reset failed: no pose reset method found " +
                "(setPose / seedFieldRelative / seedFieldCentric / resetPose).", false);
        }
    }

    @SuppressWarnings("all")
    private boolean tryInvokePoseResetMethod(String methodName, Pose2d pose) {
        try {
            Method m = getClass().getMethod(methodName, Pose2d.class);
            m.invoke(this, pose);
            SignalLogger.writeString("AutoBuilder_PoseResetMethodUsed", methodName);
            return true;
        } catch (NoSuchMethodException | IllegalAccessException |
                 InvocationTargetException | SecurityException e) {
            return false;
        }
    }

    @SuppressWarnings("all")
    private void configureAutoBuilderInternal() {
        try {
            RobotConfig robotConfiguration = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPoseForAutoBuilder,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    pathPlannerApplyRobotSpeedsRequest
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                    new PIDConstants(AutoConstants.TRANSLATION_KP, AutoConstants.TRANSLATION_KI, AutoConstants.TRANSLATION_KD),
                    new PIDConstants(AutoConstants.ROTATION_KP,    AutoConstants.ROTATION_KI,    AutoConstants.ROTATION_KD)),
                robotConfiguration,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);

            SignalLogger.writeString("PathPlanner_AutoBuilder", "Configured");
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load PathPlanner config", e.getStackTrace());
            SignalLogger.writeString("PathPlanner_AutoBuilder", "ConfigurationFailed");
        }
    }

    // ── Simulation ────────────────────────────────────────────────────────

    private void startSimulationThread() {
        lastSimulationTimeSeconds = Utils.getCurrentTimeSeconds();
        simulationNotifier = new Notifier(() -> {
            final double now = Utils.getCurrentTimeSeconds();
            double dt = now - lastSimulationTimeSeconds;
            lastSimulationTimeSeconds = now;
            updateSimState(dt, RobotController.getBatteryVoltage());
        });
        simulationNotifier.startPeriodic(SIMULATION_LOOP_PERIOD_SECONDS);
    }
}