package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }

        // ── Publish the Field2d to SmartDashboard so AdvantageScope can find it ──
        // Use this path in AdvantageScope: /SmartDashboard/Field/Robot
        SmartDashboard.putData("Field", m_field);
    }

    // ── NetworkTables — odometry thread state (250 Hz) ────────────────────
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d>             drivePose            = driveStateTable.getStructTopic("Pose",            Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds>      driveSpeeds          = driveStateTable.getStructTopic("Speeds",          ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState>    driveModuleStates    = driveStateTable.getStructArrayTopic("ModuleStates",    SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState>    driveModuleTargets   = driveStateTable.getStructArrayTopic("ModuleTargets",   SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp         = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    // ── Legacy /Pose table (kept for backwards compatibility) ─────────────
    private final NetworkTable     poseTable    = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = poseTable.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher  fieldTypePub = poseTable.getStringTopic(".type").publish();

    // ── Field2d — THIS is what AdvantageScope should graph ────────────────
    //
    // IMPORTANT: telemeterize() is called from CTRE's internal odometry thread
    // (not the main robot loop), so state.Pose is odometry-only and does NOT
    // reflect addVisionMeasurement calls.
    //
    // To see vision-corrected pose, call publishVisionFusedPose() from the
    // main robot loop (e.g. drivetrain.periodic() or RobotContainer).
    //
    // AdvantageScope paths:
    //   /SmartDashboard/Field/Robot           ← vision-fused pose  ✅
    //   /SmartDashboard/Field/OdometryOnly    ← raw odometry only  (for comparison)
    //   /DriveState/Pose                      ← same as OdometryOnly
    private final Field2d m_field = new Field2d();

    // Reused array to avoid per-call allocation
    private final double[] m_poseArray = new double[3];

    /**
     * Called from CTRE's internal odometry thread (~250 Hz).
     * {@code state.Pose} here is ODOMETRY ONLY — vision measurements are NOT fused yet.
     * Do NOT use this for displaying the vision-corrected robot pose.
     */
    public void telemeterize(SwerveDriveState state) {
        // ── Raw drivetrain state (odometry thread) ────────────────────────
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        SignalLogger.writeStruct("DriveState/Pose",   Pose2d.struct,            state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct,     state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates",    SwerveModuleState.struct,    state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets",   SwerveModuleState.struct,    state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        // Legacy /Pose table — publishes odometry-only pose
        fieldTypePub.set("Field2d");
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        // OdometryOnly robot on the Field2d (for comparison vs vision-fused)
        m_field.getObject("OdometryOnly").setPose(state.Pose);

        // Module visualization
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }

    public void publishVisionFusedPose(Pose2d visionFusedPose) {
        // "Robot" is the default slot — this is what AdvantageScope shows as the main robot
        m_field.setRobotPose(visionFusedPose);
        SignalLogger.writeStruct("DriveState/VisionFusedPose", Pose2d.struct, visionFusedPose);
    }

    // ── Module Mechanism2d ────────────────────────────────────────────────

    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1),
        new Mechanism2d(1, 1), new Mechanism2d(1, 1),
    };
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5).append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5).append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5).append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5).append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };
}