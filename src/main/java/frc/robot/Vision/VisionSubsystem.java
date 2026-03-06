package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldCosntants;

/**
 * Vision subsystem that manages N camera threads.
 *
 * <h3>Periodic cost breakdown (4 cameras)</h3>
 * <ul>
 *   <li>4× volatile read of {@code VisionIOInputs} — ~0.01 ms total</li>
 *   <li>1× {@code Timer.getFPGATimestamp()} call</li>
 *   <li>1× {@code currentRobotPoseSupplier.get()}</li>
 *   <li>0–4× {@code addVisionMeasurement()} calls (only on accepted observations)</li>
 * </ul>
 * All PhotonVision I/O runs in background threads at 20 Hz.
 * The main loop never touches NetworkTables or does pose math.
 */
public class VisionSubsystem extends SubsystemBase {

    // ── Functional interfaces ─────────────────────────────────────────────

    public interface VisionPoseMeasurementConsumer {
        void addVisionMeasurement(
                Pose2d robotPose,
                double timestampSeconds,
                Matrix<N3, N1> standardDeviations);
    }

    public interface VisionHardwareFactory {
        VisionIO createVisionHardware(
                VisionEntries.CameraSpecifications specs,
                AprilTagFieldLayout fieldLayout);
    }

    // ── State ─────────────────────────────────────────────────────────────

    private final List<CameraThread> cameraThreads = new ArrayList<>();

    private final Supplier<Pose2d>  currentPoseSupplier;
    private final Supplier<Double>  yawRateSupplier;
    private final VisionPoseMeasurementConsumer measurementConsumer;

    private final AprilTagFieldLayout          fieldLayout;
    private final VisionStandardDeviationModel stdDevModel;

    private final double fieldLengthMeters;
    private final double fieldWidthMeters;

    /** Fast lookup table: index = tag ID, value = is this a shooting target? */
    private final boolean[] shootingTagTable;

    private boolean visionEnabled               = true;
    private boolean shootingTargetValidThisCycle = false;

    // ── Debug logging ──────────────────────────────────────────────────────
    private long lastDebugLogTime = 0;

    // ── Constructor ───────────────────────────────────────────────────────

    public VisionSubsystem(
            AprilTagFieldLayout fieldLayout,
            double fieldLengthMeters,
            double fieldWidthMeters,
            Supplier<Pose2d> currentPoseSupplier,
            Supplier<Double> yawRateSupplier,
            VisionPoseMeasurementConsumer measurementConsumer,
            VisionStandardDeviationModel stdDevModel,
            List<VisionEntries.CameraSpecifications> cameraSpecs,
            VisionHardwareFactory hardwareFactory) {

        this.fieldLayout         = fieldLayout;
        this.fieldLengthMeters   = fieldLengthMeters;
        this.fieldWidthMeters    = fieldWidthMeters;
        this.currentPoseSupplier = currentPoseSupplier;
        this.yawRateSupplier     = yawRateSupplier;
        this.measurementConsumer = measurementConsumer;
        this.stdDevModel         = stdDevModel;

        this.shootingTagTable = buildShootingTagTable(FieldCosntants.getShootingValidTagIdentifiers());

        for (VisionEntries.CameraSpecifications spec : cameraSpecs) {
            VisionIO io = hardwareFactory.createVisionHardware(spec, fieldLayout);
            CameraThread ct = new CameraThread(
                    spec.cameraName(),
                    io,
                    spec.baseNoiseLevel(),
                    spec.cameraConfidenceMultiplier());
            cameraThreads.add(ct);
            ct.start();
        }
    }

    // ── SubsystemBase ─────────────────────────────────────────────────────

    /**
     * Called every 20 ms by the robot loop.
     * <b>ZERO</b> PhotonVision / NetworkTables calls happen here.
     * All work is just reading volatile fields and doing arithmetic.
     */
    @Override
    public void periodic() {
        if (!visionEnabled || cameraThreads.isEmpty() || fieldLayout == null) {
            shootingTargetValidThisCycle = false;
            return;
        }

        Pose2d  currentPose2d  = currentPoseSupplier.get();
        Pose3d  referencePose3d = new Pose3d(currentPose2d);
        double  nowSeconds      = Timer.getFPGATimestamp();
        double  yawRate         = yawRateSupplier.get();

        boolean foundShootingTag = false;
        int measurementsAccepted = 0;
        int measurementsRejected = 0;

        for (CameraThread cam : cameraThreads) {
            // Feed latest odometry pose to camera thread (cheap volatile write)
            cam.setReferencePose(referencePose3d);

            VisionIO.VisionIOInputs inputs = cam.getInputs();

            // ── Shooting-tag detection ────────────────────────────────────
            if (!foundShootingTag) {
                foundShootingTag = hasAnyShootingTag(
                        inputs.detectedTagIdentifiers,
                        inputs.detectedTagCount,
                        shootingTagTable);
            }

            // ── Pose measurement ──────────────────────────────────────────
            if (inputs.observationTagCount <= 0 || inputs.observationRobotPose == null) {
                continue;
            }

            VisionEnums.VisionRejectReason reject = stdDevModel.getRejectReason(
                    inputs, nowSeconds, yawRate, fieldLengthMeters, fieldWidthMeters);

            if (reject != null) {
                measurementsRejected++;
                
                // ✅ DEBUG: Log rejection reason every 1 second
                if (nowSeconds - lastDebugLogTime > 1.0) {
                    DriverStation.reportWarning(
                        "[Vision/" + cam.getCameraName() + "] REJECTED: " + reject + 
                        " | yawRate=" + String.format("%.3f", yawRate) +
                        " | distance=" + String.format("%.2f", inputs.observationAverageTagDistanceMeters) + "m" +
                        " | tagCount=" + inputs.observationTagCount,
                        false);
                }
                continue;
            }

            measurementsAccepted++;
            Matrix<N3, N1> baseStdDevs = cam.getBaseNoiseLevel().getBaseStandardDeviationMatrix();
            Matrix<N3, N1> stdDevs     = stdDevModel.calculateStdDevs(
                    inputs, baseStdDevs, cam.getCameraConfidenceMultiplier());

            // ✅ DEBUG: Log accepted measurement
            if (nowSeconds - lastDebugLogTime > 1.0) {
                DriverStation.reportWarning(
                    "[Vision/" + cam.getCameraName() + "] ACCEPTED: pose=" + inputs.observationRobotPose.toPose2d(),
                    false);
            }

            measurementConsumer.addVisionMeasurement(
                    inputs.observationRobotPose.toPose2d(),
                    inputs.observationTimestampSeconds,
                    stdDevs);
        }

        // ✅ DEBUG: Log summary every 1 second
        if (nowSeconds - lastDebugLogTime > 1.0) {
            DriverStation.reportWarning(
                "[Vision Summary] Accepted=" + measurementsAccepted + 
                " Rejected=" + measurementsRejected + 
                " ShootingTarget=" + foundShootingTag +
                " YawRate=" + String.format("%.3f", yawRate) + " rad/s",
                false);
            lastDebugLogTime = nowSeconds;
        }

        shootingTargetValidThisCycle = foundShootingTag;
    }

    // ── Public API ────────────────────────────────────────────────────────

    public boolean hasTarget() {
        for (CameraThread cam : cameraThreads) {
            if (cam.getInputs().hasTarget) return true;
        }
        return false;
    }

    /** Yaw of the best target from the first camera (useful for aiming). */
    public double getLatestTargetYawRadians() {
        return cameraThreads.isEmpty() ? 0.0 : cameraThreads.get(0).getInputs().latestTargetYawRadians;
    }

    /** {@code true} if a valid shooting AprilTag is visible this cycle. */
    public boolean isShootingTargetValid() {
        return shootingTargetValidThisCycle;
    }

    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
    }

    public void setDriverMode(boolean enabled) {
        cameraThreads.forEach(c -> c.setDriverMode(enabled));
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    // ── Private helpers ───────────────────────────────────────────────────

    private static boolean[] buildShootingTagTable(long[] tagIds) {
        if (tagIds == null || tagIds.length == 0) return new boolean[0];

        int max = 0;
        for (long id : tagIds) {
            if (id >= 0 && id < Integer.MAX_VALUE) max = Math.max(max, (int) id);
        }

        boolean[] table = new boolean[max + 1];
        for (long id : tagIds) {
            if (id >= 0 && id <= max) table[(int) id] = true;
        }
        return table;
    }

    private static boolean hasAnyShootingTag(long[] tagIds, int count, boolean[] table) {
        if (table == null || table.length == 0) return false;

        for (int i = 0; i < count; i++) {
            long id = tagIds[i];
            if (id < 0) break;
            if (id < table.length && table[(int) id]) return true;
        }
        return false;
    }
}
