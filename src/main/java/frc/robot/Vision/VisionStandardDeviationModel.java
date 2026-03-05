package frc.robot.Vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Decides whether a vision observation should be accepted and, if so,
 * what standard deviations to assign to it.
 *
 * <p>All math uses primitives. The only allocation is the final
 * {@link MatBuilder#fill} call, which WPILib requires. Everything else
 * is computed in-place — no intermediate objects.
 */
public final class VisionStandardDeviationModel {

    // ── Rejection thresholds ──────────────────────────────────────────────
    private final double maxAmbiguityForSingleTag;
    private final double maxZErrorMeters;
    private final double maxObservationAgeSeconds;
    private final double maxDistanceSingleTagMeters;
    private final double maxDistanceMultiTagMeters;
    private final double maxYawRateRadiansPerSecond;

    // ── Std-dev caps ──────────────────────────────────────────────────────
    private final double maxLinearStdDevMeters;
    private final double maxAngularStdDevRadians;

    /**
     * Constructs the model with all tuning parameters.
     *
     * @param maxAmbiguityForSingleTag       reject single-tag if ambiguity > this (0–1)
     * @param maxZErrorMeters                reject if robot Z estimate exceeds ±this
     * @param maxObservationAgeSeconds       reject if timestamp is older than this
     * @param maxDistanceSingleTagMeters     reject single-tag beyond this range
     * @param maxDistanceMultiTagMeters      reject multi-tag beyond this range
     * @param maxYawRateRadiansPerSecond     reject while spinning faster than this (0 = disabled)
     * @param maxLinearStdDevMeters          hard cap on X/Y std dev (metres)
     * @param maxAngularStdDevRadians        hard cap on θ std dev (radians)
     */
    public VisionStandardDeviationModel(
            double maxAmbiguityForSingleTag,
            double maxZErrorMeters,
            double maxObservationAgeSeconds,
            double maxDistanceSingleTagMeters,
            double maxDistanceMultiTagMeters,
            double maxYawRateRadiansPerSecond,
            double maxLinearStdDevMeters,
            double maxAngularStdDevRadians) {

        this.maxAmbiguityForSingleTag      = maxAmbiguityForSingleTag;
        this.maxZErrorMeters               = maxZErrorMeters;
        this.maxObservationAgeSeconds      = maxObservationAgeSeconds;
        this.maxDistanceSingleTagMeters    = maxDistanceSingleTagMeters;
        this.maxDistanceMultiTagMeters     = maxDistanceMultiTagMeters;
        this.maxYawRateRadiansPerSecond    = maxYawRateRadiansPerSecond;
        this.maxLinearStdDevMeters         = maxLinearStdDevMeters;
        this.maxAngularStdDevRadians       = maxAngularStdDevRadians;
    }

    /**
     * Returns the first reason to reject this observation, or {@code null} if it
     * should be accepted. Evaluated in order of cheapest → most expensive check.
     */
    public VisionEnums.VisionRejectReason getRejectReason(
            VisionIO.VisionIOInputs inputs,
            double currentTimestampSeconds,
            double yawRateRadiansPerSecond,
            double fieldLengthMeters,
            double fieldWidthMeters) {

        int tagCount = inputs.observationTagCount;
        if (tagCount <= 0)
            return VisionEnums.VisionRejectReason.NO_TAGS;

        double ageSeconds = currentTimestampSeconds - inputs.observationTimestampSeconds;
        if (ageSeconds < -0.05 || ageSeconds > maxObservationAgeSeconds)
            return VisionEnums.VisionRejectReason.OBSERVATION_TOO_OLD;

        if (tagCount == 1 && inputs.observationAmbiguity > maxAmbiguityForSingleTag)
            return VisionEnums.VisionRejectReason.SINGLE_TAG_AMBIGUITY_TOO_HIGH;

        var pose = inputs.observationRobotPose;
        if (pose == null)
            return VisionEnums.VisionRejectReason.NO_TAGS;

        if (Math.abs(pose.getZ()) > maxZErrorMeters)
            return VisionEnums.VisionRejectReason.ROBOT_POSE_Z_OUT_OF_RANGE;

        double x = pose.getX(), y = pose.getY();
        if (x < 0 || x > fieldLengthMeters || y < 0 || y > fieldWidthMeters)
            return VisionEnums.VisionRejectReason.OUTSIDE_FIELD_BOUNDS;

        if (maxYawRateRadiansPerSecond > 0.0
                && Math.abs(yawRateRadiansPerSecond) > maxYawRateRadiansPerSecond)
            return VisionEnums.VisionRejectReason.ROBOT_ROTATING_TOO_FAST;

        double dist = inputs.observationAverageTagDistanceMeters;
        if (tagCount == 1 && dist > maxDistanceSingleTagMeters)
            return VisionEnums.VisionRejectReason.TOO_FAR_FOR_SINGLE_TAG;
        if (tagCount >= 2 && dist > maxDistanceMultiTagMeters)
            return VisionEnums.VisionRejectReason.TOO_FAR_FOR_MULTI_TAG;

        return null; // accepted
    }

    /**
     * Calculates measurement standard deviations for an accepted observation.
     * Scale = dist² / tagCount, with a multi-tag bonus and a distance ramp past 4 m.
     * Rotation is set to 1e6 (effectively ignored) when only one tag is visible.
     */
    public Matrix<N3, N1> calculateStdDevs(
            VisionIO.VisionIOInputs inputs,
            Matrix<N3, N1> baseStdDevs,
            double confidenceMultiplier) {

        double dist     = Math.max(0.001, inputs.observationAverageTagDistanceMeters);
        int    tagCount = Math.max(1, inputs.observationTagCount);

        double distSqPerTag   = (dist * dist) / tagCount;
        double multiTagBonus  = (tagCount >= 2) ? 0.7 : 1.0;
        double distRamp       = (dist > 4.0) ? (1.0 + Math.min(1.0, (dist - 4.0) / 2.0)) : 1.0;
        double scale          = distSqPerTag * multiTagBonus * distRamp * confidenceMultiplier;

        double xSd  = Math.min(baseStdDevs.get(0, 0) * scale, maxLinearStdDevMeters);
        double ySd  = Math.min(baseStdDevs.get(1, 0) * scale, maxLinearStdDevMeters);
        double rSd;

        if (!inputs.observationRotationTrusted) {
            rSd = 1e6; // effectively ignore rotation — single tag
        } else {
            rSd = Math.min(baseStdDevs.get(2, 0) * scale, maxAngularStdDevRadians);
        }

        return MatBuilder.fill(Nat.N3(), Nat.N1(), xSd, ySd, rSd);
    }
}