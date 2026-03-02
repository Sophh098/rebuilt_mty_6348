package frc.robot.Vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionStandardDeviationModel {

    private final double maximumAmbiguityForSingleTag;
    private final double maximumZErrorMeters;
    private final double maximumObservationAgeSeconds;

    private final double maximumDistanceForSingleTagMeters;
    private final double maximumDistanceForMultiTagMeters;
    private final double maximumYawRateRadiansPerSecond;

    private final double maximumLinearStandardDeviationMeters;
    private final double maximumAngularStandardDeviationRadians;

    public VisionStandardDeviationModel(
            double maximumAmbiguityForSingleTag,
            double maximumZErrorMeters,
            double maximumObservationAgeSeconds,
            double maximumDistanceForSingleTagMeters,
            double maximumDistanceForMultiTagMeters,
            double maximumYawRateRadiansPerSecond,
            double maximumLinearStandardDeviationMeters,
            double maximumAngularStandardDeviationRadians
    ) {
        this.maximumAmbiguityForSingleTag = maximumAmbiguityForSingleTag;
        this.maximumZErrorMeters = maximumZErrorMeters;
        this.maximumObservationAgeSeconds = maximumObservationAgeSeconds;

        this.maximumDistanceForSingleTagMeters = maximumDistanceForSingleTagMeters;
        this.maximumDistanceForMultiTagMeters = maximumDistanceForMultiTagMeters;
        this.maximumYawRateRadiansPerSecond = maximumYawRateRadiansPerSecond;

        this.maximumLinearStandardDeviationMeters = maximumLinearStandardDeviationMeters;
        this.maximumAngularStandardDeviationRadians = maximumAngularStandardDeviationRadians;
    }

    public VisionEnums.VisionRejectReason getRejectReasonOrNull(
            VisionEntries.VisionObservation observation,
            double currentRobotTimestampSeconds,
            double yawRateRadiansPerSecond,
            double fieldLengthMeters,
            double fieldWidthMeters
    ) {
        if (observation.tagCount() <= 0) {
            return VisionEnums.VisionRejectReason.NO_TAGS;
        }

        double observationAgeSeconds = currentRobotTimestampSeconds - observation.timestampSeconds();

        // If timestamps are not time-synced, age can go negative; treat that as invalid.
        if (observationAgeSeconds < -0.05 || observationAgeSeconds > maximumObservationAgeSeconds) {
            return VisionEnums.VisionRejectReason.OBSERVATION_TOO_OLD;
        }

        if (observation.tagCount() == 1 && observation.ambiguity() > maximumAmbiguityForSingleTag) {
            return VisionEnums.VisionRejectReason.SINGLE_TAG_AMBIGUITY_TOO_HIGH;
        }

        if (Math.abs(observation.robotPose().getZ()) > maximumZErrorMeters) {
            return VisionEnums.VisionRejectReason.ROBOT_POSE_Z_OUT_OF_RANGE;
        }

        double xPositionMeters = observation.robotPose().getX();
        double yPositionMeters = observation.robotPose().getY();
        if (xPositionMeters < 0.0
                || xPositionMeters > fieldLengthMeters
                || yPositionMeters < 0.0
                || yPositionMeters > fieldWidthMeters) {
            return VisionEnums.VisionRejectReason.OUTSIDE_FIELD_BOUNDS;
        }

        if (maximumYawRateRadiansPerSecond > 0.0 && Math.abs(yawRateRadiansPerSecond) > maximumYawRateRadiansPerSecond) {
            return VisionEnums.VisionRejectReason.ROBOT_ROTATING_TOO_FAST;
        }

        double distanceMeters = observation.averageTagDistanceMeters();
        if (observation.tagCount() == 1 && distanceMeters > maximumDistanceForSingleTagMeters) {
            return VisionEnums.VisionRejectReason.TOO_FAR_FOR_SINGLE_TAG;
        }
        if (observation.tagCount() >= 2 && distanceMeters > maximumDistanceForMultiTagMeters) {
            return VisionEnums.VisionRejectReason.TOO_FAR_FOR_MULTI_TAG;
        }

        return null;
    }

    public Matrix<N3, N1> calculateStandardDeviations(
            VisionEntries.VisionObservation observation,
            Matrix<N3, N1> baseStandardDeviations,
            double cameraConfidenceMultiplier
    ) {
        double distanceMeters = Math.max(0.001, observation.averageTagDistanceMeters());
        int tagCount = Math.max(1, observation.tagCount());

        double distanceSquaredOverTagCount = (distanceMeters * distanceMeters) / tagCount;

        double multiTagBonusMultiplier = (tagCount >= 2) ? 0.7 : 1.0;

        double distancePenaltyMultiplier = 1.0;
        if (distanceMeters > 4.0) {
            double ramp = Math.min(1.0, (distanceMeters - 4.0) / 2.0);
            distancePenaltyMultiplier = 1.0 + ramp;
        }

        double globalScale =
                distanceSquaredOverTagCount
                        * multiTagBonusMultiplier
                        * distancePenaltyMultiplier
                        * cameraConfidenceMultiplier;

        double xStandardDeviationMeters = baseStandardDeviations.get(0, 0) * globalScale;
        double yStandardDeviationMeters = baseStandardDeviations.get(1, 0) * globalScale;

        // Rotation: if untrusted, make it HUGE and DO NOT clamp it down.
        double rotationStandardDeviationRadians = baseStandardDeviations.get(2, 0) * globalScale;
        if (!observation.visionRotationTrusted()) {
            rotationStandardDeviationRadians = 1e6;
        }

        xStandardDeviationMeters = Math.min(xStandardDeviationMeters, maximumLinearStandardDeviationMeters);
        yStandardDeviationMeters = Math.min(yStandardDeviationMeters, maximumLinearStandardDeviationMeters);

        if (observation.visionRotationTrusted()) {
            rotationStandardDeviationRadians =
                    Math.min(rotationStandardDeviationRadians, maximumAngularStandardDeviationRadians);
        }

        return MatBuilder.fill(
                Nat.N3(),
                Nat.N1(),
                xStandardDeviationMeters,
                yStandardDeviationMeters,
                rotationStandardDeviationRadians
        );
    }
}