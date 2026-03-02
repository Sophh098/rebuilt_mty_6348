// File: src/main/java/frc/SuperSubsystem/SuperVision/VisionIOPhotonVision.java
package frc.robot.Vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOPhotonVision implements VisionIO {

    protected final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    private VisionEnums.PoseEstimationMode poseEstimationMode =
            VisionEnums.PoseEstimationMode.COPROCESSOR_MULTI_TAG;

    private Pose3d referencePoseForEstimation = new Pose3d();

    private double lastFramesPerSecondWindowStartTimestampSeconds = Timer.getFPGATimestamp();
    private long framesProcessedInCurrentWindow = 0;

    private static final double[] EMPTY_DOUBLE_ARRAY = new double[0];
    private static final Pose3d[] EMPTY_POSE3D_ARRAY = new Pose3d[0];
    private static final double[] EMPTY_AMBIGUITY_ARRAY = new double[0];
    private static final long[] EMPTY_LONG_ARRAY = new long[0];
    private static final boolean[] EMPTY_BOOLEAN_ARRAY = new boolean[0];

    public VisionIOPhotonVision(
            String cameraName,
            Transform3d robotToCameraTransform3d,
            AprilTagFieldLayout aprilTagFieldLayout
    ) {
        this.photonCamera = new PhotonCamera(cameraName);

        // 2026+ supported constructor (fieldTags, robotToCamera)
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCameraTransform3d);
    }

    public void setPoseEstimationMode(VisionEnums.PoseEstimationMode poseEstimationMode) {
        this.poseEstimationMode = poseEstimationMode;
    }

    @Override
    public void setReferencePoseForEstimation(Pose3d referencePose) {
        this.referencePoseForEstimation = referencePose;
    }

    @Override
    public void setDriverMode(boolean driverModeEnabled) {
        photonCamera.setDriverMode(driverModeEnabled);
    }

    @Override
    public void updateInputs(VisionIOInputs visionInputs) {
        visionInputs.cameraConnected = photonCamera.isConnected();
        visionInputs.photonPoseEstimatorEnabled = true;

        List<PhotonPipelineResult> unreadPipelineResultList = photonCamera.getAllUnreadResults();

        if (unreadPipelineResultList == null || unreadPipelineResultList.isEmpty()) {
            visionInputs.hasTarget = false;

            visionInputs.observationTimestampsSeconds = EMPTY_DOUBLE_ARRAY;
            visionInputs.observationRobotPoses = EMPTY_POSE3D_ARRAY;
            visionInputs.observationAmbiguities = EMPTY_AMBIGUITY_ARRAY;
            visionInputs.observationTagCounts = EMPTY_LONG_ARRAY;
            visionInputs.observationAverageTagDistanceMeters = EMPTY_DOUBLE_ARRAY;
            visionInputs.observationRotationTrusted = EMPTY_BOOLEAN_ARRAY;
            visionInputs.observationTypeOrdinals = EMPTY_LONG_ARRAY;
            visionInputs.detectedTagIdentifiers = EMPTY_LONG_ARRAY;

            updateFramesPerSecondEstimate(visionInputs, false);
            return;
        }

        PhotonPipelineResult newestPipelineResult =
                unreadPipelineResultList.get(unreadPipelineResultList.size() - 1);

        boolean hasTargetThisCycle = false;
        Set<Integer> detectedTagIdentifierSet = new HashSet<>();

        List<Double> observationTimestampSecondsList = new ArrayList<>(1);
        List<Pose3d> observationRobotPoseList = new ArrayList<>(1);
        List<Double> observationAmbiguityList = new ArrayList<>(1);
        List<Long> observationTagCountList = new ArrayList<>(1);
        List<Double> observationAverageTagDistanceMetersList = new ArrayList<>(1);
        List<Boolean> observationRotationTrustedList = new ArrayList<>(1);
        List<Long> observationTypeOrdinalList = new ArrayList<>(1);

        if (newestPipelineResult != null && newestPipelineResult.hasTargets()) {
            hasTargetThisCycle = true;

            var bestTrackedTarget = newestPipelineResult.getBestTarget();
            visionInputs.latestTargetYawRadians = Math.toRadians(bestTrackedTarget.getYaw());
            visionInputs.latestTargetPitchRadians = Math.toRadians(bestTrackedTarget.getPitch());

            for (var trackedTarget : newestPipelineResult.getTargets()) {
                detectedTagIdentifierSet.add(trackedTarget.getFiducialId());
            }
        } else {
            visionInputs.latestTargetYawRadians = 0.0;
            visionInputs.latestTargetPitchRadians = 0.0;
        }

        Optional<EstimatedRobotPose> estimatedRobotPoseOptional =
                (newestPipelineResult == null)
                        ? Optional.empty()
                        : estimateRobotPoseWithFallback(newestPipelineResult);

        if (estimatedRobotPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedRobotPose = estimatedRobotPoseOptional.get();
            Pose3d estimatedRobotPose3d = estimatedRobotPose.estimatedPose;

            int usedTagCount =
                    (estimatedRobotPose.targetsUsed != null)
                            ? estimatedRobotPose.targetsUsed.size()
                            : 0;

            if (usedTagCount > 0) {
                double totalDistanceMeters = 0.0;

                for (var usedTrackedTarget : estimatedRobotPose.targetsUsed) {
                    detectedTagIdentifierSet.add(usedTrackedTarget.getFiducialId());
                    totalDistanceMeters += usedTrackedTarget.getBestCameraToTarget().getTranslation().getNorm();
                }

                double averageDistanceMeters = totalDistanceMeters / usedTagCount;

                double poseAmbiguity = 0.0;
                if (usedTagCount == 1) {
                    poseAmbiguity = estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity();
                }

                boolean isRotationTrusted = (usedTagCount >= 2);

                VisionEnums.PoseObservationType observationType =
                        (usedTagCount >= 2)
                                ? VisionEnums.PoseObservationType.PHOTONVISION_MULTI_TAG
                                : VisionEnums.PoseObservationType.PHOTONVISION_SINGLE_TAG;

                observationTimestampSecondsList.add(estimatedRobotPose.timestampSeconds);
                observationRobotPoseList.add(estimatedRobotPose3d);
                observationAmbiguityList.add(poseAmbiguity);
                observationTagCountList.add((long) usedTagCount);
                observationAverageTagDistanceMetersList.add(averageDistanceMeters);
                observationRotationTrustedList.add(isRotationTrusted);
                observationTypeOrdinalList.add((long) observationType.ordinal());
            }
        }

        visionInputs.hasTarget = hasTargetThisCycle;

        visionInputs.observationTimestampsSeconds = new double[observationTimestampSecondsList.size()];
        for (int index = 0; index < observationTimestampSecondsList.size(); index++) {
            visionInputs.observationTimestampsSeconds[index] = observationTimestampSecondsList.get(index);
        }

        visionInputs.observationRobotPoses = observationRobotPoseList.toArray(Pose3d[]::new);

        visionInputs.observationAmbiguities = new double[observationAmbiguityList.size()];
        for (int index = 0; index < observationAmbiguityList.size(); index++) {
            visionInputs.observationAmbiguities[index] = observationAmbiguityList.get(index);
        }

        visionInputs.observationTagCounts = new long[observationTagCountList.size()];
        for (int index = 0; index < observationTagCountList.size(); index++) {
            visionInputs.observationTagCounts[index] = observationTagCountList.get(index);
        }

        visionInputs.observationAverageTagDistanceMeters = new double[observationAverageTagDistanceMetersList.size()];
        for (int index = 0; index < observationAverageTagDistanceMetersList.size(); index++) {
            visionInputs.observationAverageTagDistanceMeters[index] = observationAverageTagDistanceMetersList.get(index);
        }

        visionInputs.observationRotationTrusted = new boolean[observationRotationTrustedList.size()];
        for (int index = 0; index < observationRotationTrustedList.size(); index++) {
            visionInputs.observationRotationTrusted[index] = observationRotationTrustedList.get(index);
        }

        visionInputs.observationTypeOrdinals = new long[observationTypeOrdinalList.size()];
        for (int index = 0; index < observationTypeOrdinalList.size(); index++) {
            visionInputs.observationTypeOrdinals[index] = observationTypeOrdinalList.get(index);
        }

        long[] detectedTagIdentifierArray = new long[detectedTagIdentifierSet.size()];
        int detectedTagIndex = 0;
        for (int detectedTagIdentifier : detectedTagIdentifierSet) {
            detectedTagIdentifierArray[detectedTagIndex++] = detectedTagIdentifier;
        }
        visionInputs.detectedTagIdentifiers = detectedTagIdentifierArray;

        updateFramesPerSecondEstimate(visionInputs, true);
    }

    private Optional<EstimatedRobotPose> estimateRobotPoseWithFallback(PhotonPipelineResult pipelineResult) {
        Optional<EstimatedRobotPose> coprocessorMultiTagEstimate =
                photonPoseEstimator.estimateCoprocMultiTagPose(pipelineResult);

        if (coprocessorMultiTagEstimate.isPresent()) {
            return coprocessorMultiTagEstimate;
        }

        return switch (poseEstimationMode) {
            case COPROCESSOR_MULTI_TAG -> Optional.empty();
            case LOWEST_AMBIGUITY -> photonPoseEstimator.estimateLowestAmbiguityPose(pipelineResult);
            case CLOSEST_TO_REFERENCE_POSE ->
                    photonPoseEstimator.estimateClosestToReferencePose(pipelineResult, referencePoseForEstimation);
            case AVERAGE_BEST_TARGETS -> photonPoseEstimator.estimateAverageBestTargetsPose(pipelineResult);
        };
    }

    private void updateFramesPerSecondEstimate(VisionIOInputs visionInputs, boolean processedFrameThisCycle) {
        double currentTimestampSeconds = Timer.getFPGATimestamp();

        if (processedFrameThisCycle) {
            framesProcessedInCurrentWindow++;
        }

        double windowDurationSeconds = currentTimestampSeconds - lastFramesPerSecondWindowStartTimestampSeconds;
        if (windowDurationSeconds >= 1.0) {
            visionInputs.framesPerSecond = framesProcessedInCurrentWindow;
            framesProcessedInCurrentWindow = 0;
            lastFramesPerSecondWindowStartTimestampSeconds = currentTimestampSeconds;
        }
    }
}