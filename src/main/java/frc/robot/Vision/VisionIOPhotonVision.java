package frc.robot.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOPhotonVision implements VisionIO {

  private static final long noFiducialIdentifierSentinel = -1L;

  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator photonPoseEstimator;

  private VisionEnums.PoseEstimationMode poseEstimationMode =
      VisionEnums.PoseEstimationMode.COPROCESSOR_MULTI_TAG;

  private Pose3d referencePoseForEstimation = new Pose3d();

  private double lastFramesPerSecondWindowStartTimestampSeconds = Timer.getFPGATimestamp();
  private long drainedPipelineResultsInCurrentWindow = 0;

  private final int cameraFramesPerSecondLimit;
  private final boolean enableRobotControllerFallbackPoseEstimation;

  public VisionIOPhotonVision(
      String cameraName,
      Transform3d robotToCameraTransform3d,
      AprilTagFieldLayout aprilTagFieldLayout
  ) {
    photonCamera = new PhotonCamera(cameraName);
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCameraTransform3d);

    // Clave para 4 cámaras: evita backlog y costo variable por getAllUnreadResults()
    cameraFramesPerSecondLimit = 20;
    photonCamera.setFPSLimit(cameraFramesPerSecondLimit);

    // Máxima velocidad: false (solo coprocessor multitag).
    // Si quieres “salvar” pose cuando solo hay 1 tag visible: true.
    enableRobotControllerFallbackPoseEstimation = false;
  }

  public void setPoseEstimationMode(VisionEnums.PoseEstimationMode newPoseEstimationMode) {
    poseEstimationMode = newPoseEstimationMode;
  }

  @Override
  public void setReferencePoseForEstimation(Pose3d referencePose) {
    referencePoseForEstimation = referencePose;
  }

  @Override
  public void setDriverMode(boolean driverModeEnabled) {
    photonCamera.setDriverMode(driverModeEnabled);
  }

  @Override
  public void updateInputs(VisionIOInputs visionInputs) {
    visionInputs.cameraConnected = photonCamera.isConnected();
    visionInputs.photonPoseEstimatorEnabled = true;

    // Reset outputs (NO reasignamos arreglos porque son final)
    visionInputs.hasTarget = false;
    visionInputs.latestTargetYawRadians = 0.0;
    visionInputs.latestTargetPitchRadians = 0.0;

    // "No observation" marker
    visionInputs.observationTagCounts[0] = 0;
    visionInputs.observationTimestampsSeconds[0] = 0.0;
    visionInputs.observationRobotPoses[0] = null;
    visionInputs.observationAmbiguities[0] = 0.0;
    visionInputs.observationAverageTagDistanceMeters[0] = 0.0;
    visionInputs.observationRotationTrusted[0] = false;
    visionInputs.observationTypeOrdinals[0] = 0;

    clearDetectedTagIdentifiers(visionInputs);

    List<PhotonPipelineResult> unreadPipelineResults = photonCamera.getAllUnreadResults();
    int unreadPipelineResultCount = unreadPipelineResults.size();

    if (unreadPipelineResultCount == 0) {
      updateFramesPerSecondEstimate(visionInputs, 0);
      return;
    }

    PhotonPipelineResult newestResultWithTargets = selectNewestResultWithTargets(unreadPipelineResults);
    if (newestResultWithTargets == null) {
      updateFramesPerSecondEstimate(visionInputs, unreadPipelineResultCount);
      return;
    }

    visionInputs.hasTarget = true;

    var bestTrackedTarget = newestResultWithTargets.getBestTarget();
    visionInputs.latestTargetYawRadians = Math.toRadians(bestTrackedTarget.getYaw());
    visionInputs.latestTargetPitchRadians = Math.toRadians(bestTrackedTarget.getPitch());

    fillDetectedTagIdentifiers(visionInputs, newestResultWithTargets);

    Optional<EstimatedRobotPose> estimatedRobotPoseOptional = estimateRobotPose(newestResultWithTargets);
    if (estimatedRobotPoseOptional.isPresent()) {
      writeSingleObservationToInputs(visionInputs, estimatedRobotPoseOptional.get());
    }

    updateFramesPerSecondEstimate(visionInputs, unreadPipelineResultCount);
  }

  private static PhotonPipelineResult selectNewestResultWithTargets(List<PhotonPipelineResult> unreadPipelineResults) {
    for (int resultIndex = unreadPipelineResults.size() - 1; resultIndex >= 0; resultIndex--) {
      PhotonPipelineResult candidateResult = unreadPipelineResults.get(resultIndex);
      if (candidateResult != null && candidateResult.hasTargets()) {
        return candidateResult;
      }
    }
    return null;
  }

  private static void clearDetectedTagIdentifiers(VisionIOInputs visionInputs) {
    for (int index = 0; index < visionInputs.detectedTagIdentifiers.length; index++) {
      visionInputs.detectedTagIdentifiers[index] = noFiducialIdentifierSentinel;
    }
  }

  private static void fillDetectedTagIdentifiers(VisionIOInputs visionInputs, PhotonPipelineResult pipelineResult) {
    int writeIndex = 0;
    int capacity = visionInputs.detectedTagIdentifiers.length;

    for (var trackedTarget : pipelineResult.getTargets()) {
      if (writeIndex >= capacity) {
        break;
      }

      int fiducialIdentifier = trackedTarget.getFiducialId();
      if (fiducialIdentifier < 0) {
        continue;
      }

      visionInputs.detectedTagIdentifiers[writeIndex++] = fiducialIdentifier;
    }

    // Sentinel already set for the rest because we cleared the array first
  }

  private Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult pipelineResult) {
    Optional<EstimatedRobotPose> coprocessorMultiTagEstimate =
        photonPoseEstimator.estimateCoprocMultiTagPose(pipelineResult);

    if (coprocessorMultiTagEstimate.isPresent()) {
      return coprocessorMultiTagEstimate;
    }

    if (!enableRobotControllerFallbackPoseEstimation) {
      return Optional.empty();
    }

    return switch (poseEstimationMode) {
      case COPROCESSOR_MULTI_TAG -> Optional.empty();
      case LOWEST_AMBIGUITY -> photonPoseEstimator.estimateLowestAmbiguityPose(pipelineResult);
      case CLOSEST_TO_REFERENCE_POSE ->
          photonPoseEstimator.estimateClosestToReferencePose(pipelineResult, referencePoseForEstimation);
      case AVERAGE_BEST_TARGETS -> photonPoseEstimator.estimateAverageBestTargetsPose(pipelineResult);
    };
  }

  private static void writeSingleObservationToInputs(VisionIOInputs visionInputs, EstimatedRobotPose estimatedRobotPose) {
    int usedTagCount =
        (estimatedRobotPose.targetsUsed != null) ? estimatedRobotPose.targetsUsed.size() : 0;

    if (usedTagCount <= 0) {
      visionInputs.observationTagCounts[0] = 0;
      return;
    }

    double totalDistanceMeters = 0.0;
    for (var usedTrackedTarget : estimatedRobotPose.targetsUsed) {
      totalDistanceMeters += usedTrackedTarget.getBestCameraToTarget().getTranslation().getNorm();
    }

    double averageDistanceMeters = totalDistanceMeters / usedTagCount;

    double poseAmbiguity = 0.0;
    if (usedTagCount == 1) {
      poseAmbiguity = estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity();
    }

    boolean rotationTrusted = (usedTagCount >= 2);

    VisionEnums.PoseObservationType observationType =
        (usedTagCount >= 2)
            ? VisionEnums.PoseObservationType.PHOTONVISION_MULTI_TAG
            : VisionEnums.PoseObservationType.PHOTONVISION_SINGLE_TAG;

    visionInputs.observationTimestampsSeconds[0] = estimatedRobotPose.timestampSeconds;
    visionInputs.observationRobotPoses[0] = estimatedRobotPose.estimatedPose;
    visionInputs.observationAmbiguities[0] = poseAmbiguity;
    visionInputs.observationTagCounts[0] = usedTagCount;
    visionInputs.observationAverageTagDistanceMeters[0] = averageDistanceMeters;
    visionInputs.observationRotationTrusted[0] = rotationTrusted;
    visionInputs.observationTypeOrdinals[0] = observationType.ordinal();
  }

  private void updateFramesPerSecondEstimate(VisionIOInputs visionInputs, int drainedResultCountThisCycle) {
    double currentTimestampSeconds = Timer.getFPGATimestamp();

    drainedPipelineResultsInCurrentWindow += drainedResultCountThisCycle;

    double windowDurationSeconds = currentTimestampSeconds - lastFramesPerSecondWindowStartTimestampSeconds;
    if (windowDurationSeconds >= 1.0) {
      visionInputs.framesPerSecond = drainedPipelineResultsInCurrentWindow;
      drainedPipelineResultsInCurrentWindow = 0;
      lastFramesPerSecondWindowStartTimestampSeconds = currentTimestampSeconds;
    }
  }
}