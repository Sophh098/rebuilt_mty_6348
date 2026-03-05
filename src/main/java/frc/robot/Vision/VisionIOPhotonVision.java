package frc.robot.Vision;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * PhotonVision implementation of {@link VisionIO}.
 *
 * <p><b>Threading model:</b> {@code updateInputs()} is called by a dedicated
 * background thread inside {@link CameraThread}, NOT by the main robot loop.
 * The main loop only calls {@link CameraThread#readLatestInto(VisionIO.VisionIOInputs)},
 * which is a fast volatile-read with zero allocation.
 *
 * <p>This keeps all PhotonVision I/O (network tables, pose math) off the
 * main 20 ms loop, eliminating loop overtime with 4+ cameras.
 */
public class VisionIOPhotonVision implements VisionIO {

    // ── Camera / estimator ───────────────────────────────────────────────
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    // ── Shared snapshot between camera thread → main loop ───────────────
    // AtomicReference gives us a safe, lock-free single-writer / single-reader swap.
    private final AtomicReference<Snapshot> latestSnapshot = new AtomicReference<>(new Snapshot());

    // ── Pose reference (written by main loop, read by camera thread) ─────
    private volatile Pose3d referencePose = new Pose3d();

    // ── FPS tracking ─────────────────────────────────────────────────────
    private long fpsWindowStart  = System.nanoTime();
    private long fpsWindowFrames = 0;
    private long lastPublishedFps = 0;

    // Pre-allocated mutable snapshot to avoid per-frame allocation
    private final Snapshot workSnapshot = new Snapshot();

    // ── Constants ─────────────────────────────────────────────────────────
    /** Drop multi-tag if it falls outside this Z band (metres). */
    private static final double MAX_Z_ERROR_METERS = 0.75;

    public VisionIOPhotonVision(
            String cameraName,
            Transform3d robotToCameraTransform,
            AprilTagFieldLayout fieldLayout) {

        photonCamera = new PhotonCamera(cameraName);
        photonCamera.setVersionCheckEnabled(false); // saves ~1 ms per call on NT read
        photonCamera.setFPSLimit(20);               // cap backlog; 20 Hz is plenty

        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCameraTransform);
    }

    // ── VisionIO interface ───────────────────────────────────────────────

    @Override
    public void setReferencePose(Pose3d pose) {
        referencePose = pose;                        // volatile write — fast
    }

    @Override
    public void setDriverMode(boolean enabled) {
        photonCamera.setDriverMode(enabled);
    }

    /**
     * Called exclusively from the camera background thread.
     * Reads PhotonVision, runs pose math, stores result in atomic snapshot.
     */
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.cameraConnected = photonCamera.isConnected();

        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        int resultCount = results.size();

        // Track FPS
        fpsWindowFrames += resultCount;
        long nowNs = System.nanoTime();
        if (nowNs - fpsWindowStart >= 1_000_000_000L) {
            lastPublishedFps = fpsWindowFrames;
            fpsWindowFrames  = 0;
            fpsWindowStart   = nowNs;
        }
        inputs.framesPerSecond = lastPublishedFps;

        // Reset observation
        inputs.observationTagCount = 0;
        inputs.hasTarget           = false;
        inputs.detectedTagCount    = 0;

        if (resultCount == 0) return;

        // Use only the newest frame that has targets
        PhotonPipelineResult best = newestWithTargets(results);
        if (best == null) return;

        // ── Aiming data ───────────────────────────────────────────────
        inputs.hasTarget = true;
        PhotonTrackedTarget bestTarget = best.getBestTarget();
        inputs.latestTargetYawRadians   = Math.toRadians(bestTarget.getYaw());
        inputs.latestTargetPitchRadians = Math.toRadians(bestTarget.getPitch());

        // ── Tag IDs ───────────────────────────────────────────────────
        fillTagIdentifiers(inputs, best);

        // ── Pose estimate ─────────────────────────────────────────────
        Optional<EstimatedRobotPose> poseOpt =
                photonPoseEstimator.estimateCoprocMultiTagPose(best);

        if (poseOpt.isEmpty()) {
            // Fallback: single-tag lowest-ambiguity (only if target has low ambiguity)
            if (bestTarget.getPoseAmbiguity() < 0.15) {
                poseOpt = photonPoseEstimator.estimateLowestAmbiguityPose(best);
            }
        }

        if (poseOpt.isEmpty()) return;

        EstimatedRobotPose est = poseOpt.get();
        int usedTags = est.targetsUsed != null ? est.targetsUsed.size() : 0;
        if (usedTags == 0) return;

        // Quick Z sanity — reject wildly wrong estimates right here
        if (Math.abs(est.estimatedPose.getZ()) > MAX_Z_ERROR_METERS) return;

        double totalDist = 0.0;
        double ambiguity = 0.0;

        for (PhotonTrackedTarget t : est.targetsUsed) {
            totalDist += t.getBestCameraToTarget().getTranslation().getNorm();
        }
        if (usedTags == 1) {
            ambiguity = est.targetsUsed.get(0).getPoseAmbiguity();
        }

        inputs.observationTagCount                 = usedTags;
        inputs.observationTimestampSeconds         = est.timestampSeconds;
        inputs.observationRobotPose                = est.estimatedPose;
        inputs.observationAmbiguity                = ambiguity;
        inputs.observationAverageTagDistanceMeters = totalDist / usedTags;
        inputs.observationRotationTrusted          = (usedTags >= 2);
        inputs.observationIsMultiTag               = (usedTags >= 2);
    }

    // ── Private helpers ───────────────────────────────────────────────────

    private static PhotonPipelineResult newestWithTargets(List<PhotonPipelineResult> results) {
        for (int i = results.size() - 1; i >= 0; i--) {
            PhotonPipelineResult r = results.get(i);
            if (r != null && r.hasTargets()) return r;
        }
        return null;
    }

    private static void fillTagIdentifiers(VisionIOInputs inputs, PhotonPipelineResult result) {
        int count = 0;
        int cap   = inputs.detectedTagIdentifiers.length;
        for (PhotonTrackedTarget t : result.getTargets()) {
            if (count >= cap) break;
            int id = t.getFiducialId();
            if (id < 0) continue;
            inputs.detectedTagIdentifiers[count++] = id;
        }
        inputs.detectedTagCount = count;
    }

    // ── Snapshot (unused in this design — kept for future logging) ────────

    private static final class Snapshot {
        // placeholder for future AdvantageKit-style logging
    }
}