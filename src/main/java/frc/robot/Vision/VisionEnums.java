package frc.robot.Vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionEnums {
    private VisionEnums() {}

    public enum PoseEstimationMode {
        COPROCESSOR_MULTI_TAG,
        LOWEST_AMBIGUITY,
        CLOSEST_TO_REFERENCE_POSE,
        AVERAGE_BEST_TARGETS
    }

    public enum PoseObservationType {
        PHOTONVISION_MULTI_TAG,
        PHOTONVISION_SINGLE_TAG
    }

    /** Base noise per camera; later scaled dynamically by distance/tag count. */
    public enum PoseEstimateNoiseLevel {
        LOW(0.025, 0.025, 0.025),
        MEDIUM(0.15, 0.15, 0.15),
        HIGH(0.30, 0.30, 0.30),
        NONE(99.0, 99.0, 99.0),

        FR_CAM(0.3,0.3,0.3),
        FL_CAM(0.3,0.3,0.3),
        R_CAM(0.3,0.3,0.3),
        L_CAM(0.3,0.3,0.3);

        private final Matrix<N3, N1> baseStandardDeviationMatrix;

        PoseEstimateNoiseLevel(
                double xStandardDeviationMeters,
                double yStandardDeviationMeters,
                double rotationStandardDeviationRadians
        ) {
            this.baseStandardDeviationMatrix = MatBuilder.fill(
                    Nat.N3(),
                    Nat.N1(),
                    xStandardDeviationMeters,
                    yStandardDeviationMeters,
                    rotationStandardDeviationRadians
            );
        }

        public Matrix<N3, N1> getBaseStandardDeviationMatrix() {
            return baseStandardDeviationMatrix;
        }
    }

    public enum VisionRejectReason {
        NO_TAGS,
        SINGLE_TAG_AMBIGUITY_TOO_HIGH,
        ROBOT_POSE_Z_OUT_OF_RANGE,
        OUTSIDE_FIELD_BOUNDS,
        OBSERVATION_TOO_OLD,
        TOO_FAR_FOR_SINGLE_TAG,
        TOO_FAR_FOR_MULTI_TAG,
        ROBOT_ROTATING_TOO_FAST,
        PHOTON_POSE_ESTIMATOR_DISABLED
    }
}