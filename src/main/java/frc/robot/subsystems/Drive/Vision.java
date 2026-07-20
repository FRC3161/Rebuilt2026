package frc.robot.subsystems.Drive;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

public class Vision {

    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonCamera camera3;

    private final PhotonPoseEstimator poseEstimator1;
    private final PhotonPoseEstimator poseEstimator2;
    private final PhotonPoseEstimator poseEstimator3;

    private final AprilTagFieldLayout tagLayout;

    public Vision() {
        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        camera1 = new PhotonCamera(VisionConstants.cameraName);
        camera2 = new PhotonCamera(VisionConstants.camera2Name);
        camera3 = new PhotonCamera(VisionConstants.camera3Name);

        poseEstimator1 = new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.kRobotToCam);

        poseEstimator2 = new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.kRobotToCam2);

        poseEstimator3 = new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.kRobotToCam3);

        poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /** Called from the drivetrain's periodic. */
    public void updateVision(CommandSwerveDrivetrain driveEstimator) {
        processCamera(camera1, poseEstimator1, driveEstimator);
        processCamera(camera2, poseEstimator2, driveEstimator);
        processCamera(camera3, poseEstimator3, driveEstimator);
    }

    /**
     * Feeds every frame received since the last loop into the pose estimator.
     * Using getAllUnreadResults (rather than getLatestResult) guarantees each
     * frame is applied exactly once.
     */
    private void processCamera(
            PhotonCamera camera,
            PhotonPoseEstimator estimator,
            CommandSwerveDrivetrain driveEstimator) {

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets())
                continue;

            Optional<EstimatedRobotPose> estimate = estimator.update(result);
            if (estimate.isEmpty())
                continue;

            // Ambiguity is only meaningful for single-tag solves; multi-tag
            // PNP reports -1.
            if (result.getTargets().size() == 1
                    && result.getBestTarget().getPoseAmbiguity() > 0.2)
                continue;

            Pose2d visionPose = estimate.get().estimatedPose.toPose2d();
            double timestamp = result.getTimestampSeconds();
            Matrix<N3, N1> stdDevs = getEstimationStdDevs(result, visionPose);

            driveEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);

            // Skip rejected/very-low-confidence fixes (huge std devs) — they'd
            // just inject noise into the slip comparison rather than signal.
            if (stdDevs.get(0, 0) != Double.MAX_VALUE) {
                driveEstimator.slipDetection.reportVisionUpdate(visionPose.getTranslation());
            }
        }
    }

    /** Calculates measurement noise based on tag count and distance. */
    public Matrix<N3, N1> getEstimationStdDevs(
            PhotonPipelineResult result,
            Pose2d estimatedPose) {

        var estStdDevs = VisionConstants.kSingleTagStdDevs;

        int numTags = 0;
        double avgDist = 0;

        for (var tgt : result.getTargets()) {
            var tagPose = tagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;

            numTags++;
            avgDist += tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0)
            return estStdDevs;

        avgDist /= numTags;

        // Multiple tags are more accurate
        if (numTags > 1)
            estStdDevs = VisionConstants.kMultiTagStdDevs;

        // Reject single-tag far measurements
        if (numTags == 1 && avgDist > 4)
            return VecBuilder.fill(
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE);

        // Scale noise with distance
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 20));

        return estStdDevs;
    }
}
