package frc.robot.subsystems.Drive;

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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
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

        poseEstimator1 =
            new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.kRobotToCam);

        poseEstimator2 =
            new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.kRobotToCam2);

        poseEstimator3 =
            new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.kRobotToCam3);

        poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

   /**
 * Called from the Drive subsystem periodic
 */
public void updateVision(CommandSwerveDrivetrain driveEstimator) {

    processCamera(camera1, poseEstimator1, driveEstimator);
    processCamera(camera2, poseEstimator2, driveEstimator);
    processCamera(camera3, poseEstimator3, driveEstimator);
}

/**
 * Processes a single camera measurement
 */
private void processCamera(
    PhotonCamera camera,
    PhotonPoseEstimator estimator,
    CommandSwerveDrivetrain driveEstimator) {  // <--- changed type here

    PhotonPipelineResult result = camera.getLatestResult();

    if (!result.hasTargets())
        return;

    Optional<EstimatedRobotPose> estimate = estimator.update(result);

    if (estimate.isEmpty())
        return;

    Pose2d visionPose = estimate.get().estimatedPose.toPose2d();
    double timestamp = result.getTimestampSeconds();

    // Reject high ambiguity
    if (result.getBestTarget().getPoseAmbiguity() > 0.2)
        return;

    // Reject impossible jumps
    Pose2d currentPose = driveEstimator.getPose();  // <--- use getPose() from CommandSwerveDrivetrain

    // if (visionPose
    //         .getTranslation()
    //         .getDistance(currentPose.getTranslation())
    //     > 2.0)
    //     return;

    Matrix<N3, N1> stdDevs = getEstimationStdDevs(result, visionPose);

    driveEstimator.addVisionMeasurement(
        visionPose,
        timestamp,
        stdDevs);
}
    /**
     * Calculates measurement noise based on tag count and distance
     */
    public Matrix<N3, N1> getEstimationStdDevs(
        PhotonPipelineResult result,
        Pose2d estimatedPose) {

        var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;

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
            estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;

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