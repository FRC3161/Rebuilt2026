// package frc.robot.subsystems.Drive;

// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import frc.robot.Constants;
// import frc.robot.Constants.VisionConstants;

// public class Vision {
//     private final PhotonCamera camera1; // camera facing x
//     private final PhotonCamera camera2; // camera facing y
//     private final PhotonCamera camera3; // camera facing -y
//     private final PhotonPoseEstimator poseEstimator1;
//     private final PhotonPoseEstimator poseEstimator2;
//     private final PhotonPoseEstimator poseEstimator3;
//     private double lastEstTimestamp1 = 0;
//     private double lastEstTimestamp2 = 0;
//     private double lastEstTimestamp3 = 0;
//     public Optional<EstimatedRobotPose> latestVision = Optional.empty();
//     public Optional<EstimatedRobotPose> latestVision2 = Optional.empty();
//     public Optional<EstimatedRobotPose> latestVision3 = Optional.empty();
//     public AprilTagFieldLayout kTagLayout;

//     public Vision() {
//         kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
//         camera1 = new PhotonCamera(VisionConstants.cameraName);
//         camera2 = new PhotonCamera(VisionConstants.camera2Name);
//         camera3 = new PhotonCamera(VisionConstants.camera3Name);

//         poseEstimator1 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 VisionConstants.kRobotToCam);
//         poseEstimator2 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 VisionConstants.kRobotToCam2);
//         poseEstimator3 = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 VisionConstants.kRobotToCam3);
//         poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // TODO: TRY TRIG SOLVE
//         poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//         poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//     }

//     public PhotonPipelineResult getLatestResult1() {
//         var targets1 = camera1.getLatestResult().getTargets();
//         for (var tgt1 : targets1) {
//             var tagID = tgt1.getFiducialId();
//             if (tagID == 13 || tagID == 14 || tagID == 29 || tagID == 30) {
//                 return new PhotonPipelineResult();
//             }
//         }
//         return camera1.getLatestResult();
//     }

//     public PhotonPipelineResult getLatestResult2() {
//         return camera2.getLatestResult();
//     }

//     public PhotonPipelineResult getLatestResult3() {
//         return camera3.getLatestResult();
//     }

//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose1() {
//         // camera 1
//         var latestPoseEstimate1 = poseEstimator1.update(getLatestResult1());
//         double latestTimestamp = getLatestResult1().getTimestampSeconds();
//         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp1) > 1e-5;
//         if (newResult) {
//             lastEstTimestamp1 = latestTimestamp;
//         }
//         latestVision = latestPoseEstimate1;
//         return latestVision;
//     }

//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose2() {
//         // camera 2
//         var latestPoseEstimate2 = poseEstimator2.update(getLatestResult2());
//         double latestTimestamp = getLatestResult2().getTimestampSeconds();
//         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp2) > 1e-5;
//         if (newResult) {
//             lastEstTimestamp2 = latestTimestamp;
//         }
//         latestVision2 = latestPoseEstimate2;
//         return latestVision2;
//     }

//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose3() {
//         // camera 3
//         var latestPoseEstimate3 = poseEstimator3.update(getLatestResult3());
//         double latestTimestamp = getLatestResult3().getTimestampSeconds();
//         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp3) > 1e-5;
//         if (newResult) {
//             lastEstTimestamp3 = latestTimestamp;
//         }
//         latestVision3 = latestPoseEstimate3;
//         return latestVision3;
//     }

//     public Pose2d getPoseFromCams() {
//         var visionEst = getEstimatedGlobalPose1();
//         var visionEst2 = getEstimatedGlobalPose2();
//         var visionEst3 = getEstimatedGlobalPose3();
//         Pose2d combinedEstimate = new Pose2d();

//         if (visionEst.isPresent() && visionEst2.isPresent() && visionEst3.isPresent()) {
//             Pose2d poseFrom1 = visionEst.get().estimatedPose.toPose2d();
//             Pose2d poseFrom2 = visionEst2.get().estimatedPose.toPose2d();
//             Pose2d poseFrom3 = visionEst3.get().estimatedPose.toPose2d();

//             double estXFrom1 = poseFrom1.getX();
//             double estXFrom2 = poseFrom2.getX();
//             double estXFrom3 = poseFrom3.getX();

//             double estYFrom1 = poseFrom1.getY();
//             double estYFrom2 = poseFrom2.getY();
//             double estYFrom3 = poseFrom3.getY();

//             double estCOSFrom1 = poseFrom1.getRotation().getCos();
//             double estCOSFrom2 = poseFrom2.getRotation().getCos();
//             double estCOSFrom3 = poseFrom3.getRotation().getCos();

//             double estSINFrom1 = poseFrom1.getRotation().getSin();
//             double estSINFrom2 = poseFrom2.getRotation().getSin();
//             double estSINFrom3 = poseFrom3.getRotation().getSin();

//             double avgX = (estXFrom1 + estXFrom2 + estXFrom3) / 3;
//             double avgY = (estYFrom1 + estYFrom2 + estYFrom3) / 3;

//             double avgCos = (estCOSFrom1 + estCOSFrom2 + estCOSFrom3) / 3;
//             double avgSin = (estSINFrom1 + estSINFrom2 + estSINFrom3) / 3;

//             combinedEstimate = new Pose2d(avgX, avgY, new Rotation2d(avgCos, avgSin));
//         } else if (visionEst.isPresent() && (visionEst2.isEmpty())) {

//         } else if (visionEst2.isPresent() && (visionEst.isEmpty())) {

//         } else {
//             combinedEstimate = new Pose2d();
//         }
//         return combinedEstimate;
//     }

//     public Pose2d getPoseFromCams12() {
//         var visionEst = getEstimatedGlobalPose1();
//         var visionEst2 = getEstimatedGlobalPose2();
//         Pose2d combinedEstimate = new Pose2d();

//         if (visionEst.isPresent() && visionEst2.isPresent()) {
//             Pose2d poseFrom1 = visionEst.get().estimatedPose.toPose2d();
//             Pose2d poseFrom2 = visionEst2.get().estimatedPose.toPose2d();

//             double estXFrom1 = poseFrom1.getX();
//             double estXFrom2 = poseFrom2.getX();

//             double estYFrom1 = poseFrom1.getY();
//             double estYFrom2 = poseFrom2.getY();

//             double estCOSFrom1 = poseFrom1.getRotation().getCos();
//             double estCOSFrom2 = poseFrom2.getRotation().getCos();

//             double estSINFrom1 = poseFrom1.getRotation().getSin();
//             double estSINFrom2 = poseFrom2.getRotation().getSin();

//             double avgX = (estXFrom1 + estXFrom2) / 2;
//             double avgY = (estYFrom1 + estYFrom2) / 2;

//             double avgCos = (estCOSFrom1 + estCOSFrom2) / 2;
//             double avgSin = (estSINFrom1 + estSINFrom2) / 2;

//             combinedEstimate = new Pose2d(avgX, avgY, new Rotation2d(avgCos, avgSin));

//         } else {
//             combinedEstimate = new Pose2d();
//         }
//         return combinedEstimate;
//     }

//     public Pose2d getPoseFromCams23() {
//         var visionEst2 = getEstimatedGlobalPose2();
//         var visionEst3 = getEstimatedGlobalPose3();
//         Pose2d combinedEstimate = new Pose2d();

//         if (visionEst2.isPresent() && visionEst3.isPresent()) {
//             Pose2d poseFrom2 = visionEst2.get().estimatedPose.toPose2d();
//             Pose2d poseFrom3 = visionEst3.get().estimatedPose.toPose2d();

//             double estXFrom2 = poseFrom2.getX();
//             double estXFrom3 = poseFrom3.getX();

//             double estYFrom2 = poseFrom2.getY();
//             double estYFrom3 = poseFrom3.getY();

//             double estCOSFrom2 = poseFrom2.getRotation().getCos();
//             double estCOSFrom3 = poseFrom3.getRotation().getCos();

//             double estSINFrom2 = poseFrom2.getRotation().getSin();
//             double estSINFrom3 = poseFrom3.getRotation().getSin();

//             double avgX = (estXFrom2 + estXFrom3) / 2;
//             double avgY = (estYFrom2 + estYFrom3) / 2;

//             double avgCos = (estCOSFrom2 + estCOSFrom3) / 2;
//             double avgSin = (estSINFrom2 + estSINFrom3) / 2;

//             combinedEstimate = new Pose2d(avgX, avgY, new Rotation2d(avgCos, avgSin));
//         } else {
//             combinedEstimate = new Pose2d();
//         }
//         return combinedEstimate;
//     }

//     public Pose2d getPoseFromCams13() {
//         var visionEst = getEstimatedGlobalPose1();
//         var visionEst3 = getEstimatedGlobalPose3();
//         Pose2d combinedEstimate = new Pose2d();

//         if (visionEst.isPresent() && visionEst3.isPresent()) {
//             Pose2d poseFrom1 = visionEst.get().estimatedPose.toPose2d();
//             Pose2d poseFrom3 = visionEst3.get().estimatedPose.toPose2d();

//             double estXFrom1 = poseFrom1.getX();
//             double estXFrom3 = poseFrom3.getX();

//             double estYFrom1 = poseFrom1.getY();
//             double estYFrom3 = poseFrom3.getY();

//             double estCOSFrom1 = poseFrom1.getRotation().getCos();
//             double estCOSFrom3 = poseFrom3.getRotation().getCos();

//             double estSINFrom1 = poseFrom1.getRotation().getSin();
//             double estSINFrom3 = poseFrom3.getRotation().getSin();

//             double avgX = (estXFrom1 + estXFrom3) / 2;
//             double avgY = (estYFrom1 + estYFrom3) / 2;

//             double avgCos = (estCOSFrom1 + estCOSFrom3) / 2;
//             double avgSin = (estSINFrom1 + estSINFrom3) / 2;

//             combinedEstimate = new Pose2d(avgX, avgY, new Rotation2d(avgCos, avgSin));
//         } else {
//             combinedEstimate = new Pose2d();
//         }
//         return combinedEstimate;
//     }

//     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
//         var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
//         var targets1 = getLatestResult1().getTargets();
//         var targets2 = getLatestResult2().getTargets();
//         var targets3 = getLatestResult3().getTargets();
//         int numTags1 = 0;
//         int numTags2 = 0;
//         int numTags3 = 0;
//         double avgDist1 = 0;
//         double avgDist2 = 0;
//         double avgDist3 = 0;

//         for (var tgt1 : targets1) {
//             var tagPose = poseEstimator1.getFieldTags().getTagPose(tgt1.getFiducialId());
//             if (tagPose.isEmpty()) {
//                 continue;
//             }
//             numTags1++;
//             avgDist1 += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
//         }
//         for (var tgt2 : targets2) {
//             var tagPose = poseEstimator2.getFieldTags().getTagPose(tgt2.getFiducialId());
//             if (tagPose.isEmpty()) {
//                 continue;
//             }
//             numTags2++;
//             avgDist2 += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
//         }
//         for (var tgt3 : targets3) {
//             var tagPose = poseEstimator3.getFieldTags().getTagPose(tgt3.getFiducialId());
//             if (tagPose.isEmpty()) {
//                 continue;
//             }
//             numTags3++;
//             avgDist3 += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
//         }

//         if (numTags1 == 0) { // josh check (should check if 0 for other cams too?)
//             return estStdDevs;
//         }

//         avgDist1 /= numTags1;
//         avgDist2 /= numTags2;
//         avgDist3 /= numTags3;

//         // Decrease std devs if multiple targets are visible
//         if (numTags1 > 1 || numTags2 > 1 || numTags3 > 1) { // josh check (check if other cams are greater than 1 too?)
//             estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
//         }
//         // Increase std devs based on (average) distance
//         if ((numTags1 == 1 && avgDist1 > 4) || (numTags2 == 1 && avgDist2 > 4) || (numTags3 == 1 && avgDist3 > 4)) {
//             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
//         } else if (avgDist1 < avgDist2 || avgDist1 < avgDist3) {
//             estStdDevs = estStdDevs.times(1 + (avgDist1 * avgDist1 / 30));
//         } else if (avgDist2 < avgDist1 || avgDist2 < avgDist3) {
//             estStdDevs = estStdDevs.times(1 + (avgDist2 * avgDist2 / 30));
//         } else if (avgDist3 < avgDist1 || avgDist3 < avgDist2) {
//             estStdDevs = estStdDevs.times(1 + (avgDist3 * avgDist3 / 30));
//         }
//         return estStdDevs;
//     }
// }