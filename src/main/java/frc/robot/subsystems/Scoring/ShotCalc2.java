// package frc.robot.subsystems.Scoring;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

// //option 4
// public final class ShotCalc2 {

// public static record ShooterCommand(double RPS, Rotation2d turretAngle,
// double hoodAngle) {
// }

// private static ChassisSpeeds filteredSpeeds = new ChassisSpeeds();

// public static ShooterCommand calculateSOTF(
// CommandSwerveDrivetrain drivetrain) {

// Translation2d robotPosition = drivetrain.getPose().getTranslation();
// Translation2d turretPosition =
// drivetrain.getCurrentTurretPose().getTranslation();
// ChassisSpeeds rawFieldSpeeds = drivetrain.getState().Speeds;
// double omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
// Translation2d goalPosition = drivetrain.getScoringLocation();

// double alpha = 0.15;
// filteredSpeeds.vxMetersPerSecond = alpha * rawFieldSpeeds.vxMetersPerSecond
// + (1 - alpha) * filteredSpeeds.vxMetersPerSecond;
// filteredSpeeds.vyMetersPerSecond = alpha * rawFieldSpeeds.vyMetersPerSecond
// + (1 - alpha) * filteredSpeeds.vyMetersPerSecond;
// filteredSpeeds.omegaRadiansPerSecond = omega;

// Translation2d turretOffset = turretPosition.minus(robotPosition);
// Translation2d rotationalVelocity = new Translation2d(
// -omega * turretOffset.getY(),
// omega * turretOffset.getX());

// Translation2d totalVelocity = new Translation2d(
// filteredSpeeds.vxMetersPerSecond,
// filteredSpeeds.vyMetersPerSecond).plus(rotationalVelocity);

// Translation2d toGoal = goalPosition.minus(turretPosition);
// double distance = toGoal.getNorm();
// if (distance < 1e-6) {
// return new ShooterCommand(0, new Rotation2d(), 0);
// }

// // double timeOfFlight = ShooterConstants.TOF_MAP.get(distance);
// double timeOfFlight =
// ShooterConstants.timeOfFlightInterpolation.getPrediction(distance);

// for (int i = 0; i < 2; i++) {
// Translation2d predictedTurretPos =
// turretPosition.plus(totalVelocity.times(timeOfFlight));
// toGoal = goalPosition.minus(predictedTurretPos);
// distance = toGoal.getNorm();
// timeOfFlight = ShooterConstants.TOF_MAP.get(distance);
// }

// // double baselineRPS = ShooterConstants.RPS_MAP.get(distance);
// double baselineRPS =
// ShooterConstants.shooterSpeedInterpolation.getPrediction(distance);

// // double baselineHoodAngle = ShooterConstants.HOOD_MAP.get(distance);
// double baselineHoodAngle =
// ShooterConstants.hoodAngleInterpolation.getPrediction(distance);

// double baselineVelocity = distance / timeOfFlight;

// Translation2d correctedVector = goalPosition.minus(turretPosition);
// double correctedDist = correctedVector.getNorm();
// Translation2d targetVelocity =
// correctedVector.div(correctedDist).times(baselineVelocity);
// Translation2d shotVelocity = targetVelocity.minus(totalVelocity);

// Rotation2d turretAngle = shotVelocity.getAngle();

// double requiredVelocity = shotVelocity.getNorm();
// double velocityRatio = requiredVelocity / baselineVelocity;
// double adjustedRPS = MathUtil.clamp(
// baselineRPS * velocityRatio,
// ShooterConstants.MIN_RPS,
// ShooterConstants.MAX_RPS);

// double adjustedHood = baselineHoodAngle;

// return new ShooterCommand(adjustedRPS, turretAngle, adjustedHood);
// }
// }