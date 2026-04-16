package frc.robot.subsystems.Scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// option 3
public final class ShotCalc {

    public static record ShooterCommand(double RPS, Rotation2d turretAngle, double hoodAngle) {
    }

    // Low-pass filter state — persists between calls
    private static ChassisSpeeds filteredSpeeds = new ChassisSpeeds();

    public static ShooterCommand calculateSOTF(
            Translation2d robotPosition,
            Translation2d turretPosition,
            ChassisSpeeds rawFieldSpeeds,
            double omega,
            Translation2d goalPosition) {

        // 1. LOW-PASS FILTER on translational velocity
        double alpha = 1.0;
        filteredSpeeds.vxMetersPerSecond = alpha * rawFieldSpeeds.vxMetersPerSecond
                + (1 - alpha) * filteredSpeeds.vxMetersPerSecond;
        filteredSpeeds.vyMetersPerSecond = alpha * rawFieldSpeeds.vyMetersPerSecond
                + (1 - alpha) * filteredSpeeds.vyMetersPerSecond;
        filteredSpeeds.omegaRadiansPerSecond = omega; // no filter on omega

        // 2. ROTATIONAL VELOCITY CORRECTION
        Translation2d turretOffset = turretPosition.minus(robotPosition);
        Translation2d rotationalVelocity = new Translation2d(
                -omega * turretOffset.getY(),
                omega * turretOffset.getX());

        // 3. TOTAL VELOCITY at turret (translational + rotational)
        Translation2d totalVelocity = new Translation2d(
                filteredSpeeds.vxMetersPerSecond,
                filteredSpeeds.vyMetersPerSecond).plus(rotationalVelocity);

        // SmartDashboard.putNumber("Rotational Vel X", rotationalVelocity.getX());
        // SmartDashboard.putNumber("Rotational Vel Y", rotationalVelocity.getY());
        // SmartDashboard.putNumber("Translational Vel X",
        // filteredSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("Translational Vel Y",
        // filteredSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber("Total Vel X", totalVelocity.getX());
        // SmartDashboard.putNumber("Total Vel Y", totalVelocity.getY());

        // 4. NULL SAFETY — too close to target
        Translation2d toGoal = goalPosition.minus(turretPosition);
        double distance = toGoal.getNorm();
        if (distance < 0.5)
            return new ShooterCommand(0, new Rotation2d(), 0);

        boolean isPassing = goalPosition.equals(FieldConstants.BLUE_PASS_SPOT_1)
                || goalPosition.equals(FieldConstants.BLUE_PASS_SPOT_2)
                || goalPosition.equals(FieldConstants.RED_PASS_SPOT_1)
                || goalPosition.equals(FieldConstants.RED_PASS_SPOT_2);

        // SmartDashboard.putBoolean("Is Passing Shot", isPassing);

        // 5. INITIAL TABLE LOOKUPS at real distance
        double timeOfFlight = isPassing
                ? ShooterConstants.PASSING_TOF_MAP.get(distance)
                : ShooterConstants.TOF_MAP.get(distance);
        Translation2d virtualTarget = goalPosition;

        // 6. ITERATIVE SOLVE (10 passes)
        for (int i = 0; i < 10; i++) {
            virtualTarget = goalPosition.minus(totalVelocity.times(timeOfFlight));
            toGoal = virtualTarget.minus(turretPosition);
            distance = toGoal.getNorm();
            timeOfFlight = isPassing
                    ? ShooterConstants.PASSING_TOF_MAP.get(distance)
                    : ShooterConstants.TOF_MAP.get(distance);
        }

        // 7. FINAL TABLE LOOKUPS at corrected distance
        double baselineRPS = isPassing
                ? ShooterConstants.PASSING_RPS_MAP.get(distance)
                : ShooterConstants.RPS_MAP.get(distance);
        double baselineHoodAngle = isPassing
                ? ShooterConstants.PASSING_HOOD_MAP.get(distance)
                : ShooterConstants.HOOD_MAP.get(distance);
        double baselineVelocity = distance / timeOfFlight;

        // 8. VECTOR SUBTRACTION FOR AIM
        Translation2d correctedVector = virtualTarget.minus(turretPosition);
        double correctedDist = correctedVector.getNorm();
        Translation2d targetVelocity = correctedVector.div(correctedDist).times(baselineVelocity);
        Translation2d shotVelocity = targetVelocity.minus(totalVelocity);

        // 9. TURRET ANGLE
        Rotation2d turretAngle = shotVelocity.getAngle();

        // 10. SCALE RPS PROPORTIONALLY
        double requiredVelocity = shotVelocity.getNorm();
        double velocityRatio = requiredVelocity / baselineVelocity;
        double adjustedRPS = MathUtil.clamp(
                baselineRPS, /* * velocityRatio, */
                ShooterConstants.MIN_RPS,
                ShooterConstants.MAX_RPS);

        // SmartDashboard.putNumber("Baseline Velocity", baselineVelocity);
        // SmartDashboard.putNumber("Target Velocity Norm", targetVelocity.getNorm());
        // SmartDashboard.putNumber("Shot Velocity Norm", shotVelocity.getNorm());
        // SmartDashboard.putNumber("Shot Angle", turretAngle.getDegrees());
        // SmartDashboard.putNumber("Velocity Ratio", velocityRatio);
        // SmartDashboard.putNumber("Adjusted RPS", adjustedRPS);

        // 11. HOOD ANGLE
        double adjustedHood = baselineHoodAngle;

        return new ShooterCommand(adjustedRPS, turretAngle, adjustedHood);
    }

    public static double rpsOffset = 0.0;
    public static double hoodOffset = 0.0;
}