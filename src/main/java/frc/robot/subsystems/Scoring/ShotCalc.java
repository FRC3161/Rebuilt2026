package frc.robot.subsystems.Scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.SystemState;
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
        // Velocity contribution at the turret due to robot spinning
        Translation2d turretOffset = turretPosition.minus(robotPosition);
        Translation2d rotationalVelocity = new Translation2d(
                -omega * turretOffset.getY(),
                omega * turretOffset.getX());

        // 3. TOTAL VELOCITY at turret (translational + rotational)
        Translation2d totalVelocity = new Translation2d(
                filteredSpeeds.vxMetersPerSecond,
                filteredSpeeds.vyMetersPerSecond).plus(rotationalVelocity);

        SmartDashboard.putNumber("SOTF/Rotational Vel X", rotationalVelocity.getX());
        SmartDashboard.putNumber("SOTF/Rotational Vel Y", rotationalVelocity.getY());
        SmartDashboard.putNumber("SOTF/Translational Vel X", filteredSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("SOTF/Translational Vel Y", filteredSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("SOTF/Total Vel X", totalVelocity.getX());
        SmartDashboard.putNumber("SOTF/Total Vel Y", totalVelocity.getY());

        // 4. NULL SAFETY — too close to target
        Translation2d toGoal = goalPosition.minus(turretPosition);
        double distance = toGoal.getNorm();
        {
            if (distance < 0.5) // If we're within 0.5m of the goal, SOTF isn't reliable
                return new ShooterCommand(0, new Rotation2d(), 0);
        }

        // 5. INITIAL TABLE LOOKUPS at real distance
        double timeOfFlight = ShooterConstants.TOF_MAP.get(distance);
        Translation2d virtualTarget = goalPosition;

        // 6. ITERATIVE SOLVE (20 passes)
        // Resolves circular dependency: distance -> TOF -> corrected aim -> new
        // distance
        for (int i = 0; i < 20; i++) {
            // ADJUSTMENT: Calculate total vel of ball by adding robot vel to ball's vel
            // when standing
            // Where will the turret be when the ball arrives
            Translation2d predictedTurretPos = turretPosition.plus(totalVelocity.times(timeOfFlight));

            // Where should we aim so the ball meets the goal after robot moves
            virtualTarget = goalPosition.minus(totalVelocity.times(timeOfFlight));

            // Recalculate distance and TOF based on corrected geometry
            toGoal = virtualTarget.minus(predictedTurretPos);
            distance = toGoal.getNorm();
            timeOfFlight = ShooterConstants.TOF_MAP.get(distance);
        }

        // 7. FINAL TABLE LOOKUPS at corrected distance

        double baselineRPS = ShooterConstants.RPS_MAP.get(distance);
        double baselineHoodAngle = ShooterConstants.HOOD_MAP.get(distance);
        double baselineVelocity = distance / timeOfFlight;

        if (Shooter.getState() == SystemState.HUB_SHOOTING) {
            baselineRPS = ShooterConstants.RPS_MAP.get(distance);
            baselineHoodAngle = ShooterConstants.HOOD_MAP.get(distance);
        } else if (Shooter.getState() == SystemState.PASS_SHOOTING) {
            baselineRPS = ShooterConstants.PASSING_RPS_MAP.get(distance);
            baselineHoodAngle = ShooterConstants.PASSING_HOOD_MAP.get(distance);
        }

        // 8. VECTOR SUBTRACTION FOR AIM
        // Desired shot velocity toward virtual target, minus robot velocity
        Translation2d correctedVector = virtualTarget.minus(turretPosition);
        double correctedDist = correctedVector.getNorm();
        Translation2d targetVelocity = correctedVector.div(correctedDist).times(baselineVelocity);
        Translation2d shotVelocity = targetVelocity.minus(totalVelocity);

        // 9. TURRET ANGLE
        Rotation2d turretAngle = shotVelocity.getAngle();

        // 10. SCALE RPS PROPORTIONALLY (from this class's original approach)
        // Avoids unit conversion — stays in RPS space entirely
        double requiredVelocity = shotVelocity.getNorm();
        double velocityRatio = requiredVelocity / baselineVelocity;
        double adjustedRPS = MathUtil.clamp(
                baselineRPS * velocityRatio,
                ShooterConstants.MIN_RPS,
                ShooterConstants.MAX_RPS);

        // 11. HOOD ANGLE — baseline from table, can refine if needed
        double adjustedHood = baselineHoodAngle;

        return new ShooterCommand(adjustedRPS, turretAngle, adjustedHood);

    }

}