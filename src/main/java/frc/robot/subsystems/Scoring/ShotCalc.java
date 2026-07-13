package frc.robot.subsystems.Scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

/**
 * Shoot-on-the-fly solver using the virtual-target method.
 *
 * Physics: the ball leaves the robot carrying the robot's velocity, so a shot
 * aimed (and ranged) at a virtual target displaced backwards by
 * (robot velocity x time of flight) lands on the real goal. The stationary
 * shot tables (RPS/HOOD/TOF maps) are looked up at the distance to that
 * virtual target — no other velocity compensation is applied, because the
 * virtual-target shift already accounts for all of it. Aiming at the virtual
 * target AND subtracting robot velocity from the shot vector (as earlier
 * versions did) double-counts the correction and undershoots moving shots.
 */
public final class ShotCalc {

    public static record ShooterCommand(double RPS, Rotation2d turretAngle, double hoodAngle) {
    }

    /** Below this distance to the goal the solver is unreliable. */
    private static final double MIN_SOLVE_DISTANCE = 0.5;

    /** Iterations to converge distance <-> time-of-flight. */
    private static final int SOLVER_ITERATIONS = 5;

    public static ShooterCommand calculateSOTF(
            Translation2d robotPosition,
            Translation2d turretPosition,
            ChassisSpeeds fieldSpeeds,
            Translation2d goalPosition) {

        // Velocity inherited by the ball: chassis translation plus the
        // tangential velocity of the turret due to chassis rotation.
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        Translation2d turretOffset = turretPosition.minus(robotPosition);
        Translation2d rotationalVelocity = new Translation2d(
                -omega * turretOffset.getY(),
                omega * turretOffset.getX());
        Translation2d totalVelocity = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond).plus(rotationalVelocity);

        Translation2d toGoal = goalPosition.minus(turretPosition);
        double distance = toGoal.getNorm();

        // Too close for the solver — fall back to a stationary shot straight
        // at the goal. Never returns 0 RPS, so the feeder can't dump a ball
        // into a dead flywheel.
        if (distance < MIN_SOLVE_DISTANCE) {
            return new ShooterCommand(
                    ShooterConstants.RPS_MAP.get(distance),
                    toGoal.getAngle(),
                    ShooterConstants.HOOD_MAP.get(distance));
        }

        // Converge the circular dependency: virtual target position depends
        // on time of flight, which depends on distance to the virtual target.
        // Distance is always measured from the REAL turret position — the
        // turret is not predicted forward, because the virtual-target shift
        // is the same physical correction expressed in the field frame.
        double timeOfFlight = ShooterConstants.TOF_MAP.get(distance);
        Translation2d virtualTarget = goalPosition;
        for (int i = 0; i < SOLVER_ITERATIONS; i++) {
            virtualTarget = goalPosition.minus(totalVelocity.times(timeOfFlight));
            distance = virtualTarget.minus(turretPosition).getNorm();
            timeOfFlight = ShooterConstants.TOF_MAP.get(distance);
        }

        Rotation2d turretAngle = virtualTarget.minus(turretPosition).getAngle();
        double rps = MathUtil.clamp(
                ShooterConstants.RPS_MAP.get(distance),
                ShooterConstants.MIN_RPS,
                ShooterConstants.MAX_RPS);
        double hoodAngle = ShooterConstants.HOOD_MAP.get(distance);

        SmartDashboard.putNumber("SOTF/Inherited Vel X", totalVelocity.getX());
        SmartDashboard.putNumber("SOTF/Inherited Vel Y", totalVelocity.getY());
        SmartDashboard.putNumber("SOTF/Virtual Target Distance", distance);
        SmartDashboard.putNumber("SOTF/Time Of Flight", timeOfFlight);

        return new ShooterCommand(rps, turretAngle, hoodAngle);
    }
}
