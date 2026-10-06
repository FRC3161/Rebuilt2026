package frc.robot.subsystems.Scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Shoot-on-the-fly solver -- the Programming-Points (Niagara/ONCMP) version.
 *
 * Shifts the goal backwards by (robot velocity x time of flight) to get a
 * virtual target, looks up the shot tables at the distance to it, then
 * subtracts robot velocity from the shot vector toward it to get the turret
 * angle and an RPS scale factor.
 *
 * Note: the virtual-target shift and the velocity subtraction both lead the
 * turret, so the aim leads by roughly 2x (velocity x TOF). A one-shift
 * version (aim straight at the virtual target, no RPS scaling) was tried on
 * Claudes-Code from 2026-07-13 and over-led side-to-side on the real robot,
 * so this one is kept on purpose. If the aim math ever changes again, retest
 * it on the robot against this version -- don't go by the math alone.
 *
 * Hub-shot hood/RPS come from a second solve that uses only
 * SOTF_RANGE_VELOCITY_SCALE of the robot's velocity, so they react less to
 * driving toward/away from the hub than the turret aim does.
 */
public final class ShotCalc {

    public static record ShooterCommand(double RPS, Rotation2d turretAngle, double hoodAngle) {
    }

    /** Below this distance to the goal the solver is unreliable. */
    private static final double MIN_SOLVE_DISTANCE = 0.5;

    /** Iterations to converge distance <-> time-of-flight. */
    private static final int SOLVER_ITERATIONS = 10;

    public static ShooterCommand calculateSOTF(
            Translation2d robotPosition,
            Translation2d turretPosition,
            ChassisSpeeds fieldSpeeds,
            Translation2d goalPosition) {

        // Velocity at the turret: chassis translation plus the tangential
        // velocity of the turret due to chassis rotation.
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

        // Passing uses its own tables -- flatter, faster shots than an arcing
        // hub shot, tuned separately (values only meaningful at the actual
        // pass spots, not general "far away" shots).
        boolean isPassing = goalPosition.equals(FieldConstants.BLUE_PASS_SPOT_1)
                || goalPosition.equals(FieldConstants.BLUE_PASS_SPOT_2)
                || goalPosition.equals(FieldConstants.RED_PASS_SPOT_1)
                || goalPosition.equals(FieldConstants.RED_PASS_SPOT_2);

        // Too close for the solver -- fall back to a stationary shot straight
        // at the goal. (Programming-Points returns 0 RPS here; kept this
        // branch's fallback so the feeder can't dump a ball into a dead
        // flywheel.)
        if (distance < MIN_SOLVE_DISTANCE) {
            return new ShooterCommand(
                    isPassing ? ShooterConstants.PASSING_RPS_MAP.get(distance)
                            : ShooterConstants.activeRpsMap().get(distance),
                    toGoal.getAngle(),
                    isPassing ? ShooterConstants.PASSING_HOOD_MAP.get(distance)
                            : ShooterConstants.activeHoodMap().get(distance));
        }

        // Aim uses the robot's full velocity.
        Solve aim = solve(turretPosition, goalPosition, totalVelocity, distance, isPassing);
        Rotation2d turretAngle = aim.shotVelocity().getAngle();

        // Hood/RPS use a scaled-down velocity for hub shots (see
        // SOTF_RANGE_VELOCITY_SCALE). Passing keeps full velocity.
        double rangeScale = isPassing ? 1.0 : ShooterConstants.SOTF_RANGE_VELOCITY_SCALE;
        Solve range = rangeScale == 1.0
                ? aim
                : solve(turretPosition, goalPosition, totalVelocity.times(rangeScale), distance, isPassing);

        double baselineRPS = isPassing
                ? ShooterConstants.PASSING_RPS_MAP.get(range.distance())
                : ShooterConstants.activeRpsMap().get(range.distance());
        double hoodAngle = isPassing
                ? ShooterConstants.PASSING_HOOD_MAP.get(range.distance())
                : ShooterConstants.activeHoodMap().get(range.distance());

        // Scale RPS by how much faster/slower the shot vector is than the
        // stationary shot. Hood stays at the table value.
        double velocityRatio = range.shotVelocity().getNorm() / range.baselineVelocity();
        double rps = MathUtil.clamp(
                baselineRPS * velocityRatio,
                ShooterConstants.MIN_RPS,
                ShooterConstants.MAX_RPS);

        SmartDashboard.putNumber("SOTF/Inherited Vel X", totalVelocity.getX());
        SmartDashboard.putNumber("SOTF/Inherited Vel Y", totalVelocity.getY());
        SmartDashboard.putNumber("SOTF/Virtual Target Distance", aim.distance());
        SmartDashboard.putNumber("SOTF/Range Distance", range.distance());
        SmartDashboard.putNumber("SOTF/Time Of Flight", aim.timeOfFlight());
        SmartDashboard.putNumber("SOTF/Velocity Ratio", velocityRatio);

        return new ShooterCommand(rps, turretAngle, hoodAngle);
    }

    private static record Solve(double distance, double timeOfFlight, double baselineVelocity,
            Translation2d shotVelocity) {
    }

    /**
     * Virtual-target solve for one inherited velocity: converges distance <->
     * time of flight, then subtracts the velocity from the stationary shot
     * vector toward the virtual target.
     */
    private static Solve solve(Translation2d turretPosition, Translation2d goalPosition,
            Translation2d velocity, double realDistance, boolean isPassing) {
        // Virtual target position depends on time of flight, which depends on
        // distance to the virtual target.
        double distance = realDistance;
        double timeOfFlight = isPassing
                ? ShooterConstants.PASSING_TOF_MAP.get(distance)
                : ShooterConstants.TOF_MAP.get(distance);
        Translation2d virtualTarget = goalPosition;
        for (int i = 0; i < SOLVER_ITERATIONS; i++) {
            virtualTarget = goalPosition.minus(velocity.times(timeOfFlight));
            distance = virtualTarget.minus(turretPosition).getNorm();
            timeOfFlight = isPassing
                    ? ShooterConstants.PASSING_TOF_MAP.get(distance)
                    : ShooterConstants.TOF_MAP.get(distance);
        }

        // Vector subtraction for aim: the stationary shot toward the virtual
        // target, minus the velocity the ball inherits from the robot.
        double baselineVelocity = distance / timeOfFlight;
        Translation2d correctedVector = virtualTarget.minus(turretPosition);
        Translation2d targetVelocity = correctedVector.div(correctedVector.getNorm()).times(baselineVelocity);
        return new Solve(distance, timeOfFlight, baselineVelocity, targetVelocity.minus(velocity));
    }
}
