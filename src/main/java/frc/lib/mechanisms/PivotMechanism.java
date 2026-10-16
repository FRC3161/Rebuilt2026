package frc.lib.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

/**
 * Something that rotates to an angle against gravity: arms, wrists, hoods,
 * intake pivots. Needs withArm(...) and withLimits(...).
 *
 * Units: mechanism rotations in the team's own frame -- by default 0 =
 * horizontal and positive = counterclockwise (raising the arm), but
 * withZeroAt(...) and withPositiveDirection(...) can put 0 anywhere (e.g.
 * straight up) and make positive clockwise. Everything the code sees --
 * setPosition, getPosition, limits, the dashboard -- uses that frame. The
 * real angle from horizontal is only used inside, for the gravity
 * feedforward and the physics sim.
 */
public class PivotMechanism extends PositionMechanism {
    private final SingleJointedArmSim sim;
    private MechanismLigament2d ligament;

    public PivotMechanism(MechanismConfig config) {
        super(config);
        config.require(config.armLengthMeters > 0, "needs withArm(lengthMeters, massKg)");
        config.require(config.hasLimits(), "needs withLimits(minRotations, maxRotations)");
        double moi = config.massKg > 0
                ? SingleJointedArmSim.estimateMOI(config.armLengthMeters, config.massKg)
                : config.momentOfInertia;
        // The sim works in real angles (counterclockwise from horizontal).
        double a = toHorizontalFrame(config.minPosition), b = toHorizontalFrame(config.maxPosition);
        sim = new SingleJointedArmSim(
                gearbox(),
                config.gearRatio,
                moi,
                config.armLengthMeters,
                Units.rotationsToRadians(Math.min(a, b)),
                Units.rotationsToRadians(Math.max(a, b)),
                true,
                Units.rotationsToRadians(toHorizontalFrame(config.startingPosition)));
        initialize();
    }

    /** +1 if positive angles turn counterclockwise (the default), -1 if clockwise. */
    private double sign() {
        return config.positiveDirection == MechanismConfig.Direction.CLOCKWISE ? -1 : 1;
    }

    /** Converts a position in the team's frame to rotations counterclockwise from horizontal. */
    public double toHorizontalFrame(double rotations) {
        return config.zeroAngle + sign() * rotations;
    }

    /** Converts rotations counterclockwise from horizontal back to the team's frame. */
    public double fromHorizontalFrame(double rotationsFromHorizontal) {
        return sign() * (rotationsFromHorizontal - config.zeroAngle);
    }

    /** Drives to an angle in the team's frame, clamped to the limits. Same as setPosition in rotations. */
    public void setAngle(Rotation2d angle) {
        runPosition(angle.getRotations());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(getPosition());
    }

    /**
     * Gravity: kG volts at horizontal, scaled by the cosine of the real
     * angle, applied in whichever motor direction raises the arm.
     */
    @Override
    protected double positionFeedforward() {
        double fromHorizontal = Units.rotationsToRadians(toHorizontalFrame(getPosition()));
        return sign() * gravityGain() * Math.cos(fromHorizontal);
    }

    @Override
    protected boolean gravityInSlot() {
        return false;
    }

    @Override
    protected SimOutput simulate(double motorVolts, double dtSeconds) {
        // Positive motor output turns the arm the team's positive way.
        sim.setInputVoltage(sign() * motorVolts);
        sim.update(dtSeconds);
        return new SimOutput(
                fromHorizontalFrame(Units.radiansToRotations(sim.getAngleRads())),
                sign() * Units.radiansToRotations(sim.getVelocityRadPerSec()),
                Math.abs(sim.getCurrentDrawAmps()));
    }

    @Override
    protected MechanismLigament2d createVisual() {
        ligament = new MechanismLigament2d("Arm", 1.0, 0);
        return ligament;
    }

    @Override
    protected void updateVisual() {
        // Drawn at its real angle, so the picture matches the robot.
        ligament.setAngle(toHorizontalFrame(getPosition()) * 360.0);
    }
}
