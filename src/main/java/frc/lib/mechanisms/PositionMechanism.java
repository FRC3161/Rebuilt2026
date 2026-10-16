package frc.lib.mechanisms;

/**
 * A mechanism that drives to positions: pivots, linear slides/elevators and
 * turrets. Code that only needs "go to position X" (like an intake deploy)
 * can hold a PositionMechanism and not care which kind it is.
 */
public abstract class PositionMechanism extends Mechanism {
    protected PositionMechanism(MechanismConfig config) {
        super(config);
    }

    /** Drives to a position in mechanism units, clamped to the limits. */
    public void setPosition(double position) {
        runPosition(position);
    }

    /** True if the mechanism is within tolerance of the given position, whatever it was commanded to. */
    public boolean isNear(double position) {
        return Math.abs(getPosition() - position) < config.tolerance;
    }
}
