package frc.lib.mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Everything needed to build a mechanism: which motors, how they're geared,
 * how they're limited and tuned, and the physical numbers the simulator
 * needs. Built fluently:
 *
 * <pre>
 * MechanismConfig.named("Intake Rollers")
 *         .withLeader(32)
 *         .withFollower(34, MotorAlignmentValue.Opposed)
 *         .withGearRatio(3.0)
 *         .withCurrentLimits(40, 100);
 * </pre>
 *
 * Units: positions are mechanism rotations (meters for linear mechanisms),
 * velocities are per second. "Gear ratio" is motor rotations per one
 * rotation of the mechanism (a 9:1 reduction is 9.0).
 */
public class MechanismConfig {
    /** Motor types the library knows how to simulate. */
    public enum Motor {
        KRAKEN_X60,
        KRAKEN_X60_FOC,
        KRAKEN_X44,
        FALCON_500;

        DCMotor model(int count) {
            return switch (this) {
                case KRAKEN_X60 -> DCMotor.getKrakenX60(count);
                case KRAKEN_X60_FOC -> DCMotor.getKrakenX60Foc(count);
                case KRAKEN_X44 -> DCMotor.getKrakenX44(count);
                case FALCON_500 -> DCMotor.getFalcon500(count);
            };
        }

        TalonFXSimState.MotorType simType() {
            return this == KRAKEN_X44 ? TalonFXSimState.MotorType.KrakenX44 : TalonFXSimState.MotorType.KrakenX60;
        }
    }

    public record FollowerSpec(int id, MotorAlignmentValue alignment) {
    }

    /** Which way positive angles turn, seen from the side (horizontal pointing right). */
    public enum Direction {
        COUNTERCLOCKWISE,
        CLOCKWISE
    }

    // Identity and hardware
    public final String name;
    public int leaderId = -1;
    public final List<FollowerSpec> followers = new ArrayList<>();
    public CANBus canBus = CANBus.roboRIO();
    public Motor motor = Motor.KRAKEN_X60;

    // Mechanics
    public double gearRatio = 1.0;
    public boolean inverted = false;
    public boolean brakeMode = true;

    // Current limits (amps); eco limits are applied by setEcoMode(true)
    public double supplyCurrentLimit = 40;
    public double statorCurrentLimit = 80;
    public double ecoSupplyCurrentLimit = 30;
    public double ecoStatorCurrentLimit = 40;

    // Closed-loop gains (slot 0)
    public double kP, kI, kD, kS, kV, kA, kG;

    // Motion Magic profile, used for position control when cruise velocity > 0
    public double motionMagicCruiseVelocity = 0;
    public double motionMagicAcceleration = 0;
    public double motionMagicJerk = 0;

    // Soft limits / travel range, in mechanism units (NaN = none)
    public double minPosition = Double.NaN;
    public double maxPosition = Double.NaN;
    /** Where the mechanism sits at boot; the encoder is seeded to this. */
    public double startingPosition = 0;

    /** How close counts as "at setpoint", in position units or velocity units depending on the control mode. */
    public double tolerance = 0.02;

    // Stall detection: slower than stallVelocity while drawing more than stallCurrent (0 = disabled)
    public double stallVelocity = 0;
    public double stallCurrent = 0;

    // Physical properties used only by the simulator
    public double momentOfInertia = 0.001; // kg*m^2, at the mechanism
    public double armLengthMeters = 0;
    public double massKg = 0;
    public double drumRadiusMeters = 0;
    public boolean vertical = true;
    // Pivots only: where 0 is and which way is positive. Seen from the side
    // with horizontal pointing right, zeroAngle is the direction 0 points,
    // in rotations counterclockwise from horizontal (0.25 = straight up).
    // Defaults: 0 = horizontal, positive = counterclockwise (raising the arm).
    public double zeroAngle = 0;
    public Direction positiveDirection = Direction.COUNTERCLOCKWISE;

    /** Rollers only: simulate hard stops at min/maxPosition (e.g. an extension pushed into its end of travel). */
    public boolean simulateHardStops = false;

    /** Escape hatch for any Phoenix setting the builder doesn't cover. Runs last. */
    public Consumer<TalonFXConfiguration> extraConfig = c -> {
    };

    private MechanismConfig(String name) {
        this.name = name;
    }

    /** Starts a config. The name is used for dashboard keys and error messages. */
    public static MechanismConfig named(String name) {
        return new MechanismConfig(name);
    }

    public MechanismConfig withLeader(int canId) {
        this.leaderId = canId;
        return this;
    }

    /** Adds a follower motor. Opposed = spins the opposite direction from the leader. */
    public MechanismConfig withFollower(int canId, MotorAlignmentValue alignment) {
        followers.add(new FollowerSpec(canId, alignment));
        return this;
    }

    public MechanismConfig withCanBus(CANBus bus) {
        this.canBus = bus;
        return this;
    }

    public MechanismConfig withMotor(Motor motor) {
        this.motor = motor;
        return this;
    }

    public MechanismConfig withGearRatio(double motorRotationsPerMechanismRotation) {
        this.gearRatio = motorRotationsPerMechanismRotation;
        return this;
    }

    public MechanismConfig withInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public MechanismConfig withBrakeMode(boolean brake) {
        this.brakeMode = brake;
        return this;
    }

    public MechanismConfig withCurrentLimits(double supplyAmps, double statorAmps) {
        this.supplyCurrentLimit = supplyAmps;
        this.statorCurrentLimit = statorAmps;
        return this;
    }

    public MechanismConfig withEcoCurrentLimits(double supplyAmps, double statorAmps) {
        this.ecoSupplyCurrentLimit = supplyAmps;
        this.ecoStatorCurrentLimit = statorAmps;
        return this;
    }

    public MechanismConfig withPID(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        return this;
    }

    /** Static friction (V), velocity (V per unit/s) and acceleration (V per unit/s^2) feedforward. */
    public MechanismConfig withFeedforward(double s, double v, double a) {
        this.kS = s;
        this.kV = v;
        this.kA = a;
        return this;
    }

    /** Volts needed to hold against gravity (arms: when horizontal). */
    public MechanismConfig withGravity(double g) {
        this.kG = g;
        return this;
    }

    public MechanismConfig withMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        this.motionMagicCruiseVelocity = cruiseVelocity;
        this.motionMagicAcceleration = acceleration;
        this.motionMagicJerk = jerk;
        return this;
    }

    public MechanismConfig withLimits(double min, double max) {
        this.minPosition = min;
        this.maxPosition = max;
        return this;
    }

    public MechanismConfig withStartingPosition(double position) {
        this.startingPosition = position;
        return this;
    }

    public MechanismConfig withTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    /** Flags a stall when slower than {@code velocity} (mechanism units/s) while above {@code statorAmps}. */
    public MechanismConfig withStallDetection(double velocity, double statorAmps) {
        this.stallVelocity = velocity;
        this.stallCurrent = statorAmps;
        return this;
    }

    public MechanismConfig withMomentOfInertia(double kgMetersSquared) {
        this.momentOfInertia = kgMetersSquared;
        return this;
    }

    /** Pivot arms: length (pivot to tip) and mass, used by the simulator. */
    public MechanismConfig withArm(double lengthMeters, double massKg) {
        this.armLengthMeters = lengthMeters;
        this.massKg = massKg;
        return this;
    }

    /** Linear mechanisms: drum (or sprocket pitch) radius and carriage mass, used by the simulator. */
    public MechanismConfig withLinear(double drumRadiusMeters, double carriageMassKg) {
        this.drumRadiusMeters = drumRadiusMeters;
        this.massKg = carriageMassKg;
        return this;
    }

    /** Linear mechanisms: true (default) for up/down travel against gravity, false for horizontal slides. */
    public MechanismConfig withVertical(boolean vertical) {
        this.vertical = vertical;
        return this;
    }

    /**
     * Pivots: where the 0 position points, seen from the side with horizontal
     * to the right, in rotations counterclockwise from horizontal. E.g.
     * Units.degreesToRotations(90) if 0 means straight up. Default 0 (horizontal).
     */
    public MechanismConfig withZeroAt(double rotationsFromHorizontal) {
        this.zeroAngle = rotationsFromHorizontal;
        return this;
    }

    /**
     * Pivots: which way positive angles turn, seen from the same side view.
     * The motor must also move that way for positive output -- if it doesn't,
     * use withInverted(true). Default counterclockwise.
     */
    public MechanismConfig withPositiveDirection(Direction direction) {
        this.positiveDirection = direction;
        return this;
    }

    public MechanismConfig withSimulatedHardStops(boolean enabled) {
        this.simulateHardStops = enabled;
        return this;
    }

    public MechanismConfig withExtraConfig(Consumer<TalonFXConfiguration> extra) {
        this.extraConfig = extra;
        return this;
    }

    boolean hasLimits() {
        return !Double.isNaN(minPosition) && !Double.isNaN(maxPosition);
    }

    boolean usesMotionMagic() {
        return motionMagicCruiseVelocity > 0;
    }

    int motorCount() {
        return 1 + followers.size();
    }

    void require(boolean condition, String message) {
        if (!condition) {
            throw new IllegalArgumentException("Mechanism '" + name + "': " + message);
        }
    }
}
