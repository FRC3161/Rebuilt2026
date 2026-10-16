package frc.lib.mechanisms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

/**
 * Something that spins open-loop: intake rollers, indexers, feeders, or an
 * extension pushed into a hard stop. Units: mechanism rotations.
 *
 * With withLimits(...) + withSimulatedHardStops(true), the simulator stops
 * the mechanism dead at the ends of travel, so stall detection can be
 * tested in sim.
 */
public class RollerMechanism extends Mechanism {
    private final DCMotorSim sim;
    private MechanismLigament2d ligament;

    public RollerMechanism(MechanismConfig config) {
        super(config);
        if (config.simulateHardStops) {
            config.require(config.hasLimits(), "withSimulatedHardStops(true) needs withLimits(min, max)");
        }
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(gearbox(), config.momentOfInertia, config.gearRatio),
                gearbox());
        sim.setState(Units.rotationsToRadians(config.startingPosition), 0);
        initialize();
    }

    /** Closed-loop velocity in rotations/s (needs gains in the config). */
    public void setVelocity(double rotationsPerSecond) {
        runVelocity(rotationsPerSecond);
    }

    @Override
    protected boolean useSoftLimits() {
        return !config.simulateHardStops;
    }

    @Override
    protected boolean isTunable() {
        return config.kP != 0 || config.kV != 0;
    }

    @Override
    protected SimOutput simulate(double motorVolts, double dtSeconds) {
        sim.setInputVoltage(motorVolts);
        sim.update(dtSeconds);
        double position = Units.radiansToRotations(sim.getAngularPositionRad());
        double velocity = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());

        if (config.simulateHardStops
                && (position <= config.minPosition || position >= config.maxPosition)) {
            position = MathUtil.clamp(position, config.minPosition, config.maxPosition);
            velocity = 0;
            sim.setState(Units.rotationsToRadians(position), 0);
        }
        double current = Math.min(Math.abs(sim.getCurrentDrawAmps()),
                config.supplyCurrentLimit * config.motorCount());
        return new SimOutput(position, velocity, current);
    }

    @Override
    protected MechanismLigament2d createVisual() {
        ligament = new MechanismLigament2d("Roller", 1.0, 0);
        return ligament;
    }

    @Override
    protected void updateVisual() {
        ligament.setAngle(getPosition() * 360.0);
    }
}
