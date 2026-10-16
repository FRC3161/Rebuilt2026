package frc.lib.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

/**
 * Something that moves in a straight line: elevators and climbers
 * (vertical), linear intake slides (horizontal). Units: meters -- the motor
 * is configured so its position already reads in meters. Needs
 * withLinear(...) and withLimits(...); add withVertical(false) for
 * horizontal travel so the simulator and kG ignore gravity.
 */
public class LinearMechanism extends PositionMechanism {
    private final ElevatorSim sim;
    private MechanismLigament2d ligament;

    public LinearMechanism(MechanismConfig config) {
        super(config);
        config.require(config.drumRadiusMeters > 0 && config.massKg > 0,
                "needs withLinear(drumRadiusMeters, carriageMassKg)");
        config.require(config.hasLimits(), "needs withLimits(minMeters, maxMeters)");
        sim = new ElevatorSim(
                gearbox(),
                config.gearRatio,
                config.massKg,
                config.drumRadiusMeters,
                config.minPosition,
                config.maxPosition,
                config.vertical,
                config.startingPosition);
        initialize();
    }

    @Override
    protected double sensorToMechanismRatio() {
        // Motor rotations per meter of travel.
        return config.gearRatio / (2 * Math.PI * config.drumRadiusMeters);
    }

    @Override
    protected void customizeConfig(TalonFXConfiguration talonConfig) {
        talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    }

    @Override
    protected SimOutput simulate(double motorVolts, double dtSeconds) {
        sim.setInputVoltage(motorVolts);
        sim.update(dtSeconds);
        return new SimOutput(
                sim.getPositionMeters(),
                sim.getVelocityMetersPerSecond(),
                Math.abs(sim.getCurrentDrawAmps()));
    }

    @Override
    protected MechanismLigament2d createVisual() {
        ligament = new MechanismLigament2d("Carriage", 0.1, config.vertical ? 90 : 0);
        return ligament;
    }

    @Override
    protected void updateVisual() {
        ligament.setLength(Math.max(0.05, getPosition()));
    }
}
