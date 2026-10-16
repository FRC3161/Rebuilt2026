package frc.lib.mechanisms;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

/**
 * A wheel held at a speed: shooter flywheels, velocity-controlled rollers.
 * Units: rotations/s of the wheel. Set the tolerance to how close (in rot/s)
 * counts as "at speed".
 */
public class FlywheelMechanism extends Mechanism {
    private final FlywheelSim sim;
    private double simPosition = 0;
    private MechanismLigament2d ligament;

    public FlywheelMechanism(MechanismConfig config) {
        super(config);
        sim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(gearbox(), config.momentOfInertia, config.gearRatio),
                gearbox());
        initialize();
    }

    public void setVelocity(double rotationsPerSecond) {
        runVelocity(rotationsPerSecond);
    }

    /** True once the wheel is within tolerance of the commanded speed. */
    public boolean isAtSpeed() {
        return isAtSetpoint();
    }

    @Override
    protected boolean useSoftLimits() {
        return false;
    }

    @Override
    protected SimOutput simulate(double motorVolts, double dtSeconds) {
        sim.setInputVoltage(motorVolts);
        sim.update(dtSeconds);
        double velocity = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
        simPosition += velocity * dtSeconds;
        return new SimOutput(simPosition, velocity, Math.abs(sim.getCurrentDrawAmps()));
    }

    @Override
    protected MechanismLigament2d createVisual() {
        ligament = new MechanismLigament2d("Flywheel", 1.0, 0);
        return ligament;
    }

    @Override
    protected void updateVisual() {
        ligament.setAngle(getPosition() * 360.0);
    }
}
