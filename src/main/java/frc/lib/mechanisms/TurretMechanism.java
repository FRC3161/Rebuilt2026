package frc.lib.mechanisms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

/**
 * Something that rotates to an angle with no gravity load and limited
 * travel: turrets, swivels. Units: mechanism rotations, 0 = forward. Needs
 * withLimits(...); the travel can exceed one full turn.
 */
public class TurretMechanism extends PositionMechanism {
    private final DCMotorSim sim;
    private MechanismLigament2d ligament;

    public TurretMechanism(MechanismConfig config) {
        super(config);
        config.require(config.hasLimits(), "needs withLimits(minRotations, maxRotations)");
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(gearbox(), config.momentOfInertia, config.gearRatio),
                gearbox());
        sim.setState(Units.rotationsToRadians(config.startingPosition), 0);
        initialize();
    }

    /**
     * Points at a robot-relative angle, choosing whichever equivalent
     * position (angle plus or minus whole turns) is inside the limits and
     * closest to where the turret is now. If no equivalent fits, it goes as
     * far as the limits allow.
     */
    public void setAngle(Rotation2d robotRelativeAngle) {
        runPosition(bestPositionFor(robotRelativeAngle.getRotations(), getPosition()));
    }

    double bestPositionFor(double targetRotations, double currentRotations) {
        double best = Double.NaN;
        for (int turns = -3; turns <= 3; turns++) {
            double candidate = targetRotations + turns;
            if (candidate < config.minPosition || candidate > config.maxPosition) {
                continue;
            }
            if (Double.isNaN(best) || Math.abs(candidate - currentRotations) < Math.abs(best - currentRotations)) {
                best = candidate;
            }
        }
        return Double.isNaN(best)
                ? MathUtil.clamp(targetRotations, config.minPosition, config.maxPosition)
                : best;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(getPosition());
    }

    @Override
    protected SimOutput simulate(double motorVolts, double dtSeconds) {
        sim.setInputVoltage(motorVolts);
        sim.update(dtSeconds);
        double position = Units.radiansToRotations(sim.getAngularPositionRad());
        double velocity = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
        if (position <= config.minPosition || position >= config.maxPosition) {
            position = MathUtil.clamp(position, config.minPosition, config.maxPosition);
            velocity = 0;
            sim.setState(Units.rotationsToRadians(position), 0);
        }
        return new SimOutput(position, velocity, Math.abs(sim.getCurrentDrawAmps()));
    }

    @Override
    protected MechanismLigament2d createVisual() {
        ligament = new MechanismLigament2d("Turret", 1.0, 90);
        return ligament;
    }

    @Override
    protected void updateVisual() {
        ligament.setAngle(90 + getPosition() * 360.0);
    }
}
