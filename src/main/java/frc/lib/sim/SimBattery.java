package frc.lib.sim;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Simulated battery sag. Each simulated mechanism reports how much current
 * it's drawing; once per loop the total is turned into a loaded battery
 * voltage, so a mechanism stalled in sim browns out the others like it
 * would on the real robot.
 */
public final class SimBattery {
    private static final Map<String, Double> currents = new HashMap<>();

    private SimBattery() {
    }

    /** Reports one mechanism's current draw for this loop, in amps. */
    public static void reportCurrent(String name, double amps) {
        currents.put(name, Math.abs(amps));
    }

    /** Applies the total current draw to the simulated battery voltage. Call once per loop. */
    public static void update() {
        double[] draws = currents.values().stream().mapToDouble(Double::doubleValue).toArray();
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(draws));
    }
}
