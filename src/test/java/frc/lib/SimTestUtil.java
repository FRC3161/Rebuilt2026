package frc.lib;

import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.lib.mechanisms.Mechanism;

/** Shared setup for tests that run mechanisms in simulation. */
public final class SimTestUtil {
    private SimTestUtil() {
    }

    /** Starts the simulated HAL with the robot enabled (simulated motors only output while enabled). */
    public static void setUpEnabledRobot() {
        HAL.initialize(500, 0);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    /**
     * Runs a robot loop body every 20ms for the given time. Phoenix's
     * simulated devices run on real time, so this really waits.
     */
    public static void runFor(double seconds, Runnable loopBody) {
        int loops = (int) Math.round(seconds / 0.02);
        for (int i = 0; i < loops; i++) {
            Unmanaged.feedEnable(100);
            loopBody.run();
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    /** One loop of a lone mechanism: read, simulate, publish. */
    public static void loop(Mechanism mechanism) {
        mechanism.updateInputs();
        mechanism.simulationPeriodic(0.02);
        mechanism.updateTelemetry();
    }
}
