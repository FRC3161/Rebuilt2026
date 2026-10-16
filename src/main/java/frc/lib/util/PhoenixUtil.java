package frc.lib.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

/** Helpers for talking to CTRE Phoenix 6 devices. */
public final class PhoenixUtil {
    private PhoenixUtil() {
    }

    /**
     * Retries a Phoenix call (usually a config apply) up to 5 times. Config
     * applies can fail transiently while the CAN bus is busy at boot; a
     * failure that sticks is reported to the Driver Station instead of
     * crashing the robot.
     */
    public static boolean tryUntilOk(String description, Supplier<StatusCode> command) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = command.get();
            if (status.isOK()) {
                return true;
            }
        }
        DriverStation.reportWarning("Could not " + description + ", error code: " + status, false);
        return false;
    }
}
