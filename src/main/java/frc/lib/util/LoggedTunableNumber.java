// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A number that can be changed live from the dashboard (under
 * "Tuning/<key>"), falling back to its default when the dashboard hasn't
 * set one.
 */
public class LoggedTunableNumber {
    private static final String tableKey = "Tuning";

    private final String key;
    private final double defaultValue;
    // Starts at the default so the first hasChanged() call doesn't report a
    // change that never happened.
    private double previousValue;

    public LoggedTunableNumber(String key, double defaultValue) {
        this.key = tableKey + "/" + key;
        this.defaultValue = defaultValue;
        this.previousValue = defaultValue;
        SmartDashboard.putNumber(this.key, defaultValue);
    }

    public double get() {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    /** True if the value has changed since the last time this was called. */
    public boolean hasChanged() {
        double currentValue = get();
        if (previousValue != currentValue) {
            previousValue = currentValue;
            return true;
        }
        return false;
    }
}
