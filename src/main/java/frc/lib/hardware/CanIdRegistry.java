package frc.lib.hardware;

import java.util.HashMap;
import java.util.Map;

/**
 * Tracks which CAN IDs are in use so two devices can never silently share
 * one. Every device the library creates claims its ID here; a duplicate
 * throws at startup (in sim and on the robot) instead of producing two motors
 * fighting over the same ID.
 */
public final class CanIdRegistry {
    private static final Map<String, String> owners = new HashMap<>();

    private CanIdRegistry() {
    }

    /**
     * Claims a CAN ID on a bus for the named device.
     *
     * @throws IllegalStateException if another device already claimed it
     */
    public static synchronized void claim(String bus, int id, String owner) {
        String key = bus + ":" + id;
        String existing = owners.putIfAbsent(key, owner);
        if (existing != null) {
            throw new IllegalStateException(
                    "CAN ID " + id + " on bus '" + bus + "' is used by both '" + existing + "' and '" + owner + "'");
        }
    }

    /** Releases a claimed ID (used when a device is closed, e.g. between unit tests). */
    public static synchronized void release(String bus, int id) {
        owners.remove(bus + ":" + id);
    }

    /** Every claimed device, as "bus:id" -> owner. */
    public static synchronized Map<String, String> claimed() {
        return Map.copyOf(owners);
    }
}
