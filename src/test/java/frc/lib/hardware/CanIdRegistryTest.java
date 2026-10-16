package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class CanIdRegistryTest {
    @Test
    void duplicateIdOnSameBusThrows() {
        CanIdRegistry.claim("test-bus", 90, "first");
        try {
            assertThrows(IllegalStateException.class, () -> CanIdRegistry.claim("test-bus", 90, "second"));
        } finally {
            CanIdRegistry.release("test-bus", 90);
        }
    }

    @Test
    void sameIdOnDifferentBusesIsFine() {
        try {
            CanIdRegistry.claim("bus-a", 91, "a");
            assertDoesNotThrow(() -> CanIdRegistry.claim("bus-b", 91, "b"));
        } finally {
            CanIdRegistry.release("bus-a", 91);
            CanIdRegistry.release("bus-b", 91);
        }
    }
}
