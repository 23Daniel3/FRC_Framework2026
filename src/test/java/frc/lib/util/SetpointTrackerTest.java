package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SetpointTrackerTest {

  @Test
  void errorIsAbsolute() {
    assertEquals(5.0, SetpointTracker.getError(10.0, 15.0), 1e-9);
    assertEquals(5.0, SetpointTracker.getError(10.0, 5.0), 1e-9);
    assertEquals(0.0, SetpointTracker.getError(-3.0, -3.0), 1e-9);
  }

  @Test
  void atSetpointRespectsToleranceInclusive() {
    assertTrue(SetpointTracker.atSetpoint(100.0, 2.0, 101.9));
    assertTrue(SetpointTracker.atSetpoint(100.0, 2.0, 102.0), "tolerancia e inclusiva (<=)");
    assertFalse(SetpointTracker.atSetpoint(100.0, 2.0, 102.1));
    assertTrue(SetpointTracker.atSetpoint(-50.0, 1.0, -50.5), "funciona com valores negativos");
  }

  @Test
  void zeroToleranceRequiresExactMatch() {
    assertTrue(SetpointTracker.atSetpoint(7.0, 0.0, 7.0));
    assertFalse(SetpointTracker.atSetpoint(7.0, 0.0, 7.0001));
  }
}
