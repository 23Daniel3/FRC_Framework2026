package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Stall, automatic un-jam, free spin, temperature and high current, as the state machines see them
 * through {@code inputs.hasProblem}.
 */
class MotorHealthTest {

  private static final double DT = 0.02;

  private double now;

  @BeforeEach
  void setUp() {
    now = 0.0;
  }

  /** One cycle of a motor that is commanded and driven at {@code volts}. */
  private void driven(MotorHealth health, double volts, double amps, double velocityRps) {
    now += DT;
    health.update(now, true, true, false, volts, amps, velocityRps, 40.0, true);
  }

  private void drivenFor(
      MotorHealth health, double seconds, double volts, double amps, double velocityRps) {
    int cycles = (int) Math.round(seconds / DT);
    for (int i = 0; i < cycles; i++) {
      driven(health, volts, amps, velocityRps);
    }
  }

  @Test
  void healthyMotorHasNoProblem() {
    MotorHealth health = new MotorHealth(new MotorConfig().withStallDetection(Amps.of(40), 0.5));
    drivenFor(health, 2.0, 6.0, 20.0, 50.0);
    assertFalse(health.hasProblem());
    assertEquals("", health.getProblem());
  }

  @Test
  void stallIsReportedAfterTheConfiguredTime() {
    MotorHealth health = new MotorHealth(new MotorConfig().withStallDetection(Amps.of(40), 0.49));

    drivenFor(health, 0.3, 6.0, 60.0, 0.0);
    assertFalse(health.isStalled(), "not yet: a short spike is not a stall");

    drivenFor(health, 0.3, 6.0, 60.0, 0.0);
    assertTrue(health.isStalled());
    assertEquals("Stalled", health.getProblem());
    assertFalse(health.isUnjamming(), "detection only, no reversal");
  }

  @Test
  void holdingAPositionIsNotAStall() {
    MotorHealth health = new MotorHealth(new MotorConfig().withStallDetection(Amps.of(40), 0.2));
    for (int i = 0; i < 50; i++) {
      now += DT;
      health.update(now, true, true, true, 3.0, 60.0, 0.0, 40.0, true);
    }
    assertFalse(health.isStalled());
  }

  @Test
  void releasingTheCommandClearsTheStall() {
    MotorHealth health = new MotorHealth(new MotorConfig().withStallDetection(Amps.of(40), 0.19));
    drivenFor(health, 0.4, 6.0, 60.0, 0.0);
    assertTrue(health.isStalled());

    now += DT;
    health.update(now, true, false, false, 0.0, 0.0, 0.0, 40.0, true);
    assertFalse(health.isStalled());
  }

  @Test
  void unjamReversesForTheConfiguredTimeAndTriesAgain() {
    MotorHealth health =
        new MotorHealth(new MotorConfig().withStallReversal(Amps.of(40), 0.19, 0.29, 0.3));

    drivenFor(health, 0.24, 6.0, 60.0, 0.0);
    assertTrue(health.isUnjamming());
    assertEquals(-0.3, health.getUnjamOutput(), 1e-9, "opposite to the applied voltage");
    assertEquals(1, health.getUnjamCount());

    // During the reversal the motor turns backwards with the reversal output.
    drivenFor(health, 0.2, -3.6, 10.0, -5.0);
    assertTrue(health.isUnjamming());

    drivenFor(health, 0.1, -3.6, 10.0, -5.0);
    assertFalse(health.isUnjamming(), "reversal time is over");

    // Still jammed: a second attempt.
    drivenFor(health, 0.24, 6.0, 60.0, 0.0);
    assertTrue(health.isUnjamming());
    assertEquals(2, health.getUnjamCount());
  }

  @Test
  void unjamStopsAtOnceWhenTheCommandIsReleased() {
    MotorHealth health =
        new MotorHealth(new MotorConfig().withStallReversal(Amps.of(40), 0.19, 1.0, 0.3));
    drivenFor(health, 0.24, -6.0, 60.0, 0.0);
    assertTrue(health.isUnjamming());
    assertEquals(0.3, health.getUnjamOutput(), 1e-9);

    now += DT;
    health.update(now, true, false, false, 0.0, 0.0, 0.0, 40.0, true);
    assertFalse(health.isUnjamming());
    assertEquals(0.0, health.getUnjamOutput(), 1e-9);
  }

  @Test
  void disablingTheRobotForgetsEverything() {
    MotorHealth health =
        new MotorHealth(new MotorConfig().withStallReversal(Amps.of(40), 0.19, 1.0, 0.3));
    drivenFor(health, 0.24, 6.0, 60.0, 0.0);
    assertTrue(health.isStalled());

    now += DT;
    health.update(now, false, true, false, 6.0, 60.0, 0.0, 40.0, true);
    assertFalse(health.isStalled());
    assertFalse(health.isUnjamming());
    assertEquals(0.0, health.getAverageCurrentAmps(), 1e-9);
  }

  @Test
  void spinningFreeIsReported() {
    MotorHealth health =
        new MotorHealth(
            new MotorConfig().withFreeSpinDetection(Amps.of(5), RotationsPerSecond.of(20), 0.29));
    drivenFor(health, 1.0, 6.0, 30.0, 40.0);
    assertFalse(health.isFreeSpinning(), "a loaded mechanism draws current");

    drivenFor(health, 1.5, 6.0, 1.0, 40.0);
    assertTrue(health.isFreeSpinning());
    assertEquals("Spinning free", health.getProblem());
  }

  @Test
  void temperatureHasHysteresis() {
    MotorHealth health = new MotorHealth(new MotorConfig().temperatureWarning(Celsius.of(80)));
    health.update(0.02, true, false, false, 0.0, 0.0, 0.0, 81.0, true);
    assertTrue(health.isOverTemperature());
    health.update(0.04, true, false, false, 0.0, 0.0, 0.0, 77.0, true);
    assertTrue(health.isOverTemperature(), "within the 5 degree hysteresis");
    health.update(0.06, true, false, false, 0.0, 0.0, 0.0, 74.0, true);
    assertFalse(health.isOverTemperature());
  }

  @Test
  void highAverageCurrentIsReported() {
    MotorHealth health = new MotorHealth(new MotorConfig().withCurrentWarning(Amps.of(30), 5));
    drivenFor(health, 0.2, 6.0, 40.0, 30.0);
    assertTrue(health.isHighCurrent());
    assertEquals(40.0, health.getAverageCurrentAmps(), 1e-9);

    drivenFor(health, 0.2, 6.0, 10.0, 30.0);
    assertFalse(health.isHighCurrent());
  }

  @Test
  void problemsAreCombinedInOneText() {
    MotorHealth health =
        new MotorHealth(
            new MotorConfig()
                .withStallDetection(Amps.of(40), 0.09)
                .temperatureWarning(Celsius.of(70)));
    for (int i = 0; i < 10; i++) {
      now += DT;
      health.update(now, true, true, false, 6.0, 60.0, 0.0, 75.0, false);
    }
    assertEquals("Disconnected, Stalled, Over temperature", health.getProblem());
  }
}
