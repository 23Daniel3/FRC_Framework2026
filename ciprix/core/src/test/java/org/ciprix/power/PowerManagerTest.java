package org.ciprix.power;

import static edu.wpi.first.units.Units.Amps;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ciprix.TestEnvironment;
import org.ciprix.interfaces.motor.MotorIOSim;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import org.ciprix.power.PowerManager.Level;
import org.ciprix.sim.SimMechanism;
import org.ciprix.util.PersistentAlert;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** A sagging battery must cost the low-priority mechanisms first, never the match. */
class PowerManagerTest {

  private final TestEnvironment env = new TestEnvironment();

  @BeforeEach
  void setUp() {
    env.install();
    PowerManager.start();
  }

  @AfterEach
  void tearDown() {
    env.uninstall();
  }

  private void runSeconds(double seconds) {
    int cycles = (int) Math.round(seconds / 0.02);
    for (int i = 0; i < cycles; i++) {
      env.cycle();
    }
  }

  @Test
  void healthyBatteryKeepsEverythingAtFullPower() {
    runSeconds(1.0);
    assertEquals(Level.NORMAL, PowerManager.getLevel());
    assertEquals(1.0, PowerManager.getCurrentScale(PowerPriority.LOW), 1e-9);
    assertEquals(1.0, PowerManager.getDriveAccelerationScale(), 1e-9);
    assertTrue(env.activeAlerts().isEmpty());
  }

  @Test
  void sagReducesWithinAFewCycles() {
    runSeconds(0.2);
    env.batteryVolts = 9.0;
    runSeconds(0.1);
    assertEquals(Level.REDUCED, PowerManager.getLevel());
    assertEquals(0.5, PowerManager.getCurrentScale(PowerPriority.LOW), 1e-9);
    assertEquals(0.75, PowerManager.getCurrentScale(PowerPriority.NORMAL), 1e-9);
    assertEquals(1.0, PowerManager.getCurrentScale(PowerPriority.HIGH), 1e-9);
    assertTrue(PowerManager.getDriveAccelerationScale() < 1.0);
    assertEquals(PersistentAlert.Level.WARNING, env.activeAlerts().values().iterator().next());

    env.batteryVolts = 7.5;
    runSeconds(0.1);
    assertEquals(Level.CRITICAL, PowerManager.getLevel());
    assertEquals(0.25, PowerManager.getCurrentScale(PowerPriority.LOW), 1e-9);
  }

  @Test
  void singleNoisySampleIsIgnored() {
    runSeconds(0.2);
    env.batteryVolts = 9.0;
    env.cycle();
    env.batteryVolts = 12.5;
    runSeconds(0.2);
    assertEquals(Level.NORMAL, PowerManager.getLevel());
  }

  @Test
  void recoveryIsOneStepAtATimeAndNeedsAStableBattery() {
    env.batteryVolts = 7.0;
    runSeconds(0.2);
    assertEquals(Level.CRITICAL, PowerManager.getLevel());

    env.batteryVolts = 12.0;
    runSeconds(0.3);
    assertEquals(Level.CRITICAL, PowerManager.getLevel(), "not yet: it must stay up for a while");
    runSeconds(0.4);
    assertEquals(Level.REDUCED, PowerManager.getLevel());
    runSeconds(0.6);
    assertEquals(Level.NORMAL, PowerManager.getLevel());
    assertTrue(env.activeAlerts().isEmpty());
  }

  @Test
  void hoveringJustAboveTheThresholdDoesNotRecover() {
    env.batteryVolts = 9.0;
    runSeconds(0.2);
    env.batteryVolts = 9.8; // above 9.5 but inside the margin
    runSeconds(2.0);
    assertEquals(Level.REDUCED, PowerManager.getLevel());
  }

  @Test
  void lowPriorityMotorGetsALowerCurrentLimit() {
    MotorIOSim roller =
        MotorIOSim.talonFX(
            "Conveyor/Motor",
            new MotorConfig().currentLimit(Amps.of(40)).powerPriority(PowerPriority.LOW),
            SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001).withLoad(() -> 1e6));
    MotorIOInputs inputs = new MotorIOInputs();
    roller.runPercentOutput(1.0);
    for (int i = 0; i < 10; i++) {
      env.cycle();
      roller.updateInputs(inputs);
    }
    assertEquals(40.0, inputs.current.in(Amps), 1e-6, "stalled at its limit");

    env.batteryVolts = 9.0;
    for (int i = 0; i < 10; i++) {
      env.cycle();
      roller.updateInputs(inputs);
    }
    assertEquals(20.0, inputs.current.in(Amps), 1e-6, "half of the limit while saving power");
  }
}
