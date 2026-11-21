package org.ciprix.interfaces;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import org.ciprix.TestEnvironment;
import org.ciprix.util.PersistentAlert.Level;
import org.ciprix.util.RobotLoop;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The drive team must see a device that stops answering, and nothing while all is well. */
class HardwareHealthMonitorTest {

  private final TestEnvironment env = new TestEnvironment();

  private boolean intakeConnected;
  private String[] intakeFaults;
  private String intakeProblem;

  @BeforeEach
  void setUp() {
    env.install();
    intakeConnected = true;
    intakeFaults = new String[0];
    intakeProblem = "";
    HardwareHealthMonitor.register(
        "Intake/Roller", () -> intakeConnected, () -> intakeFaults, () -> intakeProblem);
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
  void registersItselfInTheRobotLoopOnce() {
    HardwareHealthMonitor.register("Shooter/Leader", () -> true, () -> new String[0]);
    assertEquals(1, RobotLoop.size());
    assertEquals(2, HardwareHealthMonitor.getDeviceCount());
  }

  @Test
  void healthyRobotShowsNothing() {
    runSeconds(10.0);
    assertTrue(HardwareHealthMonitor.isHealthy());
    assertTrue(env.activeAlerts().isEmpty());
    assertFalse(HardwareHealthMonitor.blink().getAsBoolean());
  }

  @Test
  void missingDeviceAtBootIsReportedAfterTheGracePeriod() {
    intakeConnected = false;

    runSeconds(2.0);
    assertTrue(env.activeAlerts().isEmpty(), "devices get time to boot");
    assertTrue(HardwareHealthMonitor.isHealthy());

    runSeconds(4.0);
    Map<String, Level> alerts = env.activeAlerts();
    assertEquals(Level.ERROR, alerts.get("Hardware disconnected: Intake/Roller"));
    assertTrue(
        alerts.keySet().stream().anyMatch(text -> text.startsWith("Boot self-test failed")),
        alerts.toString());
    assertFalse(HardwareHealthMonitor.isHealthy());
    assertArrayEquals(
        new String[] {"Intake/Roller"}, HardwareHealthMonitor.getDisconnectedDevices());

    intakeConnected = true;
    runSeconds(0.1);
    assertTrue(env.activeAlerts().isEmpty(), "everything answered: the alerts go away");
    assertTrue(HardwareHealthMonitor.isHealthy());
  }

  @Test
  void deviceLostDuringTheMatchIsReportedAtOnce() {
    runSeconds(1.0); // all connected: the boot test passes and the grace period ends
    intakeConnected = false;
    runSeconds(0.04);
    assertEquals(Level.ERROR, env.activeAlerts().get("Hardware disconnected: Intake/Roller"));
  }

  @Test
  void faultsAndProblemsAreWarnings() {
    runSeconds(1.0);
    intakeFaults = new String[] {"Fault: Over Temperature"};
    intakeProblem = "Stalled";
    runSeconds(0.04);

    assertEquals(
        Level.WARNING,
        env.activeAlerts()
            .get(
                "Hardware faults: Intake/Roller: Fault: Over Temperature"
                    + " | Intake/Roller: Stalled"));
    assertFalse(HardwareHealthMonitor.isHealthy());
    assertEquals(2, HardwareHealthMonitor.getProblems().length);

    intakeFaults = new String[0];
    intakeProblem = "";
    runSeconds(0.04);
    assertTrue(env.activeAlerts().isEmpty());
  }

  @Test
  void problemOfADisconnectedDeviceIsNotRepeated() {
    runSeconds(1.0);
    intakeConnected = false;
    intakeProblem = "Disconnected";
    runSeconds(0.04);
    assertEquals(1, env.activeAlerts().size(), env.activeAlerts().toString());
  }

  @Test
  void blinkTogglesOnlyWhileSomethingIsWrong() {
    runSeconds(1.0);
    intakeProblem = "Stalled";
    boolean sawOn = false;
    boolean sawOff = false;
    for (int i = 0; i < 50; i++) {
      env.cycle();
      if (HardwareHealthMonitor.blink().getAsBoolean()) {
        sawOn = true;
      } else {
        sawOff = true;
      }
    }
    assertTrue(sawOn && sawOff, "blinks while unhealthy");

    intakeProblem = "";
    runSeconds(0.1);
    for (int i = 0; i < 50; i++) {
      env.cycle();
      assertFalse(HardwareHealthMonitor.blink().getAsBoolean());
    }
  }
}
