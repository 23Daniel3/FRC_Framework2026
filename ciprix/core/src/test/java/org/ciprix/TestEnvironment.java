package org.ciprix;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.ciprix.interfaces.HardwareHealthMonitor;
import org.ciprix.power.PowerManager;
import org.ciprix.sim.SimBattery;
import org.ciprix.util.PersistentAlert;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;
import org.ciprix.util.RobotLoop;
import org.ciprix.util.RobotStatus;

/**
 * Replaces everything the library reads from or writes to the robot (clock, enabled state, Driver
 * Station messages, dashboard alerts, the scheduler hook, the simulated roboRIO, the battery
 * voltage) so tests run as plain Java, without the HAL.
 *
 * <pre>{@code
 * private final TestEnvironment env = new TestEnvironment();
 *
 * @BeforeEach void setUp() { env.install(); }
 * @AfterEach void tearDown() { env.uninstall(); }
 * }</pre>
 */
public final class TestEnvironment {

  /** One alert as the dashboard would show it. */
  public static final class FakeAlert {
    public final String group;
    public final PersistentAlert.Level level;
    public String text;
    public boolean active;

    FakeAlert(String group, String text, PersistentAlert.Level level) {
      this.group = group;
      this.text = text;
      this.level = level;
    }
  }

  /** Simulated time, in seconds. */
  public double now = 100.0;

  /** Simulated enabled state of the robot. */
  public boolean enabled = true;

  /** Battery voltage read by the {@link PowerManager}. */
  public double batteryVolts = 12.5;

  /** Every message reported to the Driver Station. */
  public final List<String> reports = new ArrayList<>();

  /** Alerts created by the library, in creation order. */
  public final List<FakeAlert> alerts = new ArrayList<>();

  public void install() {
    now = 100.0;
    enabled = true;
    batteryVolts = 12.5;
    reports.clear();
    alerts.clear();
    RobotClock.setSource(() -> now);
    RobotStatus.setEnabledSource(() -> enabled);
    Report.setSink((isError, message, printTrace) -> reports.add(message));
    PersistentAlert.setBackendFactory(
        (group, text, level) -> {
          FakeAlert alert = new FakeAlert(group, text, level);
          alerts.add(alert);
          return new PersistentAlert.Backend() {
            @Override
            public void set(boolean active) {
              alert.active = active;
            }

            @Override
            public void setText(String newText) {
              alert.text = newText;
            }
          };
        });
    RobotLoop.setAutoStart(false);
    RobotLoop.clearForTesting();
    HardwareHealthMonitor.resetForTesting();
    SimBattery.resetForTesting();
    SimBattery.setRoboRioSink(volts -> {});
    PowerManager.resetForTesting();
    PowerManager.setVoltageSource(() -> batteryVolts);
  }

  public void uninstall() {
    RobotLoop.clearForTesting();
    RobotLoop.setAutoStart(true);
    HardwareHealthMonitor.resetForTesting();
    SimBattery.resetForTesting();
    SimBattery.resetRoboRioSink();
    PowerManager.resetForTesting();
    PersistentAlert.resetBackendFactory();
    Report.resetSink();
    RobotStatus.resetEnabledSource();
    RobotClock.resetSource();
  }

  /** Advances the clock by one robot cycle (20 ms) and runs the library services once. */
  public void cycle() {
    now += CiprixSettings.getLoopPeriodSeconds();
    RobotLoop.runOnce();
  }

  /** Active alerts, as "text" keyed by the order they were created. */
  public Map<String, PersistentAlert.Level> activeAlerts() {
    Map<String, PersistentAlert.Level> active = new LinkedHashMap<>();
    for (FakeAlert alert : alerts) {
      if (alert.active) {
        active.put(alert.text, alert.level);
      }
    }
    return active;
  }
}
