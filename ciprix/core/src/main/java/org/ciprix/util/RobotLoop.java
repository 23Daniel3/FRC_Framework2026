package org.ciprix.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs small pieces of library code once per robot cycle, without each of them having to be a
 * subsystem.
 *
 * <p>Library services that need a heartbeat (hardware health, power management, simulation helpers)
 * register a {@link Runnable} with {@link #addPeriodic(String, Runnable)}. The first registration
 * creates one hidden {@link PeriodicSystem}, which the command scheduler calls every cycle; that
 * single object is the only place where this part of the library touches the command framework.
 *
 * <p>Robot code can use it too, for logic that is not a mechanism:
 *
 * <pre>{@code
 * RobotLoop.addPeriodic("BatteryLed", () -> led.setLowBattery(power.isBatteryLow()));
 * }</pre>
 *
 * <p>Entries run in registration order, before the subsystems that were created after the first
 * registration. Unit tests without the HAL call {@link #setAutoStart(boolean) setAutoStart(false)}
 * and step the entries by hand with {@link #runOnce()}.
 */
public final class RobotLoop {

  private static final class Entry {
    final String name;
    final Runnable action;
    boolean failureReported = false;

    Entry(String name, Runnable action) {
      this.name = name;
      this.action = action;
    }
  }

  private static final List<Entry> entries = new ArrayList<>();
  private static boolean autoStart = true;
  private static Driver driver = null;

  private RobotLoop() {}

  /**
   * Registers an action that runs once per robot cycle.
   *
   * @param name name used by the periodic profiler ({@link PeriodicTimer}) and in error reports
   * @param action what to run every cycle
   */
  public static synchronized void addPeriodic(String name, Runnable action) {
    if (action == null) {
      Report.error("RobotLoop: addPeriodic(\"" + name + "\", null) ignored.");
      return;
    }
    entries.add(new Entry(name, action));
    if (autoStart) {
      start();
    }
  }

  /**
   * Makes sure the scheduler is calling {@link #runOnce()} every cycle. Safe to call many times.
   */
  public static synchronized void start() {
    if (driver == null) {
      driver = new Driver();
    }
  }

  /**
   * Runs every registered action once. Called by the scheduler; tests may call it directly.
   *
   * <p>An action that throws is reported (once) and does not stop the others: a bug in a dashboard
   * helper must not take the robot program down in the middle of a match.
   */
  public static void runOnce() {
    // Indexed loop: no iterator garbage, and an action may register another action safely.
    for (int i = 0; i < entries.size(); i++) {
      Entry entry = entries.get(i);
      PeriodicTimer.start(entry.name);
      try {
        entry.action.run();
      } catch (RuntimeException e) {
        if (!entry.failureReported) {
          entry.failureReported = true;
          Report.error("RobotLoop: \"" + entry.name + "\" failed: " + e, true);
        }
      } finally {
        PeriodicTimer.stop(entry.name);
      }
    }
  }

  /** Number of registered actions. */
  public static int size() {
    return entries.size();
  }

  /**
   * When {@code false}, registering an action does not create the scheduler hook. Intended for unit
   * tests that run without the HAL and call {@link #runOnce()} themselves.
   */
  public static synchronized void setAutoStart(boolean enabled) {
    autoStart = enabled;
  }

  /** Removes every registered action (intended for tests). */
  public static synchronized void clearForTesting() {
    entries.clear();
  }

  /** The only object of this class that the command scheduler knows about. */
  private static final class Driver extends PeriodicSystem {
    Driver() {
      super("CiprixLoop");
    }

    @Override
    public void periodic() {
      RobotLoop.runOnce();
    }
  }
}
