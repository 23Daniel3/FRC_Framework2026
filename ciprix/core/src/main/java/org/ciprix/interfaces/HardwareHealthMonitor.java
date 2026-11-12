package org.ciprix.interfaces;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ciprix.util.PersistentAlert;
import org.ciprix.util.PersistentAlert.Level;
import org.ciprix.util.RobotClock;
import org.ciprix.util.RobotLoop;
import org.littletonrobotics.junction.Logger;

/**
 * Watches every hardware device of the robot (motors, encoders, ...) and tells the drive team when
 * one of them stops answering, reports a fault or has a problem.
 *
 * <p>Devices register themselves: every motor and encoder of the library does it in its
 * constructor, so a robot gets this for free. Robot code can register anything else that may fail
 * (a camera, a sensor) with {@link #register(String, BooleanSupplier, Supplier)}.
 *
 * <p>What the drive team sees:
 *
 * <ul>
 *   <li>a red alert listing the disconnected devices (devices get {@value #BOOT_GRACE_SECONDS}
 *       seconds to show up after boot before they count as disconnected);
 *   <li>a yellow alert listing active faults and problems (stalled motor, over temperature, ...);
 *   <li>a red alert when a device was still missing {@value #BOOT_GRACE_SECONDS} seconds after
 *       boot;
 *   <li>the outputs {@code Hardware/Healthy} and {@code Hardware/Blink}. {@code Blink} toggles
 *       while something is wrong, so a dashboard box bound to it flashes red; the same signal is
 *       available to robot code through {@link #blink()} (for the LEDs, for example).
 * </ul>
 *
 * <p>Nothing here allocates while the state of the devices does not change.
 */
public final class HardwareHealthMonitor {

  /** Time after boot during which a missing device is not yet a failure. */
  public static final double BOOT_GRACE_SECONDS = 5.0;

  /** How fast {@code Hardware/Blink} toggles while something is wrong. */
  public static final double BLINK_HZ = 2.5;

  private static final String[] NO_FAULTS = new String[0];

  private static final class Device {
    final String name;
    final BooleanSupplier connected;
    final Supplier<String[]> faults;
    final Supplier<String> problem;

    boolean lastConnected = true;
    String[] lastFaults = NO_FAULTS;
    String lastProblem = "";

    Device(
        String name,
        BooleanSupplier connected,
        Supplier<String[]> faults,
        Supplier<String> problem) {
      this.name = name;
      this.connected = connected;
      this.faults = faults;
      this.problem = problem;
    }
  }

  private static final List<Device> devices = new ArrayList<>();
  private static boolean started = false;
  private static boolean dirty = true;

  private static PersistentAlert disconnectedAlert;
  private static PersistentAlert faultAlert;
  private static PersistentAlert bootAlert;

  private static double bootTime = Double.NaN;
  private static boolean bootTestFinished = false;
  private static boolean inBootGrace = true;

  private static int disconnectedCount = 0;
  private static int faultCount = 0;
  private static int problemCount = 0;
  private static boolean blink = false;
  private static String[] disconnectedNames = NO_FAULTS;
  private static String[] problemLines = NO_FAULTS;

  private HardwareHealthMonitor() {}

  /**
   * Registers a device to be watched.
   *
   * @param name name shown to the drive team (for example {@code "Intake/Roller"})
   * @param isConnected true while the device is answering
   * @param activeFaults active faults reported by the device (empty when there are none)
   */
  public static void register(
      String name, BooleanSupplier isConnected, Supplier<String[]> activeFaults) {
    register(name, isConnected, activeFaults, null);
  }

  /**
   * Registers a device that can also describe a problem of its own (a stalled motor, a mechanism
   * spinning free, ...).
   *
   * @param name name shown to the drive team
   * @param isConnected true while the device is answering
   * @param activeFaults active faults reported by the device (empty when there are none)
   * @param problem short description of what is wrong, or an empty string when all is well
   */
  public static synchronized void register(
      String name,
      BooleanSupplier isConnected,
      Supplier<String[]> activeFaults,
      Supplier<String> problem) {
    devices.add(new Device(name, isConnected, activeFaults, problem));
    dirty = true;
    if (!started) {
      started = true;
      RobotLoop.addPeriodic("HardwareHealthMonitor", HardwareHealthMonitor::update);
    }
  }

  /**
   * True when every device is connected, without faults and without problems. Devices that are
   * still booting (first {@value #BOOT_GRACE_SECONDS} seconds) do not count as disconnected.
   */
  public static boolean isHealthy() {
    return (inBootGrace || disconnectedCount == 0) && faultCount == 0 && problemCount == 0;
  }

  /** True when at least one device is disconnected, has a fault or has a problem. */
  public static boolean hasProblem() {
    return !isHealthy();
  }

  /** True when every registered device is answering right now. */
  public static boolean isAllConnected() {
    return disconnectedCount == 0;
  }

  /** Names of the devices that are not answering (empty when all are connected). */
  public static String[] getDisconnectedDevices() {
    return disconnectedNames;
  }

  /** One line per fault or problem, as {@code "Device: what is wrong"}. */
  public static String[] getProblems() {
    return problemLines;
  }

  /** Number of registered devices. */
  public static int getDeviceCount() {
    return devices.size();
  }

  /**
   * Signal that toggles at {@value #BLINK_HZ} Hz while something is wrong and stays false while the
   * hardware is healthy. Bind it to a LED effect or to a dashboard indicator.
   */
  public static BooleanSupplier blink() {
    return () -> blink;
  }

  /** Evaluates every device. Runs once per cycle through {@link RobotLoop}. */
  public static synchronized void update() {
    double now = RobotClock.now();
    if (Double.isNaN(bootTime)) {
      bootTime = now;
    }

    boolean changed = dirty;
    dirty = false;
    int disconnected = 0;
    int faults = 0;
    int problems = 0;

    for (int i = 0; i < devices.size(); i++) {
      Device device = devices.get(i);

      boolean connected = device.connected.getAsBoolean();
      if (connected != device.lastConnected) {
        device.lastConnected = connected;
        changed = true;
      }

      String[] deviceFaults = device.faults == null ? NO_FAULTS : device.faults.get();
      if (deviceFaults == null) {
        deviceFaults = NO_FAULTS;
      }
      if (deviceFaults != device.lastFaults) {
        if (!Arrays.equals(deviceFaults, device.lastFaults)) {
          changed = true;
        }
        device.lastFaults = deviceFaults;
      }

      String problem = device.problem == null ? "" : device.problem.get();
      if (problem == null || !connected) {
        // A device that is not answering is already listed in the "disconnected" alert.
        problem = "";
      }
      if (!problem.equals(device.lastProblem)) {
        device.lastProblem = problem;
        changed = true;
      }

      if (!connected) {
        disconnected++;
      }
      if (deviceFaults.length > 0) {
        faults++;
      }
      if (!problem.isEmpty()) {
        problems++;
      }
    }

    disconnectedCount = disconnected;
    faultCount = faults;
    problemCount = problems;

    boolean grace = !bootTestFinished && now - bootTime <= BOOT_GRACE_SECONDS;
    if (grace != inBootGrace) {
      inBootGrace = grace;
      changed = true;
    }

    if (changed) {
      rebuildAlerts();
    }
    updateBootTest(now);

    boolean healthy = isHealthy();
    blink = !healthy && ((long) Math.floor(now * BLINK_HZ * 2.0)) % 2 == 0;
    Logger.recordOutput("Hardware/Healthy", healthy);
    Logger.recordOutput("Hardware/Blink", blink);
  }

  private static void rebuildAlerts() {
    if (disconnectedAlert == null) {
      disconnectedAlert = new PersistentAlert("Hardware disconnected", Level.ERROR);
      faultAlert = new PersistentAlert("Hardware faults", Level.WARNING);
    }

    List<String> missing = new ArrayList<>();
    List<String> lines = new ArrayList<>();
    for (int i = 0; i < devices.size(); i++) {
      Device device = devices.get(i);
      if (!device.lastConnected) {
        missing.add(device.name);
      }
      for (String fault : device.lastFaults) {
        lines.add(device.name + ": " + fault);
      }
      if (!device.lastProblem.isEmpty()) {
        lines.add(device.name + ": " + device.lastProblem);
      }
    }
    disconnectedNames = missing.toArray(new String[0]);
    problemLines = lines.toArray(new String[0]);

    if (missing.isEmpty() || inBootGrace) {
      disconnectedAlert.set(false);
    } else {
      disconnectedAlert.show("Hardware disconnected: " + String.join(", ", missing));
    }

    if (lines.isEmpty()) {
      faultAlert.set(false);
    } else {
      faultAlert.show("Hardware faults: " + String.join(" | ", lines));
    }

    Logger.recordOutput("Hardware/Disconnected", disconnectedNames);
    Logger.recordOutput("Hardware/Problems", problemLines);
  }

  private static void updateBootTest(double now) {
    if (!bootTestFinished) {
      if (disconnectedCount == 0) {
        bootTestFinished = true;
        if (inBootGrace) {
          inBootGrace = false;
          dirty = true;
        }
      } else if (now - bootTime > BOOT_GRACE_SECONDS) {
        bootTestFinished = true;
        if (bootAlert == null) {
          bootAlert = new PersistentAlert("Boot self-test failed", Level.ERROR);
        }
        bootAlert.show(
            "Boot self-test failed, devices missing after "
                + (int) BOOT_GRACE_SECONDS
                + " s: "
                + String.join(", ", disconnectedNames));
      }
    } else if (bootAlert != null && bootAlert.isActive() && disconnectedCount == 0) {
      // Everything answered in the end: the alert has done its job.
      bootAlert.set(false);
    }
  }

  /** Forgets every device and alert (intended for tests). */
  public static synchronized void resetForTesting() {
    devices.clear();
    started = false;
    dirty = true;
    disconnectedAlert = null;
    faultAlert = null;
    bootAlert = null;
    bootTime = Double.NaN;
    bootTestFinished = false;
    inBootGrace = true;
    disconnectedCount = 0;
    faultCount = 0;
    problemCount = 0;
    blink = false;
    disconnectedNames = NO_FAULTS;
    problemLines = NO_FAULTS;
  }
}
