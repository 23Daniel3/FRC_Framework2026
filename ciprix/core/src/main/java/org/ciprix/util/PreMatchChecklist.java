package org.ciprix.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.BooleanSupplier;
import org.ciprix.interfaces.HardwareHealthMonitor;
import org.ciprix.power.PowerManager;
import org.littletonrobotics.junction.Logger;

/**
 * Checklist for the queue line: while the robot is disabled, it says on the dashboard what is still
 * wrong before the match. Each item is a sentence and a check:
 *
 * <pre>{@code
 * PreMatchChecklist.addDefaults(); // battery, devices, faults
 * PreMatchChecklist.add("Alliance set", () -> DriverStation.getAlliance().isPresent());
 * PreMatchChecklist.add("Vision sees the robot", vision::hasRecentPose);
 * }</pre>
 *
 * <p>Published under {@code PreMatch/Ready} and {@code PreMatch/Pending} (a boolean box and a list
 * in Elastic), and a warning alert lists what is pending. Once the robot is enabled the checklist
 * stops and its alert goes away: it is for the queue, not for the match.
 */
public final class PreMatchChecklist {

  /** Voltage of a charged battery at rest. */
  public static final double MIN_BATTERY_VOLTS = 12.4;

  private record Item(String description, BooleanSupplier check) {}

  private static final List<Item> items = new ArrayList<>();
  private static final String[] NOTHING = new String[0];
  private static boolean started = false;
  private static boolean ready = false;
  private static String[] pending = NOTHING;
  private static PersistentAlert alert = null;

  private PreMatchChecklist() {}

  /**
   * Adds an item.
   *
   * @param description what must be true, as the drive team should read it
   * @param check true when the item is fine
   */
  public static synchronized void add(String description, BooleanSupplier check) {
    if (description == null || check == null) {
      Report.error("PreMatchChecklist: item ignored, description and check are required.");
      return;
    }
    items.add(new Item(description, check));
    if (!started) {
      started = true;
      RobotLoop.addPeriodic("PreMatchChecklist", PreMatchChecklist::update);
    }
  }

  /** Adds the items every robot needs: charged battery, every device connected, no faults. */
  public static void addDefaults() {
    add(
        String.format(Locale.ROOT, "Battery at %.1f V or more", MIN_BATTERY_VOLTS),
        () -> PowerManager.getVoltage() >= MIN_BATTERY_VOLTS);
    add("Every device connected", HardwareHealthMonitor::isAllConnected);
    add("No hardware faults", () -> HardwareHealthMonitor.getProblems().length == 0);
  }

  /** True when every item was fine in the last check. */
  public static synchronized boolean isReady() {
    return ready;
  }

  /** Items that were not fine in the last check, in the order they were added. */
  public static synchronized String[] getPending() {
    return pending.clone();
  }

  /** Runs once per cycle through {@link RobotLoop}, only while the robot is disabled. */
  static synchronized void update() {
    if (alert == null) {
      alert = new PersistentAlert("Pre-match", PersistentAlert.Level.WARNING);
    }
    if (RobotStatus.isEnabled()) {
      alert.set(false);
      return;
    }
    List<String> notReady = new ArrayList<>();
    for (Item item : items) {
      boolean fine;
      try {
        fine = item.check().getAsBoolean();
      } catch (RuntimeException e) {
        fine = false;
      }
      if (!fine) {
        notReady.add(item.description());
      }
    }
    pending = notReady.toArray(NOTHING);
    ready = notReady.isEmpty();
    Logger.recordOutput("PreMatch/Ready", ready);
    Logger.recordOutput("PreMatch/Pending", pending);
    if (!ready) {
      alert.setText("Before the match: " + String.join(", ", notReady));
    }
    alert.set(!ready);
  }

  /** Forgets every item (intended for tests). */
  public static synchronized void resetForTesting() {
    items.clear();
    started = false;
    ready = false;
    pending = NOTHING;
    alert = null;
  }
}
