package org.ciprix.logger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.ciprix.CiprixSettings;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 *
 * <p>Tuning mode is read from {@link CiprixSettings#isTuningMode()} every time the value is
 * requested, so a tunable can be created anywhere (including static initializers that run before
 * the robot configures the library). Outside tuning mode a tunable costs nothing: no NetworkTables
 * entry is created and {@link #hasChanged(int)} always returns {@code false}.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";
  private static final double EPSILON = 1e-6;

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    }
    if (!CiprixSettings.isTuningMode()) {
      return defaultValue;
    }
    if (dashboardNumber == null) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
    return dashboardNumber.get();
  }

  /** Returns the default value given at construction (ignores the dashboard). */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    if (!CiprixSettings.isTuningMode()) {
      // Outside tuning mode the value is constant: nothing to detect and nothing to allocate.
      return false;
    }
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);

    if (lastValue == null) {
      lastHasChangedValues.put(id, currentValue);
      return false;
    }

    if (Math.abs(currentValue - lastValue) > EPSILON) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (!CiprixSettings.isTuningMode()) {
      return;
    }
    boolean changed = false;
    for (LoggedTunableNumber tunableNumber : tunableNumbers) {
      // Evaluate every tunable (no short-circuit) so all of them record their latest value.
      changed |= tunableNumber.hasChanged(id);
    }
    if (changed) {
      double[] values = new double[tunableNumbers.length];
      for (int i = 0; i < tunableNumbers.length; i++) {
        values[i] = tunableNumbers[i].get();
      }
      action.accept(values);
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
