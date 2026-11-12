package org.ciprix.logger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.HashMap;
import java.util.Map;
import org.ciprix.CiprixSettings;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Tunable number that works with the WPILib units library. The dashboard value is kept in the unit
 * given at construction and the unit symbol is appended to the key, for example {@code
 * "Subsystems/Flywheel/CurrentLimit (A)"}.
 *
 * <p>Like {@link LoggedTunableNumber}, tuning mode is read from {@link
 * CiprixSettings#isTuningMode()} on every access and nothing is published outside tuning mode.
 */
@SuppressWarnings({"rawtypes", "unchecked"})
public class LoggedTunableUnit {
  private static final String tableKey = "/Tuning";
  private static final double EPSILON = 1e-6;

  private final String key;
  private final Unit unit;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * @param dashboardKey Key on the dashboard.
   * @param defaultValue Initial measure (for example {@code Amps.of(40)}).
   */
  public LoggedTunableUnit(String dashboardKey, Measure defaultValue) {
    this.unit = defaultValue.unit();
    this.key = tableKey + "/" + dashboardKey + " (" + unit.symbol() + ")";
    initDefault(defaultValue);
  }

  /** Sets the default value. The default value can only be set once. */
  public void initDefault(Measure defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue.in(unit);
    }
  }

  /** Returns the raw number in the original unit. */
  public double getRaw() {
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

  /**
   * Returns the value as a measure of the right type (Current, Voltage, AngularVelocity, ...). Java
   * infers {@code T} from the call site.
   */
  public <T extends Measure> T get() {
    return (T) unit.of(getRaw());
  }

  /**
   * Checks whether the value has changed since the last check made with the same id.
   *
   * @param id Unique identifier of the caller (usually {@code hashCode()}).
   */
  public boolean hasChanged(int id) {
    if (!CiprixSettings.isTuningMode()) {
      return false;
    }
    double currentValue = getRaw();
    Double lastValue = lastHasChangedValues.get(id);

    if (lastValue == null || Math.abs(currentValue - lastValue) > EPSILON) {
      lastHasChangedValues.put(id, currentValue);
      return lastValue != null;
    }
    return false;
  }

  /** Runs the action if any of the given tunables changed. */
  public static void ifChanged(int id, Runnable action, LoggedTunableUnit... numbers) {
    if (!CiprixSettings.isTuningMode()) {
      return;
    }
    boolean changed = false;
    for (LoggedTunableUnit number : numbers) {
      changed |= number.hasChanged(id);
    }
    if (changed) {
      action.run();
    }
  }
}
