package org.ciprix.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;

/**
 * Single place where the library asks whether the robot is enabled.
 *
 * <p>By default it reads the Driver Station, which AdvantageKit replays from the log, so everything
 * that uses this class stays deterministic in replay. Keeping the question behind this class has
 * two benefits: unit tests can enable and disable the "robot" without the HAL ({@link
 * #setEnabledSource(BooleanSupplier)}), and a future WPILib that moves the Driver Station API only
 * needs this file to change.
 */
public final class RobotStatus {
  private static final BooleanSupplier DEFAULT_ENABLED = DriverStation::isEnabled;
  private static volatile BooleanSupplier enabledSource = DEFAULT_ENABLED;

  private RobotStatus() {}

  /** True while the robot is enabled (teleop, autonomous or test). */
  public static boolean isEnabled() {
    return enabledSource.getAsBoolean();
  }

  /** True while the robot is disabled. */
  public static boolean isDisabled() {
    return !enabledSource.getAsBoolean();
  }

  /** Replaces the source of the enabled state (intended for tests). */
  public static void setEnabledSource(BooleanSupplier newSource) {
    enabledSource = newSource == null ? DEFAULT_ENABLED : newSource;
  }

  /** Restores the Driver Station as the source of the enabled state. */
  public static void resetEnabledSource() {
    enabledSource = DEFAULT_ENABLED;
  }
}
