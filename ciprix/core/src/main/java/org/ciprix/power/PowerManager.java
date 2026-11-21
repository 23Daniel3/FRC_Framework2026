package org.ciprix.power;

import edu.wpi.first.wpilibj.RobotController;
import java.util.function.DoubleSupplier;
import org.ciprix.util.PersistentAlert;
import org.ciprix.util.RobotClock;
import org.ciprix.util.RobotLoop;
import org.littletonrobotics.junction.Logger;

/**
 * Keeps the robot away from a brownout. It watches the battery voltage and, when it sags, lowers
 * the current limit of the motors that matter least ({@link PowerPriority}) and tells the
 * drivetrain to accelerate more gently, which is where most of the current goes.
 *
 * <p>Nothing has to be called every cycle: the first motor with a priority other than {@link
 * PowerPriority#HIGH}, or the first call to a getter, registers it in the {@link RobotLoop}. The
 * drivetrain applies the acceleration scale to its own limits:
 *
 * <pre>{@code
 * double scale = PowerManager.getDriveAccelerationScale();
 * xLimiter.setAccelerationLimit(maxAcceleration.times(scale));
 * }</pre>
 *
 * <p>The level gets worse at once when the voltage crosses a threshold and better one step at a
 * time, only after the voltage stayed {@value #RECOVER_MARGIN_VOLTS} V above it for {@value
 * #RECOVER_SECONDS} s, so it does not chatter. Published under {@code Power/}.
 */
public final class PowerManager {

  /** How hard the robot is saving power. */
  public enum Level {
    /** Battery healthy: everything at full power. */
    NORMAL,
    /** Battery below {@value PowerManager#REDUCED_BELOW_VOLTS} V: low priorities are reduced. */
    REDUCED,
    /** Battery below {@value PowerManager#CRITICAL_BELOW_VOLTS} V: close to a brownout. */
    CRITICAL
  }

  /** Below this voltage the level becomes {@link Level#REDUCED}. */
  public static final double REDUCED_BELOW_VOLTS = 9.5;

  /** Below this voltage the level becomes {@link Level#CRITICAL}. */
  public static final double CRITICAL_BELOW_VOLTS = 8.0;

  /** How far above a threshold the voltage must be to step back. */
  public static final double RECOVER_MARGIN_VOLTS = 0.5;

  /** How long the voltage must stay above the threshold plus the margin to step back. */
  public static final double RECOVER_SECONDS = 0.5;

  /** Drivetrain acceleration kept at each level. */
  private static final double[] DRIVE_ACCELERATION_SCALES = {1.0, 0.6, 0.35};

  /** Weight of the newest sample: filters single-sample noise but follows a sag within ~40 ms. */
  private static final double FILTER_WEIGHT = 0.5;

  // A lambda, so the HAL is only touched when it runs (unit tests replace it).
  private static final DoubleSupplier ROBOT_CONTROLLER = () -> RobotController.getBatteryVoltage();

  private static DoubleSupplier voltageSource = ROBOT_CONTROLLER;
  private static boolean started = false;
  private static Level level = Level.NORMAL;
  private static double voltage = Double.NaN;
  private static double recoverStart = Double.NaN;
  private static PersistentAlert alert = null;

  private PowerManager() {}

  /** Registers the manager in the {@link RobotLoop}. Safe to call many times. */
  public static synchronized void start() {
    if (!started) {
      started = true;
      RobotLoop.addPeriodic("PowerManager", PowerManager::update);
    }
  }

  /** Current level. */
  public static Level getLevel() {
    start();
    return level;
  }

  /** Filtered battery voltage used for the decisions. */
  public static double getVoltage() {
    start();
    return Double.isNaN(voltage) ? voltageSource.getAsDouble() : voltage;
  }

  /** Fraction of the configured current limit a motor of {@code priority} keeps right now. */
  public static double getCurrentScale(PowerPriority priority) {
    start();
    return priority == null ? 1.0 : priority.scaleAt(level);
  }

  /** Fraction of its acceleration limits the drivetrain should use right now. */
  public static double getDriveAccelerationScale() {
    start();
    return DRIVE_ACCELERATION_SCALES[level.ordinal()];
  }

  /** Runs once per cycle. */
  static void update() {
    double sample = voltageSource.getAsDouble();
    voltage = Double.isNaN(voltage) ? sample : voltage + FILTER_WEIGHT * (sample - voltage);
    double now = RobotClock.now();

    Level worst =
        voltage < CRITICAL_BELOW_VOLTS
            ? Level.CRITICAL
            : voltage < REDUCED_BELOW_VOLTS ? Level.REDUCED : Level.NORMAL;

    if (worst.ordinal() >= level.ordinal()) {
      // Worse (or the same): react at once.
      level = worst;
      recoverStart = Double.NaN;
    } else {
      // Better: one step at a time, after the voltage stayed clearly above the threshold.
      double threshold = level == Level.CRITICAL ? CRITICAL_BELOW_VOLTS : REDUCED_BELOW_VOLTS;
      if (voltage >= threshold + RECOVER_MARGIN_VOLTS) {
        if (Double.isNaN(recoverStart)) {
          recoverStart = now;
        }
        if (now - recoverStart >= RECOVER_SECONDS) {
          level = Level.values()[level.ordinal() - 1];
          recoverStart = Double.NaN;
        }
      } else {
        recoverStart = Double.NaN;
      }
    }

    Logger.recordOutput("Power/Level", level.name());
    Logger.recordOutput("Power/FilteredVoltage", voltage);
    updateAlert();
  }

  private static void updateAlert() {
    if (level == Level.NORMAL) {
      if (alert != null) {
        alert.set(false);
      }
      return;
    }
    if (alert == null) {
      alert = new PersistentAlert("Power", PersistentAlert.Level.WARNING);
    }
    alert.setText(
        level == Level.CRITICAL
            ? "Battery near brownout: low-priority mechanisms cut, drivetrain softened"
            : "Battery sagging: low-priority mechanisms reduced, drivetrain softened");
    alert.set(true);
  }

  /** Reads the voltage from somewhere else (intended for unit tests without the HAL). */
  public static synchronized void setVoltageSource(DoubleSupplier source) {
    voltageSource = source == null ? ROBOT_CONTROLLER : source;
  }

  /** Forgets the state and reads the roboRIO again (intended for tests). */
  public static synchronized void resetForTesting() {
    voltageSource = ROBOT_CONTROLLER;
    started = false;
    level = Level.NORMAL;
    voltage = Double.NaN;
    recoverStart = Double.NaN;
    alert = null;
  }
}
