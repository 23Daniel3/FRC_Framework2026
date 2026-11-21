package org.ciprix;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot-wide settings consumed by the Ciprix library.
 *
 * <p>The library never reads constants from robot code. Everything that used to come from {@code
 * frc.robot.Constants} is configured here, once, at the very top of the {@code Robot} constructor
 * (before {@code RobotContainer} is created):
 *
 * <pre>{@code
 * public Robot() {
 *   CiprixSettings.setMode(Constants.currentMode);
 *   CiprixSettings.setTuningMode(Constants.tuningMode);
 *   CiprixSettings.setFieldSize(FieldConstants.fieldLength, FieldConstants.fieldWidth);
 *   ...
 * }
 * }</pre>
 *
 * <p>Every value has a competition-safe default, so a robot that never touches this class still
 * works: tuning is off, the periodic profiler is off and the mode is detected automatically.
 */
public final class CiprixSettings {

  /** Runtime mode of the robot program (same convention used by AdvantageKit projects). */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  private static volatile Mode mode = null;
  private static volatile boolean tuningMode = false;
  private static volatile boolean periodicTimerEnabled = false;
  private static volatile double controllerDeadband = 0.1;
  private static volatile Alliance defaultAlliance = Alliance.Blue;
  private static volatile double fieldLengthMeters = 16.54;
  private static volatile double fieldWidthMeters = 8.07;
  private static volatile double loopPeriodSeconds = 0.02;

  private CiprixSettings() {}

  /**
   * Returns the current runtime mode. When it was never set explicitly it is detected: {@link
   * Mode#REAL} on a roboRIO and {@link Mode#SIM} everywhere else.
   */
  public static Mode getMode() {
    Mode current = mode;
    if (current == null) {
      current = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
      mode = current;
    }
    return current;
  }

  /** Sets the runtime mode explicitly (use it to select {@link Mode#REPLAY}). */
  public static void setMode(Mode newMode) {
    mode = newMode;
  }

  /** True when running on a real robot. */
  public static boolean isReal() {
    return getMode() == Mode.REAL;
  }

  /** True when running the physics simulator. */
  public static boolean isSim() {
    return getMode() == Mode.SIM;
  }

  /** True when replaying a log file. */
  public static boolean isReplay() {
    return getMode() == Mode.REPLAY;
  }

  /**
   * True when tunables are published to the dashboard and can be edited live. Keep it {@code false}
   * at competitions: tunables then cost nothing and always return their default value.
   */
  public static boolean isTuningMode() {
    return tuningMode;
  }

  /** Enables or disables live tuning for every tunable in the library. */
  public static void setTuningMode(boolean enabled) {
    tuningMode = enabled;
  }

  /** True when {@code PeriodicTimer} measures and logs the duration of each periodic block. */
  public static boolean isPeriodicTimerEnabled() {
    return periodicTimerEnabled;
  }

  /** Enables or disables the periodic profiler. */
  public static void setPeriodicTimerEnabled(boolean enabled) {
    periodicTimerEnabled = enabled;
  }

  /** Default joystick deadband used by the library controllers (0 to 1). */
  public static double getControllerDeadband() {
    return controllerDeadband;
  }

  /** Sets the default joystick deadband used by the library controllers (0 to 1). */
  public static void setControllerDeadband(double deadband) {
    controllerDeadband = deadband;
  }

  /** Alliance assumed when neither the FMS nor the dashboard selector provide one. */
  public static Alliance getDefaultAlliance() {
    return defaultAlliance;
  }

  /** Sets the alliance assumed when neither the FMS nor the dashboard selector provide one. */
  public static void setDefaultAlliance(Alliance alliance) {
    defaultAlliance = alliance;
  }

  /** Field length (X axis, meters) used by the alliance mirroring utilities. */
  public static double getFieldLengthMeters() {
    return fieldLengthMeters;
  }

  /** Field width (Y axis, meters) used by the alliance mirroring utilities. */
  public static double getFieldWidthMeters() {
    return fieldWidthMeters;
  }

  /**
   * Sets the field dimensions used by the alliance mirroring utilities.
   *
   * @param lengthMeters field length along the X axis
   * @param widthMeters field width along the Y axis
   */
  public static void setFieldSize(double lengthMeters, double widthMeters) {
    fieldLengthMeters = lengthMeters;
    fieldWidthMeters = widthMeters;
  }

  /** Nominal period of the main robot loop, in seconds. */
  public static double getLoopPeriodSeconds() {
    return loopPeriodSeconds;
  }

  /** Sets the nominal period of the main robot loop, in seconds. */
  public static void setLoopPeriodSeconds(double seconds) {
    loopPeriodSeconds = seconds;
  }
}
