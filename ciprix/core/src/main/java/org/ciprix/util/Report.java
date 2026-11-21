package org.ciprix.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Single place where the library reports warnings and errors.
 *
 * <p>By default the messages go to the Driver Station, exactly like {@code
 * DriverStation.reportWarning} and {@code DriverStation.reportError}. Keeping every report behind
 * this class has two benefits: unit tests can capture the messages without the HAL ({@link
 * #setSink(Sink)}), and a future WPILib that changes the Driver Station API only needs this file to
 * change.
 */
public final class Report {

  /** Receives every message reported by the library. */
  @FunctionalInterface
  public interface Sink {
    /**
     * @param isError true for errors, false for warnings
     * @param message the text to show
     * @param printTrace true when the stack trace helps finding the caller
     */
    void accept(boolean isError, String message, boolean printTrace);
  }

  private static final Sink DEFAULT_SINK =
      (isError, message, printTrace) -> {
        if (isError) {
          DriverStation.reportError(message, printTrace);
        } else {
          DriverStation.reportWarning(message, printTrace);
        }
      };

  private static volatile Sink sink = DEFAULT_SINK;

  private Report() {}

  /** Reports a warning (yellow in the Driver Station). */
  public static void warning(String message) {
    sink.accept(false, message, false);
  }

  /** Reports an error (red in the Driver Station). */
  public static void error(String message) {
    sink.accept(true, message, false);
  }

  /** Reports an error, optionally with the stack trace of the caller. */
  public static void error(String message, boolean printTrace) {
    sink.accept(true, message, printTrace);
  }

  /** Replaces the destination of the messages (intended for tests). */
  public static void setSink(Sink newSink) {
    sink = newSink == null ? DEFAULT_SINK : newSink;
  }

  /** Restores the Driver Station as the destination of the messages. */
  public static void resetSink() {
    sink = DEFAULT_SINK;
  }
}
