package frc.lib.util;

/**
 * Utility class for evaluating whether a measured value is within an acceptable tolerance of a
 * desired setpoint.
 *
 * <p>This is commonly used for determining convergence in control loops (e.g., PID controllers)
 * where exact equality is neither expected nor required.
 */
public class SetpointTracker {

  /**
   * Computes the absolute error between the current value and the setpoint.
   *
   * @param setpoint the desired target value
   * @param currentValue the measured or current value
   * @return the absolute error between {@code currentValue} and {@code setpoint}
   */
  public static double getError(double setpoint, double currentValue) {
    return Math.abs(currentValue - setpoint);
  }

  /**
   * Determines whether the current value is within the specified tolerance of the setpoint.
   *
   * @param setpoint the desired target value
   * @param tolerance the allowable error margin
   * @param currentValue the measured or current value
   * @return {@code true} if the absolute error is less than or equal to the tolerance; {@code
   *     false} otherwise
   */
  public static boolean atSetpoint(double setpoint, double tolerance, double currentValue) {
    return getError(setpoint, currentValue) <= tolerance;
  }
}
