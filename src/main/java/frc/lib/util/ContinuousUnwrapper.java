package frc.lib.util;

/**
 * Utility class that transforms wrapped (cyclic) readings into a continuous, unwrapped value.
 *
 * <p>Many sensors (absolute encoders, gyros, potentiometers, etc.) return values in a fixed range
 * (e.g., 0 to 4096, or 0 to 360 degrees). When the measured value passes the maximum, it wraps back
 * to zero, and the opposite happens when moving in the negative direction. This class removes that
 * wrap-around behavior, producing a linear continuous value that can grow positively or negatively
 * without bounds.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * ContinuousUnwrapper unwrapper = new ContinuousUnwrapper(4096);
 *
 * // On each loop:
 * double continuousValue = unwrapper.update(rawSensorValue);
 * }</pre>
 */
public class ContinuousUnwrapper {

  /** The maximum raw value before wrap-around occurs (e.g., 4096 for a 12-bit encoder). */
  private final double maxValue;

  /** The last raw reading from the sensor. */
  private double lastReading = Double.NaN;

  /** The continuous, unwrapped accumulated position. */
  private double accumulatedPosition = 0.0;

  /**
   * Creates a new {@code ContinuousUnwrapper}.
   *
   * @param maxValue Maximum value before wrap-around. Must be positive (e.g., 4096, 360, 2π).
   * @throws IllegalArgumentException if {@code maxValue} is not greater than zero.
   */
  public ContinuousUnwrapper(double maxValue) {
    if (maxValue <= 0) {
      throw new IllegalArgumentException("maxValue must be greater than zero.");
    }
    this.maxValue = maxValue;
  }

  /**
   * Updates the unwrapped continuous value based on the current raw reading.
   *
   * <p>If this is the first reading, the accumulated position is initialized to that value.
   *
   * @param currentReading The current raw sensor reading, expected to be in the range [0,
   *     maxValue].
   * @return The updated continuous unwrapped value. This value is unbounded and can grow positively
   *     or negatively.
   * @throws IllegalArgumentException if {@code currentReading} is outside the valid range.
   */
  public double update(double currentReading) {
    if (currentReading < 0 || currentReading > maxValue) {
      throw new IllegalArgumentException(
          String.format("Reading out of range [0, %.2f]: %.2f", maxValue, currentReading));
    }

    // Initialize on the first reading
    if (Double.isNaN(lastReading)) {
      lastReading = currentReading;
      accumulatedPosition = currentReading;
      return accumulatedPosition;
    }

    // Direct difference
    double delta = currentReading - lastReading;

    // Adjust for wrap-around (use half of the range as threshold)
    if (delta > maxValue / 2.0) {
      delta -= maxValue;
    } else if (delta < -maxValue / 2.0) {
      delta += maxValue;
    }

    accumulatedPosition += delta;
    lastReading = currentReading;

    return accumulatedPosition;
  }

  /**
   * Returns the current continuous unwrapped value.
   *
   * @return The accumulated continuous value.
   */
  public double getAccumulatedPosition() {
    return accumulatedPosition;
  }

  /**
   * Resets the accumulated continuous value to zero, while keeping the last raw reading as
   * reference.
   */
  public void reset() {
    accumulatedPosition = 0.0;
  }

  /**
   * Resets the accumulated continuous value to a specific value, while keeping the last raw reading
   * as reference.
   *
   * @param newPosition The new desired continuous value.
   */
  public void reset(double newPosition) {
    accumulatedPosition = newPosition;
  }
}
