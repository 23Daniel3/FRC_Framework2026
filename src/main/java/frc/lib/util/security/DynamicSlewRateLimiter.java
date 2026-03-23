package frc.lib.util.security;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * A Slew Rate Limiter with support for dynamic acceleration limits and strong unit semantics.
 *
 * <p>This limiter constrains the rate of change of a value (typically velocity) based on a maximum
 * acceleration that can change at runtime.
 *
 * <p>Internally, all calculations are done in doubles for performance, while the public API uses
 * WPILib Units for safety and clarity.
 */
public class DynamicSlewRateLimiter {

  // =========================================================================
  // Internal State (SI units, doubles for speed)
  // =========================================================================

  private double maxAccelMps2; // + acceleration limit
  private double maxDecelMps2; // - acceleration limit (positive magnitude)

  private double prevValueMps;
  private double prevTimeSec;

  // =========================================================================
  // Constructors
  // =========================================================================

  /**
   * Creates a new DynamicSlewRateLimiter with symmetric acceleration limits.
   *
   * @param maxAcceleration Maximum allowed acceleration magnitude.
   */
  public DynamicSlewRateLimiter(LinearAcceleration maxAcceleration) {
    this(maxAcceleration, maxAcceleration, MetersPerSecond.of(0.0));
  }

  /**
   * Creates a new DynamicSlewRateLimiter with asymmetric acceleration limits.
   *
   * @param maxAccel Maximum acceleration (positive direction).
   * @param maxDecel Maximum deceleration (negative direction, magnitude).
   * @param initialValue Initial output value.
   */
  public DynamicSlewRateLimiter(
      LinearAcceleration maxAccel, LinearAcceleration maxDecel, LinearVelocity initialValue) {
    this.maxAccelMps2 = maxAccel.in(MetersPerSecondPerSecond);
    this.maxDecelMps2 = Math.abs(maxDecel.in(MetersPerSecondPerSecond));

    this.prevValueMps = initialValue.in(MetersPerSecond);
    this.prevTimeSec = MathSharedStore.getTimestamp();
  }

  // =========================================================================
  // Dynamic Configuration
  // =========================================================================

  /**
   * Updates the acceleration limits dynamically. This can be called every loop.
   *
   * @param maxAccel New maximum acceleration.
   * @param maxDecel New maximum deceleration (magnitude).
   */
  public void setAccelerationLimits(LinearAcceleration maxAccel, LinearAcceleration maxDecel) {
    this.maxAccelMps2 = maxAccel.in(MetersPerSecondPerSecond);
    this.maxDecelMps2 = Math.abs(maxDecel.in(MetersPerSecondPerSecond));
  }

  /** Convenience method for symmetric acceleration limits. */
  public void setAccelerationLimit(LinearAcceleration maxAcceleration) {
    double acc = maxAcceleration.in(MetersPerSecondPerSecond);
    this.maxAccelMps2 = acc;
    this.maxDecelMps2 = acc;
  }

  // =========================================================================
  // Core Logic
  // =========================================================================

  /**
   * Filters the input value to respect the current acceleration limits.
   *
   * @param input Desired target value.
   * @return Rate-limited output value.
   */
  public LinearVelocity calculate(LinearVelocity input) {
    double now = MathSharedStore.getTimestamp();
    double dt = now - prevTimeSec;

    if (dt <= 1e-6) {
      return MetersPerSecond.of(prevValueMps);
    }

    double target = input.in(MetersPerSecond);
    double delta = target - prevValueMps;

    double maxDeltaUp = maxAccelMps2 * dt;
    double maxDeltaDown = -maxDecelMps2 * dt;

    double limitedDelta = MathUtil.clamp(delta, maxDeltaDown, maxDeltaUp);

    prevValueMps += limitedDelta;
    prevTimeSec = now;

    return MetersPerSecond.of(prevValueMps);
  }

  // =========================================================================
  // Utilities
  // =========================================================================

  /** Returns the last output value. */
  public LinearVelocity getLastValue() {
    return MetersPerSecond.of(prevValueMps);
  }

  /** Resets the limiter to a specific value, ignoring acceleration limits. */
  public void reset(LinearVelocity value) {
    this.prevValueMps = value.in(MetersPerSecond);
    this.prevTimeSec = MathSharedStore.getTimestamp();
  }
}
