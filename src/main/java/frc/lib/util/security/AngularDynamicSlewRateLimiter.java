package frc.lib.util.security;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * A Slew Rate Limiter with support for dynamic angular acceleration limits and strong unit
 * semantics.
 *
 * <p>This limiter constrains the rate of change of an angular value (typically angular velocity)
 * based on a maximum angular acceleration that can change at runtime.
 *
 * <p>Internally, all calculations are done in doubles (SI units) for performance, while the public
 * API uses WPILib Units for safety and clarity.
 */
public class AngularDynamicSlewRateLimiter {

  // =========================================================================
  // Internal State (SI units, doubles for speed)
  // =========================================================================

  private double maxAccelRadps2; // + angular acceleration limit
  private double maxDecelRadps2; // - angular acceleration limit (positive magnitude)

  private double prevValueRadps;
  private double prevTimeSec;

  // =========================================================================
  // Constructors
  // =========================================================================

  /**
   * Creates a new AngularDynamicSlewRateLimiter with symmetric acceleration limits.
   *
   * @param maxAngularAcceleration Maximum allowed angular acceleration magnitude.
   */
  public AngularDynamicSlewRateLimiter(AngularAcceleration maxAngularAcceleration) {
    this(maxAngularAcceleration, maxAngularAcceleration, RadiansPerSecond.of(0.0));
  }

  /**
   * Creates a new AngularDynamicSlewRateLimiter with asymmetric acceleration limits.
   *
   * @param maxAccel Maximum angular acceleration (positive direction).
   * @param maxDecel Maximum angular deceleration (negative direction, magnitude).
   * @param initialValue Initial angular velocity.
   */
  public AngularDynamicSlewRateLimiter(
      AngularAcceleration maxAccel, AngularAcceleration maxDecel, AngularVelocity initialValue) {
    this.maxAccelRadps2 = maxAccel.in(RadiansPerSecondPerSecond);
    this.maxDecelRadps2 = Math.abs(maxDecel.in(RadiansPerSecondPerSecond));

    this.prevValueRadps = initialValue.in(RadiansPerSecond);
    this.prevTimeSec = MathSharedStore.getTimestamp();
  }

  // =========================================================================
  // Dynamic Configuration
  // =========================================================================

  /**
   * Updates the angular acceleration limits dynamically. This can be called every loop.
   *
   * @param maxAccel New maximum angular acceleration.
   * @param maxDecel New maximum angular deceleration (magnitude).
   */
  public void setAccelerationLimits(AngularAcceleration maxAccel, AngularAcceleration maxDecel) {
    this.maxAccelRadps2 = maxAccel.in(RadiansPerSecondPerSecond);
    this.maxDecelRadps2 = Math.abs(maxDecel.in(RadiansPerSecondPerSecond));
  }

  /** Convenience method for symmetric angular acceleration limits. */
  public void setAccelerationLimit(AngularAcceleration maxAngularAcceleration) {
    double acc = maxAngularAcceleration.in(RadiansPerSecondPerSecond);
    this.maxAccelRadps2 = acc;
    this.maxDecelRadps2 = acc;
  }

  // =========================================================================
  // Core Logic
  // =========================================================================

  /**
   * Filters the input angular velocity to respect the current angular acceleration limits.
   *
   * @param input Desired target angular velocity.
   * @return Rate-limited angular velocity.
   */
  public AngularVelocity calculate(AngularVelocity input) {
    double now = MathSharedStore.getTimestamp();
    double dt = now - prevTimeSec;

    if (dt <= 1e-6) {
      return RadiansPerSecond.of(prevValueRadps);
    }

    double target = input.in(RadiansPerSecond);
    double delta = target - prevValueRadps;

    double maxDeltaUp = maxAccelRadps2 * dt;
    double maxDeltaDown = -maxDecelRadps2 * dt;

    double limitedDelta = MathUtil.clamp(delta, maxDeltaDown, maxDeltaUp);

    prevValueRadps += limitedDelta;
    prevTimeSec = now;

    return RadiansPerSecond.of(prevValueRadps);
  }

  // =========================================================================
  // Utilities
  // =========================================================================

  /** Returns the last output angular velocity. */
  public AngularVelocity getLastValue() {
    return RadiansPerSecond.of(prevValueRadps);
  }

  /** Resets the limiter to a specific angular velocity, ignoring acceleration limits. */
  public void reset(AngularVelocity value) {
    this.prevValueRadps = value.in(RadiansPerSecond);
    this.prevTimeSec = MathSharedStore.getTimestamp();
  }
}
