package frc.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

/**
 * A utility class that applies acceleration limiting (ramping) to {@link ChassisSpeeds}.
 *
 * <p>This class ensures that changes in robot velocity (both linear and angular) respect a
 * configured maximum acceleration. It prevents sudden jumps in speed commands, which helps maintain
 * smooth motion and reduces mechanical stress.
 *
 * <p>Typical use case:
 *
 * <pre>{@code
 * ChassisSpeedsLimiter limiter = new ChassisSpeedsLimiter();
 *
 * // In your robot loop:
 * ChassisSpeeds commandedSpeeds = new ChassisSpeeds(2.0, 0.0, 1.0);
 * ChassisSpeeds limitedSpeeds = limiter.calculate(commandedSpeeds);
 * driveSubsystem.drive(limitedSpeeds);
 * }</pre>
 */
public class ChassisSpeedsLimiter {

  /** Maximum allowed acceleration (m/s²), defined in {@link Constants}. */
  private final double maxAcceleration = DrivetrainConstants.MAX_ACCELERATION;

  /** The last calculated chassis speeds. */
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

  /** The timestamp of the last update, in seconds. */
  private double lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

  /** Creates a new {@code ChassisSpeedsLimiter}. */
  public ChassisSpeedsLimiter() {}

  /**
   * Applies acceleration limiting to the given target speeds.
   *
   * <p>If the requested change in velocity exceeds the maximum allowed acceleration, the values are
   * ramped so that the acceleration stays within limits. Otherwise, the target is passed through.
   *
   * @param targetSpeeds Desired chassis speeds ({@code vx}, {@code vy}, {@code omega}).
   * @return Smoothed chassis speeds that respect the maximum acceleration constraint.
   */
  public ChassisSpeeds calculate(ChassisSpeeds targetSpeeds) {
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    if (dt <= 0) {
      return lastSpeeds;
    }

    double vx = ramp(lastSpeeds.vxMetersPerSecond, targetSpeeds.vxMetersPerSecond, dt);
    double vy = ramp(lastSpeeds.vyMetersPerSecond, targetSpeeds.vyMetersPerSecond, dt);
    double omega = ramp(lastSpeeds.omegaRadiansPerSecond, targetSpeeds.omegaRadiansPerSecond, dt);

    lastSpeeds = new ChassisSpeeds(vx, vy, omega);
    return lastSpeeds;
  }

  /**
   * Computes a ramped value from a current value toward a target value, respecting the maximum
   * allowed acceleration.
   *
   * @param current The current value.
   * @param target The desired target value.
   * @param dt The elapsed time since the last update (s).
   * @return The ramped value, either the target (if within acceleration limits) or a limited step
   *     toward it.
   */
  private double ramp(double current, double target, double dt) {
    double delta = target - current;
    double maxDelta = maxAcceleration * dt;

    if (Math.abs(delta) > maxDelta) {
      return current + Math.copySign(maxDelta, delta);
    } else {
      return target;
    }
  }

  /**
   * Resets the limiter state.
   *
   * <p>This clears the last recorded speeds and sets the internal timestamp to the current FPGA
   * time. After calling this, the next {@link #calculate(ChassisSpeeds)} call will behave as if it
   * is the first update.
   */
  public void reset() {
    lastSpeeds = new ChassisSpeeds();
    lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }
}
