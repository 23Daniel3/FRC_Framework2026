package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.ciprix.interfaces.motor.advanced.MotorBase;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorConfig.GravityType;

/**
 * Closed-loop control computed in software: PID, feedforward (kS, kV, kA, kG) and a trapezoidal
 * motion profile, with one set of gains per slot.
 *
 * <p>It is what runs inside the motor controller on a real robot, written once so that:
 *
 * <ul>
 *   <li>{@link MotorIOSim} can behave like the real controller with the <b>same gains</b> used on
 *       the robot;
 *   <li>{@code MotorIOComposed} can close the loop on the roboRIO for motors without an encoder of
 *       their own.
 * </ul>
 *
 * <p>Gains do not mean the same thing for every vendor, so the loop has two styles (see {@link
 * Style}). It is pure math: no hardware, no timers, the caller provides the time step.
 */
public final class SoftwareClosedLoop {

  /** How the gains are interpreted. */
  public enum Style {
    /**
     * Phoenix 6 and WPILib convention. Position error in rotations, velocity error in rotations per
     * second, output in volts. kI integrates the error over time and kD multiplies its rate of
     * change.
     */
    VOLTS,

    /**
     * REV Spark convention. Velocity in RPM. The PID output is a duty cycle, computed 1000 times
     * per second without scaling by the time step (kI accumulates the error every millisecond and
     * kD multiplies the difference between two consecutive errors). The feedforward gains are in
     * volts.
     */
    SPARK
  }

  private static final int SLOTS = MotorBase.SLOT_COUNT;

  private final Style style;
  private final double nominalVolts;
  private final GravityType gravityType;
  private final boolean wrap;
  private final double wrapRange;

  private final double[] kP = new double[SLOTS];
  private final double[] kI = new double[SLOTS];
  private final double[] kD = new double[SLOTS];
  private final double[] kS = new double[SLOTS];
  private final double[] kV = new double[SLOTS];
  private final double[] kA = new double[SLOTS];
  private final double[] kG = new double[SLOTS];
  private final TrapezoidProfile[] profiles = new TrapezoidProfile[SLOTS];

  private double minOutput;
  private double maxOutput;

  // State of the loop that is running.
  private MotorControlMode activeMode = MotorControlMode.IDLE;
  private int activeSlot = -1;
  private double integral = 0.0;
  private double previousError = 0.0;
  private boolean hasPreviousError = false;
  private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
  private final TrapezoidProfile.State profileGoal = new TrapezoidProfile.State();
  private double profileAcceleration = 0.0;

  public SoftwareClosedLoop(MotorConfig config, Style style) {
    this.style = style;
    this.nominalVolts = Math.abs(config.nominalVoltage.in(Volts));
    this.gravityType = config.gravityType;
    this.wrap = config.positionWrap;
    this.wrapRange = Math.abs(config.maxPosition.in(Rotations) - config.minPosition.in(Rotations));
    this.minOutput = config.minOutput;
    this.maxOutput = config.maxOutput;

    for (int i = 0; i < SLOTS; i++) {
      kP[i] = config.kP[i];
      kI[i] = config.kI[i];
      kD[i] = config.kD[i];
      kS[i] = config.kS[i];
      kV[i] = config.kV[i];
      kA[i] = config.kA[i];
      kG[i] = config.kG[i];
      setMotion(
          i,
          config.maxMotionMaxVelocity[i].in(RotationsPerSecond),
          config.maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond));
    }
  }

  // ---------------------------------------------------------------------------
  // Gains
  // ---------------------------------------------------------------------------

  public void setPID(int slot, double p, double i, double d) {
    kP[slot] = p;
    kI[slot] = i;
    kD[slot] = d;
    if (slot == activeSlot) {
      integral = 0.0;
    }
  }

  public void setFeedforward(int slot, double s, double v, double a, double g) {
    kS[slot] = s;
    kV[slot] = v;
    kA[slot] = a;
    kG[slot] = g;
  }

  /**
   * Limits of the motion profile of a slot.
   *
   * @param maxVelocityRps cruise velocity, in rotations per second
   * @param maxAccelerationRpsPerSec acceleration, in rotations per second squared
   */
  public void setMotion(int slot, double maxVelocityRps, double maxAccelerationRpsPerSec) {
    if (maxVelocityRps > 0.0 && maxAccelerationRpsPerSec > 0.0) {
      profiles[slot] =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(maxVelocityRps, maxAccelerationRpsPerSec));
    } else {
      profiles[slot] = null;
    }
  }

  /** Output range of the loop, as a fraction of the nominal voltage (for example -1 and 1). */
  public void setOutputRange(double min, double max) {
    this.minOutput = min;
    this.maxOutput = max;
  }

  /** True when the slot has a motion profile (cruise velocity and acceleration above zero). */
  public boolean hasProfile(int slot) {
    return profiles[slot] != null;
  }

  // ---------------------------------------------------------------------------
  // Loop
  // ---------------------------------------------------------------------------

  /**
   * Clears the integrator and restarts the motion profile from the present state. Call it while the
   * robot is disabled so nothing accumulates.
   */
  public void reset(double position, double velocityRps) {
    integral = 0.0;
    hasPreviousError = false;
    previousError = 0.0;
    profileState.position = position;
    profileState.velocity = velocityRps;
    profileAcceleration = 0.0;
  }

  /**
   * Runs one step of the loop.
   *
   * @param mode {@link MotorControlMode#VELOCITY}, {@link MotorControlMode#POSITION} or {@link
   *     MotorControlMode#SMART_POSITION}; any other mode returns zero
   * @param target target velocity (rotations per second) or position (rotations)
   * @param slot gain slot
   * @param position measured position, in rotations
   * @param velocityRps measured velocity, in rotations per second
   * @param dtSeconds time since the previous step
   * @return voltage to apply, already limited to the output range
   */
  public double calculate(
      MotorControlMode mode,
      double target,
      int slot,
      double position,
      double velocityRps,
      double dtSeconds) {
    if (mode != activeMode || slot != activeSlot) {
      // New loop: nothing from the previous one may leak into it.
      reset(position, velocityRps);
      activeMode = mode;
      activeSlot = slot;
    }

    // Velocity gains: per (rotation per second) for VOLTS, per RPM for SPARK.
    double velocityUnits = style == Style.SPARK ? 60.0 : 1.0;
    double pid;
    double feedforward;

    switch (mode) {
      case VELOCITY -> {
        double error = (target - velocityRps) * velocityUnits;
        pid = pid(slot, error, Double.NaN, dtSeconds);
        // Phoenix 6 adds the gravity gain in every closed loop; a Spark only in position modes.
        feedforward =
            kS[slot] * Math.signum(target)
                + kV[slot] * target * velocityUnits
                + (style == Style.SPARK ? 0.0 : gravity(slot, position));
      }
      case SMART_POSITION -> {
        if (profiles[slot] == null) {
          // No profile configured: behaves like a plain position loop.
          pid = pid(slot, positionError(target, position), -velocityRps, dtSeconds);
          feedforward = gravity(slot, position);
        } else {
          advanceProfile(slot, target, dtSeconds);
          double error = profileState.position - position;
          pid = pid(slot, error, profileState.velocity - velocityRps, dtSeconds);
          feedforward =
              kS[slot] * Math.signum(profileState.velocity)
                  + kV[slot] * profileState.velocity * velocityUnits
                  + kA[slot] * profileAcceleration * velocityUnits
                  + gravity(slot, position);
        }
      }
      case POSITION -> {
        pid = pid(slot, positionError(target, position), -velocityRps, dtSeconds);
        feedforward = gravity(slot, position);
      }
      default -> {
        return 0.0;
      }
    }

    double volts = (style == Style.SPARK ? pid * nominalVolts : pid) + feedforward;
    return Math.max(minOutput * nominalVolts, Math.min(maxOutput * nominalVolts, volts));
  }

  private double positionError(double target, double position) {
    double error = target - position;
    if (wrap && wrapRange > 0.0) {
      error = Math.IEEEremainder(error, wrapRange);
    }
    return error;
  }

  /**
   * @param errorRate rate of change of the error per second, or NaN to differentiate the error
   */
  private double pid(int slot, double error, double errorRate, double dt) {
    double limit = style == Style.SPARK ? 1.0 : nominalVolts;
    double derivative;

    if (style == Style.SPARK) {
      // The Spark runs this 1000 times per second and does not scale by the time step.
      double iterations = dt * 1000.0;
      integral += kI[slot] * error * iterations;
      derivative = hasPreviousError ? kD[slot] * (error - previousError) / iterations : 0.0;
    } else {
      integral += kI[slot] * error * dt;
      double rate =
          Double.isNaN(errorRate)
              ? (hasPreviousError && dt > 0.0 ? (error - previousError) / dt : 0.0)
              : errorRate;
      derivative = kD[slot] * rate;
    }
    // The integrator alone never asks for more than the whole output.
    integral = Math.max(-limit, Math.min(limit, integral));

    previousError = error;
    hasPreviousError = true;
    return kP[slot] * error + integral + derivative;
  }

  private void advanceProfile(int slot, double target, double dt) {
    profileGoal.position = target;
    profileGoal.velocity = 0.0;
    double previousVelocity = profileState.velocity;
    profileState = profiles[slot].calculate(dt, profileState, profileGoal);
    profileAcceleration = dt > 0.0 ? (profileState.velocity - previousVelocity) / dt : 0.0;
  }

  private double gravity(int slot, double position) {
    double gain = kG[slot];
    if (gain == 0.0) {
      return 0.0;
    }
    return gravityType == GravityType.ARM ? gain * Math.cos(position * 2.0 * Math.PI) : gain;
  }

  /** Position the motion profile is at, in rotations (the setpoint the PID is following). */
  public double getProfilePosition() {
    return profileState.position;
  }

  /** Velocity of the motion profile, in rotations per second. */
  public double getProfileVelocity() {
    return profileState.velocity;
  }
}
