package org.ciprix.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

/**
 * Physics of one mechanism driven by a motor, for simulation: a roller, a flywheel, an arm or an
 * elevator.
 *
 * <p>It models what matters to test robot code: motor torque and back-EMF, inertia, gravity,
 * friction, hard stops, the current limit and an optional extra load (to simulate a jam). It is
 * pure math with an exact integration step, so it is stable with any gear ratio, costs almost
 * nothing and runs in unit tests without the HAL.
 *
 * <pre>{@code
 * // A roller: Kraken X60, 3:1, light.
 * SimMechanism roller = SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001);
 *
 * // An arm: NEO Vortex, 50:1, 4 kg and 0.5 m long, moving between -10 and 100 degrees.
 * SimMechanism arm =
 *     SimMechanism.arm(DCMotor.getNeoVortex(1), 50.0, 4.0, 0.5)
 *         .withLimits(-10.0 / 360.0, 100.0 / 360.0)
 *         .withStartPosition(-10.0 / 360.0);
 *
 * // An elevator: 2 Krakens, 9:1, 6 kg carriage, 2.5 cm drum, up to 1.2 m.
 * SimMechanism elevator =
 *     SimMechanism.elevator(DCMotor.getKrakenX60(2), 9.0, 6.0, 0.025)
 *         .withLimits(0.0, 1.2 / (2 * Math.PI * 0.025));
 * }</pre>
 *
 * <p>Positions are in rotations of the mechanism (the output of the gearbox). For an arm, zero is
 * horizontal and positive is up. For an elevator, one rotation of the drum lifts the carriage by
 * its circumference.
 */
public final class SimMechanism {

  private static final double GRAVITY = 9.80665;
  private static final double REST_SPEED = 1e-4; // rad/s below which the mechanism is "at rest"

  private final DCMotor motor;
  private final double gearing;
  private double inertia;

  private DoubleUnaryOperator gravityTorque = angle -> 0.0;
  private double coulombFriction = 0.0;
  private double viscousFriction = 0.0;
  private double minAngle = Double.NEGATIVE_INFINITY;
  private double maxAngle = Double.POSITIVE_INFINITY;
  private DoubleSupplier load = () -> 0.0;

  // State, at the output of the gearbox.
  private double angle = 0.0; // rad
  private double velocity = 0.0; // rad/s
  private double current = 0.0; // A, positive when it pushes in the positive direction
  private double appliedVolts = 0.0;

  private SimMechanism(DCMotor motor, double gearing, double inertia) {
    if (gearing <= 0.0) {
      throw new IllegalArgumentException("gearing must be positive, got " + gearing);
    }
    if (inertia <= 0.0) {
      throw new IllegalArgumentException("inertia must be positive, got " + inertia);
    }
    this.motor = motor;
    this.gearing = gearing;
    this.inertia = inertia;
  }

  // ---------------------------------------------------------------------------
  // Factories
  // ---------------------------------------------------------------------------

  /**
   * A mechanism that only has inertia: rollers, conveyors, flywheels.
   *
   * @param motor motor model, for example {@code DCMotor.getKrakenX60(1)}
   * @param gearing motor turns per turn of the mechanism (greater than 1 for a reduction)
   * @param inertiaKgM2 moment of inertia at the mechanism (0.001 for a light roller, 0.005 for a
   *     flywheel)
   */
  public static SimMechanism roller(DCMotor motor, double gearing, double inertiaKgM2) {
    return new SimMechanism(motor, gearing, inertiaKgM2);
  }

  /** Same as {@link #roller}: a flywheel is a roller with more inertia. */
  public static SimMechanism flywheel(DCMotor motor, double gearing, double inertiaKgM2) {
    return new SimMechanism(motor, gearing, inertiaKgM2);
  }

  /**
   * An arm that rotates around one end, with gravity. Zero is horizontal and positive is up.
   *
   * @param motor motor model
   * @param gearing motor turns per turn of the arm
   * @param massKg mass of the arm
   * @param lengthMeters length of the arm (the mass is considered evenly distributed)
   */
  public static SimMechanism arm(
      DCMotor motor, double gearing, double massKg, double lengthMeters) {
    double inertia = massKg * lengthMeters * lengthMeters / 3.0;
    SimMechanism mechanism = new SimMechanism(motor, gearing, inertia);
    double holdingTorque = massKg * GRAVITY * lengthMeters / 2.0;
    mechanism.gravityTorque = angle -> holdingTorque * Math.cos(angle);
    return mechanism;
  }

  /**
   * An elevator moved by a drum (or sprocket), with gravity. One rotation of the mechanism lifts
   * the carriage by the circumference of the drum.
   *
   * @param motor motor model
   * @param gearing motor turns per turn of the drum
   * @param carriageMassKg mass that moves
   * @param drumRadiusMeters radius of the drum or sprocket
   */
  public static SimMechanism elevator(
      DCMotor motor, double gearing, double carriageMassKg, double drumRadiusMeters) {
    double inertia = carriageMassKg * drumRadiusMeters * drumRadiusMeters;
    SimMechanism mechanism = new SimMechanism(motor, gearing, inertia);
    double weightTorque = carriageMassKg * GRAVITY * drumRadiusMeters;
    mechanism.gravityTorque = angle -> weightTorque;
    return mechanism;
  }

  // ---------------------------------------------------------------------------
  // Options
  // ---------------------------------------------------------------------------

  /** Replaces the moment of inertia at the mechanism, in kg m². */
  public SimMechanism withInertia(double inertiaKgM2) {
    if (inertiaKgM2 <= 0.0) {
      throw new IllegalArgumentException("inertia must be positive, got " + inertiaKgM2);
    }
    this.inertia = inertiaKgM2;
    return this;
  }

  /**
   * Adds friction at the mechanism.
   *
   * @param coulombNm constant torque that opposes the motion (also holds the mechanism at rest)
   * @param viscousNmPerRadPerSec torque proportional to the speed
   */
  public SimMechanism withFriction(double coulombNm, double viscousNmPerRadPerSec) {
    this.coulombFriction = Math.abs(coulombNm);
    this.viscousFriction = Math.abs(viscousNmPerRadPerSec);
    return this;
  }

  /** Hard stops, in rotations of the mechanism. */
  public SimMechanism withLimits(double minRotations, double maxRotations) {
    this.minAngle = Math.min(minRotations, maxRotations) * 2.0 * Math.PI;
    this.maxAngle = Math.max(minRotations, maxRotations) * 2.0 * Math.PI;
    this.angle = Math.max(minAngle, Math.min(maxAngle, angle));
    return this;
  }

  /** Where the mechanism is when the simulation starts, in rotations of the mechanism. */
  public SimMechanism withStartPosition(double rotations) {
    this.angle = Math.max(minAngle, Math.min(maxAngle, rotations * 2.0 * Math.PI));
    return this;
  }

  /**
   * Replaces the gravity model by a custom one.
   *
   * @param torqueFromRotations torque (N·m) that pulls the mechanism in the negative direction, as
   *     a function of its position in rotations
   */
  public SimMechanism withGravity(DoubleUnaryOperator torqueFromRotations) {
    this.gravityTorque = angle -> torqueFromRotations.applyAsDouble(angle / (2.0 * Math.PI));
    return this;
  }

  /**
   * Adds a load that opposes the motion, in N·m at the mechanism, read every step. Use it to
   * simulate a game piece being pulled in, or a jam (a very large value stalls the motor).
   */
  public SimMechanism withLoad(DoubleSupplier resistingTorqueNm) {
    this.load = resistingTorqueNm == null ? () -> 0.0 : resistingTorqueNm;
    return this;
  }

  // ---------------------------------------------------------------------------
  // Stepping
  // ---------------------------------------------------------------------------

  /**
   * Advances the simulation with a voltage applied to the motor.
   *
   * @param volts voltage requested by the controller
   * @param dtSeconds time step
   * @param currentLimitAmps motor current limit (zero or negative disables it)
   * @param batteryVolts voltage available from the battery
   */
  public void step(double volts, double dtSeconds, double currentLimitAmps, double batteryVolts) {
    double available = Math.abs(batteryVolts);
    double request = Math.max(-available, Math.min(available, volts));
    advance(request, false, dtSeconds, currentLimitAmps);
  }

  /**
   * Advances the simulation with the motor in neutral.
   *
   * @param dtSeconds time step
   * @param brake true to brake (motor leads shorted), false to coast
   */
  public void stepNeutral(double dtSeconds, boolean brake) {
    advance(0.0, !brake, dtSeconds, 0.0);
  }

  private void advance(double volts, boolean coast, double dt, double currentLimit) {
    double kt = motor.KtNMPerAmp;
    double kv = motor.KvRadPerSecPerVolt;
    double resistance = motor.rOhms;

    double backEmf = velocity * gearing / kv;
    double unlimitedCurrent = coast ? 0.0 : (volts - backEmf) / resistance;
    boolean limited = !coast && currentLimit > 0.0 && Math.abs(unlimitedCurrent) > currentLimit;
    double startCurrent =
        limited ? Math.copySign(currentLimit, unlimitedCurrent) : unlimitedCurrent;

    double gravity = gravityTorque.applyAsDouble(angle);
    double resisting = coulombFriction + Math.max(0.0, load.getAsDouble());

    // Torque that tries to move the mechanism, before friction.
    double driving = gearing * kt * startCurrent - gravity - viscousFriction * velocity;

    double frictionTorque;
    if (Math.abs(velocity) > REST_SPEED) {
      frictionTorque = Math.copySign(resisting, velocity);
    } else if (Math.abs(driving) <= resisting) {
      // Not enough torque to break away: the mechanism stays where it is.
      velocity = 0.0;
      finish(volts, coast, limited, currentLimit);
      return;
    } else {
      frictionTorque = Math.copySign(resisting, driving);
    }

    // dω/dt = a - k ω, integrated exactly over the step.
    double a;
    double k;
    if (coast || limited) {
      a = (gearing * kt * startCurrent - gravity - frictionTorque) / inertia;
      k = viscousFriction / inertia;
    } else {
      a = (gearing * kt * volts / resistance - gravity - frictionTorque) / inertia;
      k = (gearing * gearing * kt / (kv * resistance) + viscousFriction) / inertia;
    }

    double oldVelocity = velocity;
    double newVelocity;
    double newAngle;
    if (k > 1e-9) {
      double decay = Math.exp(-k * dt);
      double finalVelocity = a / k;
      newVelocity = finalVelocity + (oldVelocity - finalVelocity) * decay;
      newAngle = angle + finalVelocity * dt + (oldVelocity - finalVelocity) * (1.0 - decay) / k;
    } else {
      newVelocity = oldVelocity + a * dt;
      newAngle = angle + oldVelocity * dt + 0.5 * a * dt * dt;
    }

    // Friction cannot push the mechanism backwards: when the speed crosses zero, it stops.
    if (resisting > 0.0
        && oldVelocity != 0.0
        && Math.signum(newVelocity) != Math.signum(oldVelocity)) {
      newVelocity = 0.0;
    }

    if (newAngle <= minAngle) {
      newAngle = minAngle;
      newVelocity = Math.max(0.0, newVelocity);
    } else if (newAngle >= maxAngle) {
      newAngle = maxAngle;
      newVelocity = Math.min(0.0, newVelocity);
    }

    angle = newAngle;
    velocity = newVelocity;
    finish(volts, coast, limited, currentLimit);
  }

  /** Records the electrical state at the end of the step. */
  private void finish(double volts, boolean coast, boolean wasLimited, double currentLimit) {
    double backEmf = velocity * gearing / motor.KvRadPerSecPerVolt;
    if (coast) {
      current = 0.0;
      appliedVolts = 0.0;
      return;
    }
    double value = (volts - backEmf) / motor.rOhms;
    if (currentLimit > 0.0 && Math.abs(value) > currentLimit) {
      value = Math.copySign(currentLimit, value);
    }
    current = value;
    // The controller lowers the voltage to respect the limit.
    appliedVolts = value * motor.rOhms + backEmf;
    if (!wasLimited && Math.abs(appliedVolts - volts) < 1e-9) {
      appliedVolts = volts;
    }
  }

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------

  /** Position of the mechanism, in rotations. */
  public double getPositionRotations() {
    return angle / (2.0 * Math.PI);
  }

  /** Velocity of the mechanism, in rotations per second. */
  public double getVelocityRps() {
    return velocity / (2.0 * Math.PI);
  }

  /** Position of the motor shaft, in rotations. */
  public double getRotorPositionRotations() {
    return getPositionRotations() * gearing;
  }

  /** Velocity of the motor shaft, in rotations per second. */
  public double getRotorVelocityRps() {
    return getVelocityRps() * gearing;
  }

  /** Motor (stator) current, in amps. Positive when it pushes in the positive direction. */
  public double getCurrentAmps() {
    return current;
  }

  /** Voltage actually applied to the motor (after the battery and the current limit). */
  public double getAppliedVolts() {
    return appliedVolts;
  }

  /**
   * Current drawn from the battery, in amps (never negative).
   *
   * @param batteryVolts present battery voltage
   */
  public double getSupplyCurrentAmps(double batteryVolts) {
    if (Math.abs(batteryVolts) < 1e-6) {
      return 0.0;
    }
    return Math.max(0.0, current * appliedVolts / Math.abs(batteryVolts));
  }

  /** Motor turns per turn of the mechanism. */
  public double getGearing() {
    return gearing;
  }

  /** The motor model. */
  public DCMotor getMotor() {
    return motor;
  }

  /** Moves the mechanism to a position and speed (intended for tests). */
  public void setState(double positionRotations, double velocityRps) {
    angle = Math.max(minAngle, Math.min(maxAngle, positionRotations * 2.0 * Math.PI));
    velocity = velocityRps * 2.0 * Math.PI;
  }
}
