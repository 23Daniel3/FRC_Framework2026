package org.ciprix.interfaces.motor.advanced;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ciprix.CiprixSettings;
import org.ciprix.interfaces.motor.MotorControlMode;
import org.ciprix.interfaces.motor.MotorHealth;
import org.ciprix.interfaces.motor.basic.BasicMotorBase;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;
import org.ciprix.logger.LoggedTunableNumber;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;
import org.ciprix.util.RobotStatus;

/**
 * Base for closed-loop-capable motor IOs. Extends {@link BasicMotorBase}, reusing the command
 * intent, the tunable output range and the current limit, and adds what closed-loop control needs:
 *
 * <ul>
 *   <li>velocity, position and profiled position commands, with up to {@value #SLOT_COUNT} gain
 *       slots. A concrete motor only translates them to its hardware ({@link #applyVelocity},
 *       {@link #applyPosition}, {@link #applySmartPosition});
 *   <li>gains editable from the dashboard in tuning mode ({@code Name/Slot0/PID/kP}, ...). Only the
 *       slots that have something configured are published, so the dashboard stays clean;
 *   <li>{@code atSetpoint}, computed the same way for every vendor from the configured tolerances;
 *   <li>the health of the motor ({@link MotorHealth}): average current, stall, automatic un-jam,
 *       free spin and temperature, written to the inputs ({@code hasProblem}, {@code stalled}, ...)
 *       so the state machines can react and the result is the same in replay.
 * </ul>
 */
public abstract class MotorBase extends BasicMotorBase implements MotorIO {

  /** Number of gain slots supported by the configuration. */
  public static final int SLOT_COUNT = 4;

  /** Target of the last position command, in rotations. */
  protected double targetPositionRotations = 0.0;

  /** Target of the last velocity command, in rotations per second. */
  protected double targetVelocityRps = 0.0;

  protected final double positionToleranceRotations;
  protected final double velocityToleranceRps;

  private final MotorConfig config;
  private final MotorHealth health;
  private final boolean positionWrap;
  private final double wrapRangeRotations;

  private final TunableSlot[] tunableSlots = new TunableSlot[SLOT_COUNT];

  private MotorIOInputs lastInputs = new MotorIOInputs();
  private boolean invalidSlotReported = false;

  /** Facade handed to subsystems: it can command the motor but cannot read or configure it. */
  private final class MotorControllerImpl implements MotorController {

    @Override
    public void setBrakeMode(boolean enabled) {
      MotorBase.this.setBrakeMode(enabled);
    }

    @Override
    public void runVoltage(Voltage volts) {
      MotorBase.this.runVoltage(volts);
    }

    @Override
    public void runPercentOutput(double percent) {
      MotorBase.this.runPercentOutput(percent);
    }

    @Override
    public void stop() {
      MotorBase.this.stop();
    }

    @Override
    public void setCurrentLimit(Current current) {
      MotorBase.this.setCurrentLimit(current);
    }

    @Override
    public void setOffset(Angle offset) {
      MotorBase.this.setOffset(offset);
    }

    @Override
    public void runVelocity(AngularVelocity velocity) {
      MotorBase.this.runVelocity(velocity, 0);
    }

    @Override
    public void runPosition(Angle position) {
      MotorBase.this.runPosition(position, 0);
    }

    @Override
    public void runSmartPosition(Angle position) {
      MotorBase.this.runSmartPosition(position, 0);
    }

    @Override
    public void runVelocity(AngularVelocity velocity, int slot) {
      MotorBase.this.runVelocity(velocity, slot);
    }

    @Override
    public void runPosition(Angle position, int slot) {
      MotorBase.this.runPosition(position, slot);
    }

    @Override
    public void runSmartPosition(Angle position, int slot) {
      MotorBase.this.runSmartPosition(position, slot);
    }
  }

  private final MotorController controller = new MotorControllerImpl();

  public MotorBase(String name, MotorConfig config) {
    super(name, config);
    this.config = config;
    this.health = new MotorHealth(config);

    this.positionToleranceRotations = config.positionTolerance.in(Rotations);
    this.velocityToleranceRps = config.velocityTolerance.in(RotationsPerSecond);
    this.positionWrap = config.positionWrap;
    this.wrapRangeRotations =
        Math.abs(config.maxPosition.in(Rotations) - config.minPosition.in(Rotations));

    for (int i = 0; i < SLOT_COUNT; i++) {
      // Slot 0 is always tunable; the others only when the robot configured them.
      if (i == 0 || slotIsConfigured(config, i)) {
        tunableSlots[i] = new TunableSlot(name, i, config);
      }
    }
  }

  public MotorBase(String name, BasicMotorConfig config) {
    this(name, MotorConfig.fromBasic(config));
  }

  private static boolean slotIsConfigured(MotorConfig config, int slot) {
    return config.kP[slot] != 0
        || config.kI[slot] != 0
        || config.kD[slot] != 0
        || config.kS[slot] != 0
        || config.kV[slot] != 0
        || config.kA[slot] != 0
        || config.kG[slot] != 0
        || config.maxMotionMaxVelocity[slot].in(RotationsPerSecond) != 0;
  }

  // ---------------------------------------------------------------------------
  // Commands (what the robot code calls)
  // ---------------------------------------------------------------------------

  @Override
  public final void runVelocity(AngularVelocity velocity) {
    runVelocity(velocity, 0);
  }

  @Override
  public final void runPosition(Angle position) {
    runPosition(position, 0);
  }

  @Override
  public final void runSmartPosition(Angle position) {
    runSmartPosition(position, 0);
  }

  /** Closed-loop velocity with the gains of {@code slot}. */
  @Override
  public final void runVelocity(AngularVelocity velocity, int slot) {
    targetVelocityRps = velocity.in(RotationsPerSecond);
    command(MotorControlMode.VELOCITY, targetVelocityRps, checkSlot(slot));
  }

  /** Closed-loop position with the gains of {@code slot}. */
  @Override
  public final void runPosition(Angle position, int slot) {
    targetPositionRotations = position.in(Rotations);
    command(MotorControlMode.POSITION, targetPositionRotations, checkSlot(slot));
  }

  /**
   * Closed-loop position that follows a motion profile (Motion Magic, MAXMotion) with the gains and
   * the limits of {@code slot}.
   */
  @Override
  public final void runSmartPosition(Angle position, int slot) {
    targetPositionRotations = position.in(Rotations);
    command(MotorControlMode.SMART_POSITION, targetPositionRotations, checkSlot(slot));
  }

  private int checkSlot(int slot) {
    if (slot >= 0 && slot < SLOT_COUNT) {
      return slot;
    }
    if (!invalidSlotReported) {
      invalidSlotReported = true;
      Report.error("Motor " + name + ": slot " + slot + " does not exist, using slot 0.");
    }
    return 0;
  }

  @Override
  protected void sendToHardware(MotorControlMode mode, double value, int slot) {
    switch (mode) {
      case VELOCITY -> applyVelocity(value, slot);
      case POSITION -> applyPosition(value, slot);
      case SMART_POSITION -> applySmartPosition(value, slot);
      default -> super.sendToHardware(mode, value, slot);
    }
  }

  // ---------------------------------------------------------------------------
  // Inputs and health
  // ---------------------------------------------------------------------------

  /**
   * Reads the motor, computes {@code atSetpoint} and the health of the motor, runs the automatic
   * un-jam and applies gains changed from the dashboard. Call it once per cycle.
   */
  @Override
  public final void updateInputs(MotorIOInputs inputs) {
    double now = RobotClock.now();

    checkConfigTuning();
    if (CiprixSettings.isTuningMode()) {
      for (TunableSlot slot : tunableSlots) {
        if (slot != null) {
          slot.check();
        }
      }
    }

    updateHardwareInputs(inputs);
    lastInputs = inputs;
    inputs.atSetpoint = calculateAtSetpoint(inputs);

    boolean holding =
        inputs.atSetpoint
            && (currentMode == MotorControlMode.POSITION
                || currentMode == MotorControlMode.SMART_POSITION);
    health.update(
        now,
        RobotStatus.isEnabled(),
        isCommanded(),
        holding,
        inputs.appliedVolts.in(Volts),
        inputs.current.in(Amps),
        inputs.velocity.in(RotationsPerSecond),
        inputs.temperature.in(Celsius),
        inputs.isConnected);

    inputs.averageCurrentAmps = health.getAverageCurrentAmps();
    inputs.stalled = health.isStalled();
    inputs.unjamming = health.isUnjamming();
    inputs.freeSpinning = health.isFreeSpinning();
    publishHealth(inputs, health.getProblem());

    if (health.isUnjamming()) {
      if (!isOutputOverridden()) {
        Report.warning(
            "Motor "
                + name
                + " stalled: reversing to free the mechanism (attempt "
                + health.getUnjamCount()
                + ").");
      }
      overrideOutput(health.getUnjamOutput());
    } else {
      // Back to what the robot code asked for last (also covers commands received meanwhile).
      releaseOutputOverride();
    }

    refreshCommand(now);
  }

  private boolean calculateAtSetpoint(MotorIOInputs inputs) {
    switch (currentMode) {
      case POSITION:
        return positionError(inputs) < positionToleranceRotations;
      case SMART_POSITION:
        return positionError(inputs) < positionToleranceRotations
            && Math.abs(inputs.velocity.in(RotationsPerSecond)) < velocityToleranceRps;
      case VELOCITY:
        return Math.abs(inputs.velocity.in(RotationsPerSecond) - targetVelocityRps)
            < velocityToleranceRps;
      default:
        return false;
    }
  }

  private double positionError(MotorIOInputs inputs) {
    double error = inputs.position.in(Rotations) - targetPositionRotations;
    if (positionWrap && wrapRangeRotations > 0.0) {
      // Continuous mechanism (a turret, a swerve azimuth): the shortest way around counts.
      error = Math.IEEEremainder(error, wrapRangeRotations);
    }
    return Math.abs(error);
  }

  /** Inputs object given to {@link #updateInputs(MotorIOInputs)} last. */
  @Override
  public MotorIOInputs getMotorIOInputs() {
    return lastInputs;
  }

  /** The configuration this motor was created with. */
  protected final MotorConfig getConfig() {
    return config;
  }

  // ---------------------------------------------------------------------------
  // Hardware hooks (what a concrete motor implements)
  // ---------------------------------------------------------------------------

  /** Closed-loop velocity, in rotations per second of the mechanism. */
  protected abstract void applyVelocity(double rotationsPerSecond, int slot);

  /** Closed-loop position, in rotations of the mechanism. */
  protected abstract void applyPosition(double rotations, int slot);

  /** Profiled closed-loop position, in rotations of the mechanism. */
  protected abstract void applySmartPosition(double rotations, int slot);

  // Note: updateHardwareInputs(BasicMotorIOInputs) is still abstract, inherited from
  // BasicMotorBase. Concrete subclasses implement it with the common telemetry read, AND implement
  // the overload below, which should call the common one to reuse it — this keeps a caller holding
  // only a BasicMotorIO reference getting valid telemetry too.
  protected abstract void updateHardwareInputs(MotorIOInputs inputs);

  /** Fans the single tunable output range out to every closed-loop slot on the hardware. */
  @Override
  protected void applyHardwareOutputRange(double min, double max) {
    super.applyHardwareOutputRange(min, max); // keeps the software scaling in sync
    for (int i = 0; i < SLOT_COUNT; i++) {
      applyHardwareOutputRange(i, min, max);
    }
  }

  // ---------------------------------------------------------------------------
  // Live tuning
  // ---------------------------------------------------------------------------

  /** Dashboard entries of one gain slot. Pushes a group to the hardware when one of it changes. */
  private final class TunableSlot {
    private final int slot;
    private final LoggedTunableNumber kP, kI, kD;
    private final LoggedTunableNumber kS, kV, kA, kG;
    private final LoggedTunableNumber maxVelocity, maxAcceleration, allowedError;

    TunableSlot(String motorName, int slot, MotorConfig config) {
      this.slot = slot;
      String prefix = motorName + "/Slot" + slot;
      kP = new LoggedTunableNumber(prefix + "/PID/kP", config.kP[slot]);
      kI = new LoggedTunableNumber(prefix + "/PID/kI", config.kI[slot]);
      kD = new LoggedTunableNumber(prefix + "/PID/kD", config.kD[slot]);
      kS = new LoggedTunableNumber(prefix + "/FF/kS", config.kS[slot]);
      kV = new LoggedTunableNumber(prefix + "/FF/kV", config.kV[slot]);
      kA = new LoggedTunableNumber(prefix + "/FF/kA", config.kA[slot]);
      kG = new LoggedTunableNumber(prefix + "/FF/kG", config.kG[slot]);
      maxVelocity =
          new LoggedTunableNumber(
              prefix + "/Smart/MaxVel", config.maxMotionMaxVelocity[slot].in(RotationsPerSecond));
      maxAcceleration =
          new LoggedTunableNumber(
              prefix + "/Smart/MaxAccel",
              config.maxMotionMaxAcceleration[slot].in(RotationsPerSecondPerSecond));
      allowedError =
          new LoggedTunableNumber(
              prefix + "/Smart/AllowedErr",
              config.maxMotionAllowedClosedLoopError[slot].in(Rotations));
    }

    void check() {
      int id = hashCode();
      // Non short-circuit "|" on purpose: every tunable must record its latest value.
      if (kP.hasChanged(id) | kI.hasChanged(id) | kD.hasChanged(id)) {
        applyHardwarePID(slot, kP.get(), kI.get(), kD.get());
      }
      if (kS.hasChanged(id) | kV.hasChanged(id) | kA.hasChanged(id) | kG.hasChanged(id)) {
        applyHardwareSVAG(slot, kS.get(), kV.get(), kA.get(), kG.get());
      }
      if (maxVelocity.hasChanged(id)
          | maxAcceleration.hasChanged(id)
          | allowedError.hasChanged(id)) {
        applyHardwareSmartMotion(
            slot, maxVelocity.get(), maxAcceleration.get(), allowedError.get());
      }
    }
  }

  @Override
  public MotorController getMotorController() {
    return controller;
  }
}
