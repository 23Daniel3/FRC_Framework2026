package org.ciprix.interfaces.motor.basic;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ciprix.CiprixSettings;
import org.ciprix.interfaces.HardwareHealthMonitor;
import org.ciprix.interfaces.motor.MotorControlMode;
import org.ciprix.logger.LoggedTunableNumber;
import org.ciprix.util.RobotClock;
import org.littletonrobotics.junction.Logger;

/**
 * Shared base for every motor IO in this package — both {@link BasicMotorIO}-only implementations
 * (CIM/RedLine via PWM, Talon SRX, SparkMax brushed...) and, through {@link
 * org.ciprix.interfaces.motor.advanced.MotorBase}, every closed-loop-capable implementation
 * (SparkFlex, TalonFX, ...).
 *
 * <p>This class owns the <b>intent</b> of the motor: the last command the robot code asked for. A
 * concrete motor only translates commands to its hardware ({@link #applyVoltage}, {@link
 * #applyPercentOutput}, {@link #applyStop}); everything else is written once here:
 *
 * <ul>
 *   <li>the hardware is only called when the command actually changes, so a state that repeats
 *       {@code runVelocity(...)} every cycle costs no CAN traffic while the value is the same. An
 *       unchanged command is still sent again every {@value #COMMAND_REFRESH_SECONDS} s, which
 *       covers a controller that rebooted;
 *   <li>the output range ({@code Name/Config/MinOutput|MaxOutput}) and the current limit ({@code
 *       Name/Config/CurrentLimit}) can be edited from the dashboard in tuning mode;
 *   <li>brake mode and current limit are only written to the controller when they change (those
 *       writes block the loop for a long time);
 *   <li>the motor registers itself in the {@link HardwareHealthMonitor}.
 * </ul>
 */
public abstract class BasicMotorBase implements BasicMotorIO {

  /** An unchanged command is sent again to the hardware at this interval. */
  public static final double COMMAND_REFRESH_SECONDS = 0.5;

  private static final String[] NO_FAULTS = new String[0];

  protected final String name;

  /** Voltage that a percent output of 1.0 means (see {@link BasicMotorConfig#nominalVoltage}). */
  protected final double nominalVolts;

  /** Mode of the command the robot code asked for last. */
  protected MotorControlMode currentMode = MotorControlMode.IDLE;

  protected double minOutput;
  protected double maxOutput;

  // Intent: what the robot code asked for last. Null until the first command, so a motor that is
  // never commanded (a follower, for example) is never touched.
  private MotorControlMode desiredMode = null;
  private double desiredValue = 0.0;
  private int desiredSlot = 0;

  // What the hardware was told last.
  private MotorControlMode appliedMode = null;
  private double appliedValue = Double.NaN;
  private int appliedSlot = -1;
  private double appliedTime = Double.NEGATIVE_INFINITY;

  // While true the hardware is driven by the library itself (un-jam) and the intent waits.
  private boolean outputOverridden = false;
  private double overrideValue = Double.NaN;

  private boolean brakeMode;
  private double currentLimitAmps;
  private final double temperatureWarningCelsius;

  private final LoggedTunableNumber minOutputTunable;
  private final LoggedTunableNumber maxOutputTunable;
  private final LoggedTunableNumber currentLimitTunable;

  private final String modeKey;
  private final String setpointKey;

  // Last state seen, read by the health monitor. A motor whose inputs are never read (a follower
  // nobody logs) is not judged: unknown is not the same as disconnected.
  private boolean inputsSeen = false;
  private boolean lastConnected = false;
  private String[] lastFaults = NO_FAULTS;
  private String lastProblem = "";
  private boolean overTemperature = false;

  private BasicMotorIOInputs lastBasicInputs = new BasicMotorIOInputs();

  /** Facade handed to subsystems: it can command the motor but cannot read or configure it. */
  private final class BasicControllerImpl implements BasicMotorController {

    @Override
    public void setBrakeMode(boolean enabled) {
      BasicMotorBase.this.setBrakeMode(enabled);
    }

    @Override
    public void runVoltage(Voltage volts) {
      BasicMotorBase.this.runVoltage(volts);
    }

    @Override
    public void runPercentOutput(double percent) {
      BasicMotorBase.this.runPercentOutput(percent);
    }

    @Override
    public void stop() {
      BasicMotorBase.this.stop();
    }

    @Override
    public void setCurrentLimit(Current current) {
      BasicMotorBase.this.setCurrentLimit(current);
    }
  }

  private final BasicMotorController controller = new BasicControllerImpl();

  public BasicMotorBase(String name, BasicMotorConfig config) {
    this.name = name;
    this.nominalVolts = Math.abs(config.nominalVoltage.in(Volts));
    this.minOutput = config.minOutput;
    this.maxOutput = config.maxOutput;
    this.brakeMode = config.brakeMode;
    this.currentLimitAmps = config.currentLimit.in(Amps);
    this.temperatureWarningCelsius = config.temperatureWarning.in(Celsius);

    this.minOutputTunable = new LoggedTunableNumber(name + "/Config/MinOutput", config.minOutput);
    this.maxOutputTunable = new LoggedTunableNumber(name + "/Config/MaxOutput", config.maxOutput);
    this.currentLimitTunable =
        new LoggedTunableNumber(name + "/Config/CurrentLimit", currentLimitAmps);

    this.modeKey = "Motors/" + name + "/Mode";
    this.setpointKey = "Motors/" + name + "/Setpoint";

    HardwareHealthMonitor.register(
        name, () -> lastConnected || !inputsSeen, () -> lastFaults, () -> lastProblem);
  }

  // ---------------------------------------------------------------------------
  // Commands (what the robot code calls)
  // ---------------------------------------------------------------------------

  /**
   * Applies a voltage. The request is scaled by the output range like a percent output, and never
   * exceeds the nominal voltage.
   */
  @Override
  public final void runVoltage(Voltage volts) {
    command(MotorControlMode.VOLTAGE, volts.in(Volts), 0);
  }

  /**
   * Applies a percent output in [-1, 1], where 1.0 means the nominal voltage. The request is scaled
   * by the output range, so the full [-1, 1] input stays usable (a joystick, for example) even when
   * the motor is limited to a fraction of its power.
   */
  @Override
  public final void runPercentOutput(double percent) {
    command(MotorControlMode.PERCENT, Math.max(-1.0, Math.min(1.0, percent)), 0);
  }

  /** Stops the motor (neutral: brake or coast, as configured). */
  @Override
  public final void stop() {
    command(MotorControlMode.IDLE, 0.0, 0);
  }

  /** Changes brake/coast. The controller is only written when the value changes. */
  @Override
  public final void setBrakeMode(boolean enabled) {
    if (enabled == brakeMode) {
      return;
    }
    brakeMode = enabled;
    applyBrakeMode(enabled);
  }

  /** Changes the current limit. The controller is only written when the value changes. */
  @Override
  public final void setCurrentLimit(Current current) {
    double amps = Math.abs(current.in(Amps));
    if (amps == currentLimitAmps) {
      return;
    }
    currentLimitAmps = amps;
    applyCurrentLimit(amps);
  }

  /**
   * Records the intent and sends it to the hardware when it differs from what the hardware was told
   * last. Every {@code run...} method of the hierarchy ends up here.
   */
  protected final void command(MotorControlMode mode, double value, int slot) {
    desiredMode = mode;
    desiredValue = value;
    desiredSlot = slot;
    currentMode = mode;
    if (!outputOverridden) {
      flush(false);
    }
  }

  private void flush(boolean force) {
    if (desiredMode == null) {
      return;
    }
    if (!force
        && desiredMode == appliedMode
        && desiredSlot == appliedSlot
        && desiredValue == appliedValue) {
      return;
    }
    boolean changed = desiredMode != appliedMode || desiredValue != appliedValue;
    sendToHardware(desiredMode, desiredValue, desiredSlot);
    appliedMode = desiredMode;
    appliedValue = desiredValue;
    appliedSlot = desiredSlot;
    appliedTime = RobotClock.now();
    if (changed) {
      Logger.recordOutput(modeKey, desiredMode);
      Logger.recordOutput(setpointKey, desiredValue);
    }
  }

  /**
   * Translates a command to the hardware hooks. {@link
   * org.ciprix.interfaces.motor.advanced.MotorBase} extends it with the closed-loop modes.
   */
  protected void sendToHardware(MotorControlMode mode, double value, int slot) {
    switch (mode) {
      case VOLTAGE -> applyVoltage(mapVoltage(value));
      case PERCENT -> applyPercentOutput(mapOutput(value));
      default -> applyStop();
    }
  }

  /** Makes the next command (or the next refresh) reach the hardware even if it did not change. */
  protected final void invalidateCommand() {
    appliedMode = null;
    appliedValue = Double.NaN;
    appliedSlot = -1;
  }

  /** How often an unchanged command is sent again. Zero sends it every cycle. */
  protected double getCommandRefreshSeconds() {
    return COMMAND_REFRESH_SECONDS;
  }

  /** Sends the intent again when it is time. Called once per cycle from {@code updateInputs}. */
  protected final void refreshCommand(double now) {
    if (desiredMode == null || outputOverridden) {
      return;
    }
    if (appliedMode == null || now - appliedTime >= getCommandRefreshSeconds()) {
      flush(true);
    }
  }

  /**
   * Drives the motor with a percent output chosen by the library (the un-jam reversal), ignoring
   * the intent until {@link #releaseOutputOverride()}. Commands received meanwhile are remembered
   * and applied when the override ends.
   */
  protected final void overrideOutput(double percent) {
    if (outputOverridden && percent == overrideValue) {
      return;
    }
    outputOverridden = true;
    overrideValue = percent;
    applyPercentOutput(mapOutput(percent));
  }

  /** Ends {@link #overrideOutput(double)} and goes back to what the robot code asked for last. */
  protected final void releaseOutputOverride() {
    if (!outputOverridden) {
      return;
    }
    outputOverridden = false;
    overrideValue = Double.NaN;
    invalidateCommand();
    if (desiredMode == null) {
      applyStop();
    } else {
      flush(true);
    }
  }

  /** True while the library itself is driving the motor (un-jam). */
  protected final boolean isOutputOverridden() {
    return outputOverridden;
  }

  /** Mode the robot code asked for last, or null when the motor was never commanded. */
  protected final MotorControlMode getDesiredMode() {
    return desiredMode;
  }

  /** True when the robot code asked for something other than "stop". */
  protected final boolean isCommanded() {
    return desiredMode != null && desiredMode != MotorControlMode.IDLE;
  }

  // ---------------------------------------------------------------------------
  // Inputs
  // ---------------------------------------------------------------------------

  @Override
  public void updateInputs(BasicMotorIOInputs inputs) {
    checkConfigTuning();
    updateHardwareInputs(inputs);
    lastBasicInputs = inputs;

    updateTemperatureWarning(inputs.temperature.in(Celsius));
    String problem =
        !inputs.isConnected ? "Disconnected" : overTemperature ? "Over temperature" : "";
    publishHealth(inputs, problem);
    refreshCommand(RobotClock.now());
  }

  private void updateTemperatureWarning(double celsius) {
    if (temperatureWarningCelsius <= 0.0) {
      overTemperature = false;
    } else if (celsius > temperatureWarningCelsius) {
      overTemperature = true;
    } else if (celsius < temperatureWarningCelsius - 5.0) {
      overTemperature = false;
    }
  }

  /** Writes the problem to the inputs and keeps the copy read by the health monitor. */
  protected final void publishHealth(BasicMotorIOInputs inputs, String problem) {
    inputs.problem = problem;
    inputs.hasProblem = !problem.isEmpty();
    inputsSeen = true;
    lastConnected = inputs.isConnected;
    lastFaults = inputs.activeFaults == null ? NO_FAULTS : inputs.activeFaults;
    lastProblem = problem;
  }

  /** Inputs object given to {@code updateInputs} last (the one the subsystem logs). */
  @Override
  public BasicMotorIOInputs getMotorIOInputs() {
    return lastBasicInputs;
  }

  // ---------------------------------------------------------------------------
  // Output range and live tuning
  // ---------------------------------------------------------------------------

  /**
   * Maps a requested percent output [-1.0, 1.0] to the configured (and live-tunable) output range.
   * This is a proportional mapping (scaling), so the full [-1.0, 1.0] input range stays usable
   * (e.g. from a joystick) without hitting a hard flat ceiling: with {@code maxOutput = 0.5}, a
   * request of 1.0 becomes 0.5 and a request of 0.5 becomes 0.25.
   */
  protected double mapOutput(double percent) {
    if (percent > 0) {
      return percent * maxOutput;
    } else {
      // minOutput is typically negative
      return percent * Math.abs(minOutput);
    }
  }

  /**
   * Maps a requested voltage to the configured output range, with the same proportional scaling as
   * percent output (with {@code maxOutput = 0.5}, a request for 12 V applies 6 V), and never above
   * the nominal voltage.
   */
  protected double mapVoltage(double volts) {
    double scaled = volts > 0 ? volts * maxOutput : volts * Math.abs(minOutput);
    return Math.max(-nominalVolts, Math.min(nominalVolts, scaled));
  }

  /** Re-checks the dashboard entries of the output range and current limit (tuning mode only). */
  protected final void checkConfigTuning() {
    if (!CiprixSettings.isTuningMode()) {
      return;
    }
    int id = hashCode();
    // Evaluate both (no short-circuit) so each one records its latest value.
    boolean rangeChanged = minOutputTunable.hasChanged(id) | maxOutputTunable.hasChanged(id);
    if (rangeChanged) {
      applyHardwareOutputRange(minOutputTunable.get(), maxOutputTunable.get());
    }
    if (currentLimitTunable.hasChanged(id)) {
      setCurrentLimit(Amps.of(currentLimitTunable.get()));
    }
  }

  /**
   * Called whenever the tunable output range changes. Updates the scaling used by {@link
   * #mapOutput(double)} and {@link #mapVoltage(double)} and sends the current command again, now
   * with the new range.
   *
   * <p>{@link org.ciprix.interfaces.motor.advanced.MotorBase} overrides this to additionally push
   * the range into every closed-loop slot on the hardware.
   */
  protected void applyHardwareOutputRange(double min, double max) {
    this.minOutput = min;
    this.maxOutput = max;
    invalidateCommand();
  }

  // ---------------------------------------------------------------------------
  // Hardware hooks (what a concrete motor implements)
  // ---------------------------------------------------------------------------

  /** Applies a voltage, already scaled and limited. */
  protected abstract void applyVoltage(double volts);

  /**
   * Applies a percent output in [-1, 1], already scaled, where 1.0 means the nominal voltage.
   * Controllers with voltage compensation should use it.
   */
  protected abstract void applyPercentOutput(double percent);

  /** Puts the motor in neutral. */
  protected abstract void applyStop();

  /** Writes brake/coast to the controller. Only called when the value changes. */
  protected void applyBrakeMode(boolean enabled) {}

  /** Writes the current limit to the controller. Only called when the value changes. */
  protected void applyCurrentLimit(double amps) {}

  /** Reads the hardware into {@code inputs}. */
  protected abstract void updateHardwareInputs(BasicMotorIOInputs inputs);

  /** Current limit in effect, in amps. */
  protected final double getCurrentLimitAmps() {
    return currentLimitAmps;
  }

  /** True when the motor is configured to brake in neutral. */
  protected final boolean isBrakeMode() {
    return brakeMode;
  }

  @Override
  public BasicMotorController getMotorController() {
    return controller;
  }
}
