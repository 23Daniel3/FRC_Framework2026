package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;
import org.ciprix.CiprixSettings;
import org.ciprix.interfaces.encoder.EncoderConfig;
import org.ciprix.interfaces.encoder.EncoderIO;
import org.ciprix.interfaces.encoder.EncoderIOSparkAbsolute;
import org.ciprix.interfaces.motor.advanced.MotorBase;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorConfig.FeedbackSensorType;
import org.ciprix.interfaces.motor.advanced.MotorConfig.GravityType;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;

/**
 * Everything the SPARK MAX and the SPARK Flex have in common. {@link MotorIOSparkMax} and {@link
 * MotorIOSparkFlex} only choose the controller and its secondary encoder port.
 *
 * <p>Units follow REVLib: positions in rotations and velocities in RPM, both multiplied by the
 * conversion factors of the configuration. The gains use the REV convention: the PID output is a
 * duty cycle, kS/kG in volts, kV in volts per RPM and kA in volts per RPM per second. The gravity
 * gain is applied as {@code kG} (elevator) or {@code kCos} (arm, zero at the horizontal) according
 * to {@link MotorConfig#gravityType}.
 *
 * <p>Configuration:
 *
 * <ul>
 *   <li>at boot, the whole configuration is written once (safe parameters reset first) and saved to
 *       flash, so a controller that reboots in the middle of a match comes back configured;
 *   <li>changes made while the robot runs (dashboard tuning, brake mode, current limit) are sent
 *       without blocking the loop and are not saved to flash;
 *   <li>the faults are cleared at boot, so the warning "Motor Reset Detected" means that the
 *       controller rebooted during the match (usually a brownout).
 * </ul>
 */
public abstract class MotorIOSparkBase extends MotorBase {

  /** A controller that does not answer for this long counts as disconnected. */
  public static final double DISCONNECT_TIMEOUT_SECONDS = 0.5;

  private static final int CONFIG_ATTEMPTS = 5;
  private static final String[] NO_FAULTS = new String[0];

  /**
   * The secondary encoder port of a controller: the alternate encoder of a SPARK MAX or the
   * external encoder of a SPARK Flex.
   */
  protected interface SecondaryEncoderPort {
    /** Writes the encoder settings to {@code config}. */
    void configure(
        SparkBaseConfig config,
        int countsPerRevolution,
        double positionConversionFactor,
        double velocityConversionFactor);

    /** Opens the encoder. Only called after the configuration reached the controller. */
    RelativeEncoder open();
  }

  protected final SparkBase motor;

  private final SparkClosedLoopController closedLoop;
  private final Supplier<SparkBaseConfig> configFactory;
  private final FeedbackSensorType sensorType;
  private final GravityType gravityType;
  private final RelativeEncoder relativeEncoder;
  private final AbsoluteEncoder absoluteEncoder;

  /** Cruise velocity of each slot, in rotations per second (0 = no MAXMotion profile). */
  private final double[] cruiseVelocityRps = new double[SLOT_COUNT];

  private boolean readsOk = true;
  private double lastAnswerTime = Double.NEGATIVE_INFINITY;
  private int lastFaultBits = 0;
  private int lastWarningBits = 0;
  private String[] faults = NO_FAULTS;
  private boolean missingProfileReported = false;
  private boolean absoluteOffsetReported = false;

  /**
   * @param name name used in the logs, the dashboard and the alerts
   * @param motor the controller, already created
   * @param configFactory creates an empty configuration of the right type ({@code
   *     SparkMaxConfig::new} or {@code SparkFlexConfig::new})
   * @param secondaryPort the alternate (MAX) or external (Flex) encoder port
   * @param config the configuration of the motor
   */
  protected MotorIOSparkBase(
      String name,
      SparkBase motor,
      Supplier<SparkBaseConfig> configFactory,
      SecondaryEncoderPort secondaryPort,
      MotorConfig config) {
    super(name, config);
    this.motor = motor;
    this.closedLoop = motor.getClosedLoopController();
    this.configFactory = configFactory;
    this.sensorType = resolveSensor(name, config);
    this.gravityType = config.gravityType;

    SparkBaseConfig boot = configFactory.get();
    boot.inverted(config.inverted)
        .idleMode(config.brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(toSmartLimit(config.currentLimit.in(Amps)))
        .voltageCompensation(nominalVolts)
        .openLoopRampRate(Math.max(0.0, config.openLoopRampSeconds))
        .closedLoopRampRate(Math.max(0.0, config.closedLoopRampSeconds));
    if (config.leaderMotorID != 0) {
      boot.follow(config.leaderMotorID, config.followerInverted);
    }

    int periodMs = (int) Math.max(5, Math.round(CiprixSettings.getLoopPeriodSeconds() * 1000.0));
    boot.signals
        .appliedOutputPeriodMs(periodMs)
        .busVoltagePeriodMs(periodMs)
        .outputCurrentPeriodMs(periodMs);

    double positionFactor = config.positionConversionFactor;
    double velocityFactor = config.velocityConversionFactor;
    switch (sensorType) {
      case ALTERNATE -> {
        secondaryPort.configure(boot, config.countsPerRevolution, positionFactor, velocityFactor);
        boot.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        boot.signals
            .externalOrAltEncoderPosition(periodMs)
            .externalOrAltEncoderPositionAlwaysOn(true)
            .externalOrAltEncoderVelocity(periodMs)
            .externalOrAltEncoderVelocityAlwaysOn(true);
      }
      case ABSOLUTE_DATAPORT -> {
        // The zero offset is not written: the one saved in the controller is kept.
        boot.absoluteEncoder
            .positionConversionFactor(positionFactor)
            .velocityConversionFactor(velocityFactor);
        boot.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        boot.signals
            .absoluteEncoderPositionPeriodMs(periodMs)
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(periodMs)
            .absoluteEncoderVelocityAlwaysOn(true);
      }
      default -> {
        boot.encoder
            .positionConversionFactor(positionFactor)
            .velocityConversionFactor(velocityFactor);
        boot.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        boot.signals
            .primaryEncoderPositionPeriodMs(periodMs)
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(periodMs)
            .primaryEncoderVelocityAlwaysOn(true);
      }
    }

    for (int i = 0; i < SLOT_COUNT; i++) {
      ClosedLoopSlot slot = slotOf(i);
      boot.closedLoop
          .pid(config.kP[i], config.kI[i], config.kD[i], slot)
          .outputRange(config.minOutput, config.maxOutput, slot);
      writeFeedforward(boot, slot, config.kS[i], config.kV[i], config.kA[i], config.kG[i]);
      double cruise = config.maxMotionMaxVelocity[i].in(RotationsPerSecond);
      if (cruise > 0.0) {
        writeMotion(
            boot,
            i,
            cruise,
            config.maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond),
            config.maxMotionAllowedClosedLoopError[i].in(Rotations));
      }
    }

    if (config.softLimitEnabled) {
      boot.softLimit
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(config.maxPosition.in(Rotations))
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(config.minPosition.in(Rotations));
    }
    if (config.positionWrap) {
      boot.closedLoop
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(
              config.minPosition.in(Rotations), config.maxPosition.in(Rotations));
    }

    configureAtBoot(boot);
    motor.clearFaults();

    switch (sensorType) {
      case ALTERNATE -> {
        relativeEncoder = secondaryPort.open();
        absoluteEncoder = null;
      }
      case ABSOLUTE_DATAPORT -> {
        relativeEncoder = null;
        absoluteEncoder = motor.getAbsoluteEncoder();
      }
      default -> {
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = null;
      }
    }
    if (relativeEncoder != null) {
      relativeEncoder.setPosition(0.0);
    }
  }

  /**
   * The sensor that closes the loop. An encoder fused through the factory ({@code
   * withExternalEncoder(..., ALTERNATE | ABSOLUTE_DATAPORT)}) wins over {@code feedbackType}.
   */
  private static FeedbackSensorType resolveSensor(String name, MotorConfig config) {
    FeedbackSensorType type = config.feedbackType;
    if (config.externalFusionType == FeedbackSensorType.ALTERNATE
        || config.externalFusionType == FeedbackSensorType.ABSOLUTE_DATAPORT) {
      type = config.externalFusionType;
    }
    if (type == FeedbackSensorType.INTERNAL
        || type == FeedbackSensorType.ALTERNATE
        || type == FeedbackSensorType.ABSOLUTE_DATAPORT) {
      return type;
    }
    Report.warning(
        "Motor " + name + ": feedback " + type + " does not exist on a SPARK, using its encoder.");
    return FeedbackSensorType.INTERNAL;
  }

  private static int toSmartLimit(double amps) {
    return (int) Math.max(1, Math.min(Integer.MAX_VALUE, Math.round(Math.abs(amps))));
  }

  private static ClosedLoopSlot slotOf(int slot) {
    return switch (slot) {
      case 1 -> ClosedLoopSlot.kSlot1;
      case 2 -> ClosedLoopSlot.kSlot2;
      case 3 -> ClosedLoopSlot.kSlot3;
      default -> ClosedLoopSlot.kSlot0;
    };
  }

  private void writeFeedforward(
      SparkBaseConfig target, ClosedLoopSlot slot, double s, double v, double a, double g) {
    target.closedLoop.feedForward.kS(s, slot).kV(v, slot).kA(a, slot);
    if (gravityType == GravityType.ARM) {
      // kCosRatio 1: the positions are already rotations of the mechanism.
      target.closedLoop.feedForward.kG(0.0, slot).kCos(g, slot).kCosRatio(1.0, slot);
    } else {
      target.closedLoop.feedForward.kG(g, slot).kCos(0.0, slot);
    }
  }

  private void writeMotion(
      SparkBaseConfig target,
      int slot,
      double cruiseRps,
      double accelerationRpsPerSec,
      double allowedErrorRotations) {
    cruiseVelocityRps[slot] = Math.max(0.0, cruiseRps);
    ClosedLoopSlot revSlot = slotOf(slot);
    target
        .closedLoop
        .maxMotion
        .cruiseVelocity(cruiseRps * 60.0, revSlot)
        .maxAcceleration(accelerationRpsPerSec * 60.0, revSlot)
        .allowedProfileError(allowedErrorRotations, revSlot);
  }

  private void configureAtBoot(SparkBaseConfig config) {
    REVLibError status = null;
    for (int attempt = 0; attempt < CONFIG_ATTEMPTS; attempt++) {
      try {
        status =
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      } catch (IllegalStateException e) {
        Report.error("Motor " + name + ": the SPARK rejected the configuration: " + e.getMessage());
        return;
      }
      if (status == REVLibError.kOk) {
        return;
      }
    }
    Report.error(
        "Motor "
            + name
            + ": configuration failed after "
            + CONFIG_ATTEMPTS
            + " attempts ("
            + status
            + "). Check the CAN wiring and the id.");
  }

  /** Sends a change made while the robot runs: no blocking, not saved to flash. */
  private void configureRuntime(SparkBaseConfig delta) {
    motor.configureAsync(delta, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // ---------------------------------------------------------------------------
  // Inputs
  // ---------------------------------------------------------------------------

  /** Returns {@code value} and remembers whether the read that produced it failed. */
  private double checked(double value) {
    if (motor.getLastError() != REVLibError.kOk) {
      readsOk = false;
    }
    return value;
  }

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    readsOk = true;
    double output = checked(motor.getAppliedOutput());
    double busVolts = checked(motor.getBusVoltage());
    inputs.percentOutput = output;
    inputs.appliedVolts = Volts.of(output * busVolts);
    inputs.current = Amps.of(checked(motor.getOutputCurrent()));
    inputs.temperature = Celsius.of(checked(motor.getMotorTemperature()));
    inputs.activeFaults = readFaults();
    updateConnection(inputs);
  }

  @Override
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    updateHardwareInputs((BasicMotorIOInputs) inputs);
    double position;
    double velocityRpm;
    if (absoluteEncoder != null) {
      position = checked(absoluteEncoder.getPosition());
      velocityRpm = checked(absoluteEncoder.getVelocity());
    } else {
      position = checked(relativeEncoder.getPosition());
      velocityRpm = checked(relativeEncoder.getVelocity());
    }
    inputs.position = Rotations.of(position);
    inputs.velocity = RotationsPerSecond.of(velocityRpm / 60.0);
    updateConnection(inputs);
  }

  private void updateConnection(BasicMotorIOInputs inputs) {
    double now = RobotClock.now();
    if (readsOk) {
      lastAnswerTime = now;
    }
    inputs.isConnected = now - lastAnswerTime < DISCONNECT_TIMEOUT_SECONDS;
  }

  /** Decodes the faults only when they change, so a healthy motor costs nothing here. */
  private String[] readFaults() {
    if (!motor.hasActiveFault() && !motor.hasActiveWarning()) {
      lastFaultBits = 0;
      lastWarningBits = 0;
      faults = NO_FAULTS;
      return faults;
    }
    Faults activeFaults = motor.getFaults();
    Warnings activeWarnings = motor.getWarnings();
    if (activeFaults.rawBits != lastFaultBits || activeWarnings.rawBits != lastWarningBits) {
      lastFaultBits = activeFaults.rawBits;
      lastWarningBits = activeWarnings.rawBits;
      faults = RevMotorFaults.describe(activeFaults, activeWarnings);
    }
    return faults;
  }

  // ---------------------------------------------------------------------------
  // Hardware hooks
  // ---------------------------------------------------------------------------

  @Override
  protected void applyVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  protected void applyPercentOutput(double percent) {
    // Voltage compensation is on: 1.0 means the nominal voltage.
    motor.set(percent);
  }

  @Override
  protected void applyStop() {
    motor.stopMotor();
  }

  @Override
  protected void applyVelocity(double rotationsPerSecond, int slot) {
    closedLoop.setSetpoint(rotationsPerSecond * 60.0, ControlType.kVelocity, slotOf(slot));
  }

  @Override
  protected void applyPosition(double rotations, int slot) {
    closedLoop.setSetpoint(rotations, ControlType.kPosition, slotOf(slot));
  }

  @Override
  protected void applySmartPosition(double rotations, int slot) {
    if (cruiseVelocityRps[slot] <= 0.0) {
      // MAXMotion without a cruise velocity never moves: use plain position control instead.
      if (!missingProfileReported) {
        missingProfileReported = true;
        Report.warning(
            "Motor "
                + name
                + ": runSmartPosition without smartMotion(...) on slot "
                + slot
                + ", using position control.");
      }
      applyPosition(rotations, slot);
      return;
    }
    closedLoop.setSetpoint(rotations, ControlType.kMAXMotionPositionControl, slotOf(slot));
  }

  @Override
  protected void applyBrakeMode(boolean enabled) {
    SparkBaseConfig delta = configFactory.get();
    delta.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    configureRuntime(delta);
  }

  @Override
  protected void applyCurrentLimit(double amps) {
    SparkBaseConfig delta = configFactory.get();
    delta.smartCurrentLimit(toSmartLimit(amps));
    configureRuntime(delta);
  }

  @Override
  public void setOffset(Angle offset) {
    if (relativeEncoder != null) {
      relativeEncoder.setPosition(offset.in(Rotations));
    } else if (!absoluteOffsetReported) {
      absoluteOffsetReported = true;
      Report.warning(
          "Motor "
              + name
              + ": an absolute encoder keeps its zero in the controller (REV Hardware Client);"
              + " setOffset ignored.");
    }
  }

  // ---------------------------------------------------------------------------
  // Tuning (dashboard)
  // ---------------------------------------------------------------------------

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    SparkBaseConfig delta = configFactory.get();
    delta.closedLoop.pid(p, i, d, slotOf(slot));
    configureRuntime(delta);
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    SparkBaseConfig delta = configFactory.get();
    writeFeedforward(delta, slotOf(slot), s, v, a, g);
    configureRuntime(delta);
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    SparkBaseConfig delta = configFactory.get();
    writeMotion(delta, slot, maxVel, maxAccel, allowedErr);
    configureRuntime(delta);
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    SparkBaseConfig delta = configFactory.get();
    delta.closedLoop.outputRange(min, max, slotOf(slot));
    configureRuntime(delta);
  }

  // ---------------------------------------------------------------------------
  // Extra sensors
  // ---------------------------------------------------------------------------

  /**
   * A read-only {@link EncoderIO} view of the absolute encoder connected to the data port. The
   * controller object is shared, so no CAN id is duplicated.
   */
  public EncoderIO getAbsoluteEncoderIO() {
    return new EncoderIOSparkAbsolute(name + "_AbsEnc", new EncoderConfig(), motor);
  }
}
