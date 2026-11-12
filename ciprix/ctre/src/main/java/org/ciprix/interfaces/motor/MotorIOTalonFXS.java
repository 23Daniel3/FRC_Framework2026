package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.ciprix.CiprixSettings;
import org.ciprix.interfaces.motor.advanced.MotorBase;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorConfig.FeedbackSensorType;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;

/**
 * Talon FXS (Minion, NEO, NEO 550, Vortex or brushed motors through the JST/brushed ports). Same
 * behavior and units as {@link MotorIOTalonFX}; this class only differs where the hardware does.
 *
 * <p>The Talon FXS must know which motor it drives (the motor arrangement). Pass it to the
 * constructor ({@code MotorArrangementValue.Minion_JST}, for example). The constructors without it
 * keep the arrangement saved in the controller (set with Phoenix Tuner X) and report an error when
 * there is none, because a Talon FXS without an arrangement does not drive the motor.
 *
 * <p>Feedback: the motor sensor (commutation), a CANcoder ({@code withRemoteCANcoder}), the
 * quadrature encoder on the data port ({@code withAlternateEncoder(cpr)}, cpr in edges per
 * rotation) or the pulse-width absolute encoder on the data port ({@code withAbsoluteEncoder()}).
 */
public class MotorIOTalonFXS extends MotorBase {

  /** A controller that does not answer for this long counts as disconnected. */
  public static final double DISCONNECT_TIMEOUT_SECONDS = 0.5;

  private final TalonFXS motor;
  private final TalonFXSConfiguration talonConfig = new TalonFXSConfiguration();
  private final boolean explicitSupplyLimit;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> deviceTemp;
  private final BaseStatusSignal dutyCycle;
  private final CtreFaultReader faultReader;
  private final BaseStatusSignal[] cycleSignals;

  private final double[] cruiseRps = new double[SLOT_COUNT];
  private final double[] accelerationRps2 = new double[SLOT_COUNT];
  private int appliedMotionSlot = 0;
  private double appliedMinOutput;
  private double appliedMaxOutput;

  private double lastAnswerTime = Double.NEGATIVE_INFINITY;
  private boolean slotReported = false;
  private boolean missingProfileReported = false;

  public MotorIOTalonFXS(String name, int id, CANBus canBus, BasicMotorConfig config) {
    this(name, id, canBus, null, MotorConfig.fromBasic(config));
  }

  public MotorIOTalonFXS(String name, int id, CANBus canBus, MotorConfig config) {
    this(name, id, canBus, null, config);
  }

  /**
   * @param arrangement the motor connected to the Talon FXS, or null to keep the one saved in the
   *     controller
   */
  public MotorIOTalonFXS(
      String name, int id, CANBus canBus, MotorArrangementValue arrangement, MotorConfig config) {
    super(name, config);
    this.motor = new TalonFXS(id, canBus);
    this.explicitSupplyLimit = config.supplyCurrentLimit != null;
    this.appliedMinOutput = config.minOutput;
    this.appliedMaxOutput = config.maxOutput;

    if (arrangement != null) {
      talonConfig.Commutation.MotorArrangement = arrangement;
    } else {
      // Keep what Phoenix Tuner X saved (the full configuration below would overwrite it).
      TalonFXSConfiguration saved = new TalonFXSConfiguration();
      motor.getConfigurator().refresh(saved, TalonConfigs.BOOT_TIMEOUT_SECONDS);
      talonConfig.Commutation = saved.Commutation;
      if (talonConfig.Commutation.MotorArrangement == MotorArrangementValue.Disabled) {
        Report.error(
            "Motor "
                + name
                + ": the Talon FXS has no motor arrangement and will not drive the motor. Pass one"
                + " to the constructor, for example MotorArrangementValue.Minion_JST.");
      }
    }

    TalonConfigs.writeOutput(talonConfig.MotorOutput, config.inverted, config.brakeMode);
    TalonConfigs.writeCurrentLimits(
        talonConfig.CurrentLimits,
        Math.abs(config.currentLimit.in(Amps)),
        TalonConfigs.supplyLimitAmps(config));
    TalonConfigs.writePeaks(talonConfig.Voltage, config.minOutput, config.maxOutput, nominalVolts);
    TalonConfigs.writeRamps(
        talonConfig.OpenLoopRamps,
        talonConfig.ClosedLoopRamps,
        config.openLoopRampSeconds,
        config.closedLoopRampSeconds);
    TalonConfigs.writeSoftLimits(talonConfig.SoftwareLimitSwitch, config);
    TalonConfigs.writeWrap(talonConfig.ClosedLoopGeneral, config, name);

    FeedbackSensorType sensor = TalonConfigs.resolveSensor(config, name, true);
    talonConfig.ExternalFeedback.SensorToMechanismRatio =
        TalonConfigs.sensorToMechanismRatio(config);
    talonConfig.ExternalFeedback.RotorToSensorRatio = config.rotorToSensorRatio;
    talonConfig.ExternalFeedback.ExternalFeedbackSensorSource =
        switch (sensor) {
          case REMOTE_CANCODER -> ExternalFeedbackSensorSourceValue.RemoteCANcoder;
          case FUSED_CANCODER -> ExternalFeedbackSensorSourceValue.FusedCANcoder;
          case SYNC_CANCODER -> ExternalFeedbackSensorSourceValue.SyncCANcoder;
          case ALTERNATE -> ExternalFeedbackSensorSourceValue.Quadrature;
          case ABSOLUTE_DATAPORT -> ExternalFeedbackSensorSourceValue.PulseWidth;
          default -> ExternalFeedbackSensorSourceValue.Commutation;
        };
    if (TalonConfigs.isCANcoder(sensor)) {
      talonConfig.ExternalFeedback.FeedbackRemoteSensorID = TalonConfigs.remoteSensorId(config);
    }
    if (sensor == FeedbackSensorType.ALTERNATE) {
      talonConfig.ExternalFeedback.QuadratureEdgesPerRotation = config.countsPerRevolution;
    }

    TalonConfigs.writeGains(talonConfig.Slot0, config, 0);
    TalonConfigs.writeGains(talonConfig.Slot1, config, 1);
    TalonConfigs.writeGains(talonConfig.Slot2, config, 2);
    for (int i = 0; i < SLOT_COUNT; i++) {
      cruiseRps[i] = TalonConfigs.cruise(config, i);
      accelerationRps2[i] = TalonConfigs.acceleration(config, i);
    }
    TalonConfigs.writeMotion(talonConfig.MotionMagic, cruiseRps[0], accelerationRps2[0]);
    if (TalonConfigs.usesSlot3(config)) {
      Report.warning("Motor " + name + ": a Talon has slots 0 to 2; slot 3 is replaced by slot 0.");
    }

    TalonConfigs.applyAtBoot(
        name, () -> motor.getConfigurator().apply(talonConfig, TalonConfigs.BOOT_TIMEOUT_SECONDS));

    if (config.leaderMotorID != 0) {
      motor.setControl(
          new Follower(
              config.leaderMotorID,
              config.followerInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }

    position = motor.getPosition();
    velocity = motor.getVelocity();
    motorVoltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    deviceTemp = motor.getDeviceTemp();
    dutyCycle = motor.getDutyCycle();
    faultReader = CtreMotorFaults.readerFor(motor);
    cycleSignals =
        new BaseStatusSignal[] {
          position,
          velocity,
          motorVoltage,
          statorCurrent,
          deviceTemp,
          dutyCycle,
          faultReader.field()
        };

    double loopHz = 1.0 / CiprixSettings.getLoopPeriodSeconds();
    BaseStatusSignal.setUpdateFrequencyForAll(Math.max(100.0, loopHz), position, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(loopHz, motorVoltage, statorCurrent, dutyCycle);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, deviceTemp);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, faultReader.allSignals());
    motor.optimizeBusUtilization();

    if (sensor == FeedbackSensorType.INTERNAL || sensor == FeedbackSensorType.ALTERNATE) {
      // Absolute sensors keep their position; relative ones start at zero.
      TalonConfigs.applyAtBoot(
          name, () -> motor.setPosition(0.0, TalonConfigs.BOOT_TIMEOUT_SECONDS));
    }
  }

  // ---------------------------------------------------------------------------
  // Inputs
  // ---------------------------------------------------------------------------

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    StatusCode status = BaseStatusSignal.refreshAll(cycleSignals);
    double now = RobotClock.now();
    if (status.isOK()) {
      lastAnswerTime = now;
    }
    inputs.isConnected = now - lastAnswerTime < DISCONNECT_TIMEOUT_SECONDS;
    inputs.percentOutput = dutyCycle.getValueAsDouble();
    inputs.appliedVolts = motorVoltage.getValue();
    inputs.current = statorCurrent.getValue();
    inputs.temperature = deviceTemp.getValue();
    inputs.activeFaults = faultReader.read();
  }

  @Override
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    updateHardwareInputs((BasicMotorIOInputs) inputs);
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
  }

  // ---------------------------------------------------------------------------
  // Hardware hooks
  // ---------------------------------------------------------------------------

  private int talonSlot(int slot) {
    if (slot < TalonConfigs.TALON_SLOTS) {
      return slot;
    }
    if (!slotReported) {
      slotReported = true;
      Report.warning("Motor " + name + ": slot " + slot + " does not exist on a Talon, using 0.");
    }
    return 0;
  }

  @Override
  protected void applyVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  protected void applyPercentOutput(double percent) {
    motor.setControl(voltageRequest.withOutput(percent * nominalVolts));
  }

  @Override
  protected void applyStop() {
    motor.stopMotor();
  }

  @Override
  protected void applyVelocity(double rotationsPerSecond, int slot) {
    motor.setControl(velocityRequest.withVelocity(rotationsPerSecond).withSlot(talonSlot(slot)));
  }

  @Override
  protected void applyPosition(double rotations, int slot) {
    motor.setControl(positionRequest.withPosition(rotations).withSlot(talonSlot(slot)));
  }

  @Override
  protected void applySmartPosition(double rotations, int slot) {
    int talonSlot = talonSlot(slot);
    if (cruiseRps[talonSlot] <= 0.0) {
      if (!missingProfileReported) {
        missingProfileReported = true;
        Report.warning(
            "Motor "
                + name
                + ": runSmartPosition without smartMotion(...) on slot "
                + talonSlot
                + ", using position control.");
      }
      applyPosition(rotations, talonSlot);
      return;
    }
    if (talonSlot != appliedMotionSlot) {
      appliedMotionSlot = talonSlot;
      writeMotionLimits();
    }
    motor.setControl(motionMagicRequest.withPosition(rotations).withSlot(talonSlot));
  }

  private void writeMotionLimits() {
    TalonConfigs.writeMotion(
        talonConfig.MotionMagic, cruiseRps[appliedMotionSlot], accelerationRps2[appliedMotionSlot]);
    motor.getConfigurator().apply(talonConfig.MotionMagic, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
  }

  @Override
  protected void applyBrakeMode(boolean enabled) {
    talonConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(talonConfig.MotorOutput, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
  }

  @Override
  protected void applyCurrentLimit(double amps) {
    talonConfig.CurrentLimits.StatorCurrentLimit = amps;
    if (!explicitSupplyLimit) {
      talonConfig.CurrentLimits.SupplyCurrentLimit = amps;
    }
    motor.getConfigurator().apply(talonConfig.CurrentLimits, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
  }

  @Override
  public void setOffset(Angle offset) {
    motor.setPosition(offset.in(Rotations));
  }

  // ---------------------------------------------------------------------------
  // Tuning (dashboard)
  // ---------------------------------------------------------------------------

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    if (slot >= TalonConfigs.TALON_SLOTS) {
      talonSlot(slot);
      return;
    }
    switch (slot) {
      case 0 -> {
        talonConfig.Slot0.kP = p;
        talonConfig.Slot0.kI = i;
        talonConfig.Slot0.kD = d;
        motor.getConfigurator().apply(talonConfig.Slot0, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
      }
      case 1 -> {
        talonConfig.Slot1.kP = p;
        talonConfig.Slot1.kI = i;
        talonConfig.Slot1.kD = d;
        motor.getConfigurator().apply(talonConfig.Slot1, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
      }
      default -> {
        talonConfig.Slot2.kP = p;
        talonConfig.Slot2.kI = i;
        talonConfig.Slot2.kD = d;
        motor.getConfigurator().apply(talonConfig.Slot2, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
      }
    }
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    if (slot >= TalonConfigs.TALON_SLOTS) {
      talonSlot(slot);
      return;
    }
    switch (slot) {
      case 0 -> {
        talonConfig.Slot0.kS = s;
        talonConfig.Slot0.kV = v;
        talonConfig.Slot0.kA = a;
        talonConfig.Slot0.kG = g;
        motor.getConfigurator().apply(talonConfig.Slot0, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
      }
      case 1 -> {
        talonConfig.Slot1.kS = s;
        talonConfig.Slot1.kV = v;
        talonConfig.Slot1.kA = a;
        talonConfig.Slot1.kG = g;
        motor.getConfigurator().apply(talonConfig.Slot1, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
      }
      default -> {
        talonConfig.Slot2.kS = s;
        talonConfig.Slot2.kV = v;
        talonConfig.Slot2.kA = a;
        talonConfig.Slot2.kG = g;
        motor.getConfigurator().apply(talonConfig.Slot2, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
      }
    }
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    int talonSlot = talonSlot(slot);
    cruiseRps[talonSlot] = maxVel;
    accelerationRps2[talonSlot] = maxAccel;
    if (talonSlot == appliedMotionSlot) {
      writeMotionLimits();
    }
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    if (min == appliedMinOutput && max == appliedMaxOutput) {
      return;
    }
    appliedMinOutput = min;
    appliedMaxOutput = max;
    TalonConfigs.writePeaks(talonConfig.Voltage, min, max, nominalVolts);
    motor.getConfigurator().apply(talonConfig.Voltage, TalonConfigs.RUNTIME_TIMEOUT_SECONDS);
  }

  /** The Phoenix 6 object, for features this class does not cover. */
  public TalonFXS getTalon() {
    return motor;
  }
}
