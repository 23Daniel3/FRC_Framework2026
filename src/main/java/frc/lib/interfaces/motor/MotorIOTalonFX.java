package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.security.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

/**
 * MotorIO implementation for CTRE TalonFX (Kraken/Falcon). Optimized to run all control physics
 * (SVAG + PID + Motion Magic) natively on hardware.
 */
public class MotorIOTalonFX extends MotorBase {

  private final TalonFX motor;
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final MotorIOInputs inputs = new MotorIOInputs();

  // Control Requests (reused to avoid memory allocation in the loop)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  // Status Signals for telemetry (Phoenix 6 uses optimized signals)
  private final StatusSignal<Angle> posSignal;
  private final StatusSignal<AngularVelocity> velSignal;
  private final StatusSignal<Voltage> voltSignal;
  private final StatusSignal<Current> currentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  public MotorIOTalonFX(String name, int id, CANBus canBus, MotorConfig config) {
    super(name, config);

    this.motor = new TalonFX(id, canBus);

    // --- Output and Neutral Configuration ---
    driveConfig.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    driveConfig.MotorOutput.NeutralMode =
        (config.idleMode == com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

    // --- Current Limits ---
    driveConfig.CurrentLimits.StatorCurrentLimit = config.currentLimit.in(Amps);
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit =
        config.currentLimit.in(Amps); // Battery protection
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // --- Soft Limits ---
    if (config.softLimitEnabled) {
      driveConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      driveConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.maxPosition.in(Rotations);
      driveConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      driveConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.minPosition.in(Rotations);
    }

    // --- Configure Follower Motor
    if (config.leaderMotorID != 0) {
      motor.setControl(
          new Follower(
              config.leaderMotorID,
              config.followerInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }

    driveConfig.Voltage.PeakForwardVoltage = config.maxOutput * config.nominalVoltage.in(Volts);
    driveConfig.Voltage.PeakReverseVoltage = config.minOutput * config.nominalVoltage.in(Volts);

    // --- Gear Ratio (Sensor to Mechanism) ---
    // In Phoenix 6, the reduction is set here and it automatically scales Position and Velocity.
    driveConfig.Feedback.SensorToMechanismRatio =
        (config.positionConversionFactor != 0.0) ? 1.0 / config.positionConversionFactor : 1.0;

    // --- Control Slots (0 to 3) ---
    // Apply full native PID and SVAG for each slot defined in MotorConfig
    applySlotConfig(
        0,
        config.kP[0],
        config.kI[0],
        config.kD[0],
        config.kS[0],
        config.kV[0],
        config.kA[0],
        config.kG[0]);
    applySlotConfig(
        1,
        config.kP[1],
        config.kI[1],
        config.kD[1],
        config.kS[1],
        config.kV[1],
        config.kA[1],
        config.kG[1]);
    applySlotConfig(
        2,
        config.kP[2],
        config.kI[2],
        config.kD[2],
        config.kS[2],
        config.kV[2],
        config.kA[2],
        config.kG[2]);

    // --- Motion Magic (Native on Hardware) ---
    // Slot 0 is used as the default for Motion Magic
    driveConfig.MotionMagic.MotionMagicCruiseVelocity =
        config.maxMotionMaxVelocity[0].in(RotationsPerSecond);
    driveConfig.MotionMagic.MotionMagicAcceleration =
        config.maxMotionMaxAcceleration[0].in(RotationsPerSecondPerSecond);

    // Initial configuration application
    tryUntilOk(5, () -> motor.getConfigurator().apply(driveConfig));

    // --- Signal Configuration ---
    posSignal = motor.getPosition();
    velSignal = motor.getVelocity();
    voltSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    // Optimized frequencies for control and logging
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, posSignal, velSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(20.0, voltSignal, currentSignal, temperatureSignal);

    // Optimize CAN bus bandwidth utilization
    motor.optimizeBusUtilization();

    motor.setPosition(0);
  }

  @Override
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(posSignal, velSignal, voltSignal, currentSignal, temperatureSignal);

    inputs.position = posSignal.getValue();
    inputs.velocity = velSignal.getValue();
    inputs.appliedVolts = voltSignal.getValue();
    inputs.current = currentSignal.getValue();
    inputs.temperature = temperatureSignal.getValue();
    inputs.isConnected = BaseStatusSignal.isAllGood(posSignal, velSignal);

    if (!inputs.isConnected) {
      inputs.activeFaults = frc.lib.interfaces.motor.MotorFaults.getTalonFaults(motor);
    } else {
      inputs.activeFaults = new String[] {};
    }
  }

  // --- Native Control Implementation ---

  @Override
  public void runVelocity(AngularVelocity velocity) {
    this.currentMode = MotorControlMode.VELOCITY;
    this.targetVelocity = velocity;
    motor.setControl(velocityRequest.withVelocity(velocity).withSlot(0));
  }

  @Override
  public void runPosition(Angle position) {
    this.currentMode = MotorControlMode.POSITION;
    this.targetPosition = position;
    motor.setControl(positionRequest.withPosition(position).withSlot(0));
  }

  @Override
  public void runSmartPosition(Angle position) {
    this.currentMode = MotorControlMode.SMART_POSITION;
    this.targetPosition = position;
    // Motion Magic nativo
    motor.setControl(motionMagicRequest.withPosition(position).withSlot(0));
  }

  @Override
  public void runVelocity(AngularVelocity velocity, int slot) {
    this.currentMode = MotorControlMode.VELOCITY;
    this.targetVelocity = velocity;
    motor.setControl(velocityRequest.withVelocity(velocity).withSlot(slot));
  }

  @Override
  public void runPosition(Angle position, int slot) {
    this.currentMode = MotorControlMode.POSITION;
    this.targetPosition = position;
    motor.setControl(positionRequest.withPosition(position).withSlot(slot));
  }

  @Override
  public void runSmartPosition(Angle position, int slot) {
    this.currentMode = MotorControlMode.SMART_POSITION;
    this.targetPosition = position;
    motor.setControl(motionMagicRequest.withPosition(position).withSlot(slot));
  }

  @Override
  public void runVoltage(Voltage volts) {
    this.currentMode = MotorControlMode.IDLE;
    motor.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }

  @Override
  public void runPercentOutput(double percent) {
    this.currentMode = MotorControlMode.IDLE;
    motor.setControl(dutyCycleRequest.withOutput(percent));
  }

  @Override
  public void stop() {
    this.currentMode = MotorControlMode.IDLE;
    motor.stopMotor();
  }

  @Override
  public void setOffset(Angle offset) {
    motor.setPosition(offset);
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    // Phoenix 6 prefers updating MotionMagic via TalonFXConfiguration for greater stability
    driveConfig.MotionMagic.MotionMagicCruiseVelocity = maxVel;
    driveConfig.MotionMagic.MotionMagicAcceleration = maxAccel;
    motor.getConfigurator().apply(driveConfig.MotionMagic);
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    driveConfig.Voltage.PeakForwardVoltage = max * 12.0;
    driveConfig.Voltage.PeakReverseVoltage = min * 12.0;
    motor.getConfigurator().apply(driveConfig.Voltage);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    driveConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(driveConfig.MotorOutput);
  }

  @Override
  public void setCurrentLimit(Current current) {
    driveConfig.CurrentLimits.StatorCurrentLimit = current.in(Amps);
    motor.getConfigurator().apply(driveConfig.CurrentLimits);
  }

  @Override
  public MotorIOInputs getMotorIOInputs() {
    return inputs;
  }

  private void applySlotConfig(
      int slot, double p, double i, double d, double s, double v, double a, double g) {
    switch (slot) {
      case 0 -> {
        driveConfig.Slot0.kP = p;
        driveConfig.Slot0.kI = i;
        driveConfig.Slot0.kD = d;
        driveConfig.Slot0.kS = s;
        driveConfig.Slot0.kV = v;
        driveConfig.Slot0.kA = a;
        driveConfig.Slot0.kG = g;
      }
      case 1 -> {
        driveConfig.Slot1.kP = p;
        driveConfig.Slot1.kI = i;
        driveConfig.Slot1.kD = d;
        driveConfig.Slot1.kS = s;
        driveConfig.Slot1.kV = v;
        driveConfig.Slot1.kA = a;
        driveConfig.Slot1.kG = g;
      }
      case 2 -> {
        driveConfig.Slot2.kP = p;
        driveConfig.Slot2.kI = i;
        driveConfig.Slot2.kD = d;
        driveConfig.Slot2.kS = s;
        driveConfig.Slot2.kV = v;
        driveConfig.Slot2.kA = a;
        driveConfig.Slot2.kG = g;
      }
    }
  }

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    if (motor == null) return;
    applySlotUpdate(slot, p, i, d, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    if (motor == null) return;
    applySlotUpdate(slot, Double.NaN, Double.NaN, Double.NaN, s, v, a, g);
  }

  private void applySlotUpdate(
      int slot, double p, double i, double d, double s, double v, double a, double g) {
    switch (slot) {
      case 0 -> {
        Slot0Configs s0 = new Slot0Configs();
        motor.getConfigurator().refresh(s0);
        if (!Double.isNaN(p)) {
          s0.kP = p;
          s0.kI = i;
          s0.kD = d;
        }
        if (!Double.isNaN(s)) {
          s0.kS = s;
          s0.kV = v;
          s0.kA = a;
          s0.kG = g;
        }
        motor.getConfigurator().apply(s0);
        break;
      }
      case 1 -> {
        Slot1Configs s1 = new Slot1Configs();
        motor.getConfigurator().refresh(s1);
        if (!Double.isNaN(p)) {
          s1.kP = p;
          s1.kI = i;
          s1.kD = d;
        }
        if (!Double.isNaN(s)) {
          s1.kS = s;
          s1.kV = v;
          s1.kA = a;
          s1.kG = g;
        }
        motor.getConfigurator().apply(s1);
        break;
      }
      case 2 -> {
        Slot2Configs s2 = new Slot2Configs();
        motor.getConfigurator().refresh(s2);
        if (!Double.isNaN(p)) {
          s2.kP = p;
          s2.kI = i;
          s2.kD = d;
        }
        if (!Double.isNaN(s)) {
          s2.kS = s;
          s2.kV = v;
          s2.kA = a;
          s2.kG = g;
        }
        motor.getConfigurator().apply(s2);
        break;
      }
      default -> System.err.println("[MotorIOTalonFX] Slot " + slot + " not supported.");
    }
  }
}
