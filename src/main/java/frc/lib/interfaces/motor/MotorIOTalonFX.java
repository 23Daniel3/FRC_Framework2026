package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.lib.util.CANType;
import frc.robot.generated.TunerConstants;

/**
 * Implementation of {@link MotorIO} for CTRE TalonFX (Kraken/Falcon) controllers.
 *
 * <p>Utilizes Phoenix 6 API. Handles signal caching, status updates, unit conversions, and advanced
 * control modes like Motion Magic and PID Slots.
 */
public class MotorIOTalonFX implements MotorIO {

  protected final TalonFX motor;
  protected final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

  // Requests
  protected final VoltageOut voltageRequest = new VoltageOut(0);
  protected final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  protected final PositionVoltage positionRequest = new PositionVoltage(0);
  protected final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  protected final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  // Status Signals
  protected final StatusSignal<Angle> posSignal;
  protected final StatusSignal<AngularVelocity> velSignal;
  protected final StatusSignal<Voltage> voltSignal;
  protected final StatusSignal<Current> currentSignal;
  protected final StatusSignal<Temperature> temperatureSignal;

  private final CANBus can;

  /**
   * Constructs a new MotorIOTalonFX.
   *
   * @param id The CAN ID of the TalonFX.
   * @param canBus The name of the CAN Bus (e.g., "rio", "canivore").
   * @param config The configuration object containing limits and modes.
   */
  public MotorIOTalonFX(int id, CANType canBus, MotorConfig config) {
    if (canBus == CANType.CANIVORE) {
      can = TunerConstants.kCANBus;
    } else {
      can = new CANBus();
    }
    motor = new TalonFX(id, can);

    // --- Basic Config ---
    driveConfig.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode =
        config.idleMode == com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

    // --- Current Limits ---
    driveConfig.CurrentLimits.SupplyCurrentLimit = config.currentLimit.in(Amps);
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // --- Soft Limits ---
    if (config.softLimitEnabled) {
      driveConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      driveConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.maxPosition.in(Rotations);
      driveConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      driveConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.minPosition.in(Rotations);
    }

    // --- Unit Conversion ---
    if (config.positionConversionFactor != 0.0) {
      driveConfig.Feedback.SensorToMechanismRatio = 1.0 / config.positionConversionFactor;
    } else {
      driveConfig.Feedback.SensorToMechanismRatio = 1.0;
    }

    // --- PID Slot 0 Defaults ---
    driveConfig.Slot0.kP = config.kP;
    driveConfig.Slot0.kI = config.kI;
    driveConfig.Slot0.kD = config.kD;
    driveConfig.Slot0.kV = config.kF;

    if (config.leaderMotorID != 0) {
      motor.setControl(new Follower(config.leaderMotorID, config.motorAlignment));
    }

    // --- Apply Configuration (Já estava no seu código, mas mantendo a consistência) ---
    tryUntilOk(5, () -> motor.getConfigurator().apply(driveConfig, 0.25));

    // --- Signals Setup ---
    posSignal = motor.getPosition();
    velSignal = motor.getVelocity();
    voltSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, posSignal, velSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, voltSignal, currentSignal, temperatureSignal);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(posSignal, velSignal, voltSignal, currentSignal, temperatureSignal);

    inputs.position = posSignal.getValue();
    inputs.velocity = velSignal.getValue();
    inputs.appliedVolts = voltSignal.getValue();
    inputs.current = currentSignal.getValue();
    inputs.temperature = temperatureSignal.getValue();
    inputs.isConnected = BaseStatusSignal.isAllGood(posSignal, velSignal);

    if (!inputs.isConnected) {
      inputs.activeFaults = MotorFaults.getTalonFaults(motor);
    } else {
      inputs.activeFaults = new String[] {};
    }
  }

  @Override
  public void setOffset(Angle offset) {
    motor.setPosition(offset);
  }

  // --- Basic Control ---

  @Override
  public void setVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  // --- Advanced Closed Loop Control ---

  @Override
  public void runVelocity(AngularVelocity velocity, int slotId, Voltage arbFF) {
    motor.setControl(
        velocityRequest.withVelocity(velocity).withSlot(slotId).withFeedForward(arbFF));
  }

  @Override
  public void runPosition(Angle position, int slotId, Voltage arbFF) {
    motor.setControl(
        positionRequest.withPosition(position).withSlot(slotId).withFeedForward(arbFF));
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    motor.setControl(dutyCycleRequest.withOutput(percentOutput));
  }

  // --- Configuration ---

  @Override
  public void configurePIDF(int slotId, double kP, double kI, double kD, double kF) {
    switch (slotId) {
      case 0 -> {
        Slot0Configs config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kF;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config));
      }
      case 1 -> {
        Slot1Configs config = new Slot1Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kF;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config));
      }
      case 2 -> {
        Slot2Configs config = new Slot2Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kF;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config));
      }
      default -> System.err.println("MotorIOTalonFX: Slot ID " + slotId + " not supported.");
    }
  }

  @Override
  public void configureKSVA(int slotId, double kS, double kV, double kA) {
    switch (slotId) {
      case 0 -> {
        Slot0Configs config = new Slot0Configs();
        config.kS = kS;
        config.kV = kV;
        config.kA = kA;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config));
      }

      case 1 -> {
        Slot1Configs config = new Slot1Configs();
        config.kS = kS;
        config.kV = kV;
        config.kA = kA;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config));
      }
      case 2 -> {
        Slot2Configs config = new Slot2Configs();
        config.kS = kS;
        config.kV = kV;
        config.kA = kA;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config));
      }
      default -> System.err.println("MotorIOTalonFX: Slot ID " + slotId + " not supported.");
    }
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config.MotorOutput));
  }

  @Override
  public void setCurrentLimit(Current current) {
    driveConfig.CurrentLimits.SupplyCurrentLimit = current.in(Amps);
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(driveConfig.CurrentLimits);
  }
}
