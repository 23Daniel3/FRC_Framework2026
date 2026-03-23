package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex; // Importante: Classe SparkFlex
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig; // Importante: Config específica do Flex
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.security.SparkUtil;

/**
 * Implementation of {@link MotorIO} for REV Robotics Spark Flex controllers. Adapted for the
 * specific differences in the Spark Flex API (External vs Alternate encoders).
 */
public class MotorIOSparkFlex implements MotorIO {

  private final SparkFlex motor;
  private final SparkClosedLoopController closedLoopController;
  private final SparkFlexConfig motorConfig = new SparkFlexConfig(); // Config do Flex

  // Sensors
  private RelativeEncoder internalEncoder;
  private RelativeEncoder externalEncoder; // Flex usa "External", não "Alternate"
  private AbsoluteEncoder absoluteEncoder;

  private final MotorConfig.FeedbackSensorType sensorType;

  public MotorIOSparkFlex(int id, MotorType type, MotorConfig config) {
    motor = new SparkFlex(id, type);
    closedLoopController = motor.getClosedLoopController();
    sensorType = config.feedbackType;

    // --- Basic Config ---
    motorConfig
        .inverted(config.inverted)
        .smartCurrentLimit((int) config.currentLimit.in(Amps))
        .idleMode(config.idleMode)
        .voltageCompensation(config.nominalVoltage.in(Volts));

    // --- Encoder & Feedback Selection ---
    switch (config.feedbackType) {
      case ALTERNATE -> {
        motorConfig
            .externalEncoder
            .countsPerRevolution(config.countsPerRevolution)
            .positionConversionFactor(config.positionConversionFactor)
            .velocityConversionFactor(config.velocityConversionFactor);

        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        externalEncoder = motor.getExternalEncoder();
      }
      case ABSOLUTE_DATAPORT -> {
        // Configure Absolute Encoder (Duty Cycle on Data Port)
        motorConfig
            .absoluteEncoder
            .positionConversionFactor(config.positionConversionFactor)
            .velocityConversionFactor(config.velocityConversionFactor)
            .zeroOffset(0.0);

        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        absoluteEncoder = motor.getAbsoluteEncoder();
      }
      default -> {
        // Internal Encoder (Geralmente do NEO Vortex)
        motorConfig
            .encoder
            .positionConversionFactor(config.positionConversionFactor)
            .velocityConversionFactor(config.velocityConversionFactor);

        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        internalEncoder = motor.getEncoder();
      }
    }

    // --- PID & SmartMotion (Slot 0) ---
    motorConfig
        .closedLoop
        .p(config.kP, ClosedLoopSlot.kSlot0)
        .i(config.kI, ClosedLoopSlot.kSlot0)
        .d(config.kD, ClosedLoopSlot.kSlot0)
        .apply(new FeedForwardConfig().kV(config.kF, ClosedLoopSlot.kSlot0))
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

    if (config.maxMotionMaxVelocity.in(RadiansPerSecond) > 0) {
      motorConfig
          .closedLoop
          .maxMotion
          .cruiseVelocity(
              config.maxMotionMaxVelocity.in(RotationsPerSecond) * 60, ClosedLoopSlot.kSlot0)
          .maxAcceleration(
              config.maxMotionMaxAcceleration.in(RotationsPerSecondPerSecond) * 60,
              ClosedLoopSlot.kSlot0)
          .allowedProfileError(
              config.maxMotionAllowedClosedLoopError.in(Rotations), ClosedLoopSlot.kSlot0);
    }

    // --- Soft Limits ---
    if (config.softLimitEnabled) {
      motorConfig
          .softLimit
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(config.maxPosition.in(Rotations))
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(config.minPosition.in(Rotations));
    }

    // --- Wrapping ---
    if (config.positionWrap) {
      motorConfig
          .closedLoop
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(
              config.minPosition.in(Rotations), config.maxPosition.in(Rotations));
    }

    // APLICANDO TryUntilOk NA CONFIGURAÇÃO INICIAL
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    // Read from the configured active sensor
    switch (sensorType) {
      case ALTERNATE -> {
        if (externalEncoder != null) {
          inputs.position = Rotations.of(externalEncoder.getPosition());
          inputs.velocity =
              RadiansPerSecond.of(
                  Units.rotationsPerMinuteToRadiansPerSecond(externalEncoder.getVelocity()));
        }
      }
      case ABSOLUTE_DATAPORT -> {
        if (absoluteEncoder != null) {
          inputs.position = Rotations.of(absoluteEncoder.getPosition());
          inputs.velocity =
              RadiansPerSecond.of(
                  Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity()));
        }
      }
      default -> {
        if (internalEncoder != null) {
          inputs.position = Rotations.of(internalEncoder.getPosition());
          inputs.velocity =
              RadiansPerSecond.of(
                  Units.rotationsPerMinuteToRadiansPerSecond(internalEncoder.getVelocity()));
        }
      }
    }

    inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.current = Amps.of(motor.getOutputCurrent());
    inputs.temperature = Celsius.of(motor.getMotorTemperature());
    inputs.isConnected = !motor.hasActiveFault();

    if (motor.hasActiveFault()) {
      inputs.activeFaults = MotorFaults.getSparkFaults(motor);
    } else {
      inputs.activeFaults = new String[] {};
    }
  }

  @Override
  public void setOffset(Angle offset) {
    motor.getEncoder().setPosition(offset.in(Rotations));
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    motor.set(percentOutput);
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runVelocity(AngularVelocity velocity, int slotId, Voltage arbFF) {
    double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocity.in(RadiansPerSecond));
    closedLoopController.setSetpoint(
        velocityRPM,
        ControlType.kVelocity,
        resolveSlot(slotId),
        arbFF.in(Volts),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void runPosition(Angle position, int slotId, Voltage arbFF) {
    closedLoopController.setSetpoint(
        position.in(Rotations),
        ControlType.kPosition,
        resolveSlot(slotId),
        arbFF.in(Volts),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void runSmartPosition(Angle position, int slotId, Voltage arbFF) {
    closedLoopController.setSetpoint(
        position.in(Rotations),
        ControlType.kMAXMotionPositionControl,
        resolveSlot(slotId),
        arbFF.in(Volts),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motorConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void configurePIDF(int slotId, double kP, double kI, double kD, double kF) {
    ClosedLoopSlot slot = resolveSlot(slotId);

    motorConfig
        .closedLoop
        .p(kP, slot)
        .i(kI, slot)
        .d(kD, slot)
        .apply(new FeedForwardConfig().kV(kF, slot));

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setMaxOutputSlot(double maxOutput, int slotId) {
    ClosedLoopSlot slot = resolveSlot(slotId);

    motorConfig.closedLoop.maxOutput(maxOutput, slot);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setCurrentLimit(Current current) {
    motorConfig.smartCurrentLimit((int) current.in(Amps));
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltageCompensation(Voltage voltage) {
    motorConfig.voltageCompensation(voltage.in(Volts));
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void configureSmartMotion(
      int slotId, AngularVelocity maxVel, AngularAcceleration maxAccel, Angle allowedError) {
    ClosedLoopSlot slot = resolveSlot(slotId);
    double rpm = Units.radiansPerSecondToRotationsPerMinute(maxVel.in(RadiansPerSecond));
    double rotPerSec = maxAccel.in(RotationsPerSecondPerSecond);
    double rpmPerSecond = rotPerSec * 60.0;

    motorConfig
        .closedLoop
        .maxMotion
        .cruiseVelocity(rpm, slot)
        .maxAcceleration(rpmPerSecond, slot)
        .allowedProfileError(allowedError.abs(Rotations), slot);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  private ClosedLoopSlot resolveSlot(int slotId) {
    return switch (slotId) {
      case 1 -> ClosedLoopSlot.kSlot1;
      case 2 -> ClosedLoopSlot.kSlot2;
      case 3 -> ClosedLoopSlot.kSlot3;
      default -> ClosedLoopSlot.kSlot0;
    };
  }
}
