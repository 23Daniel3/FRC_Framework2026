package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.advanced.MotorBase;
import frc.lib.interfaces.motor.advanced.MotorConfig;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;
import frc.lib.util.security.SparkUtil;

public class MotorIOSparkFlex extends MotorBase {

  private final SparkFlex motor;
  private final SparkClosedLoopController closedLoopController;
  private final SparkFlexConfig motorConfig;
  private final MotorIOInputs inputs = new MotorIOInputs();

  // Sensores
  private RelativeEncoder internalEncoder;
  private RelativeEncoder externalEncoder;
  private AbsoluteEncoder absoluteEncoder;
  private final MotorConfig.FeedbackSensorType sensorType;

  public MotorIOSparkFlex(String name, int id, BasicMotorConfig config) {
    this(name, id, MotorType.kBrushless, MotorConfig.fromBasic(config));
  }

  public MotorIOSparkFlex(String name, int id, MotorConfig config) {
    this(name, id, MotorType.kBrushless, config);
  }

  public MotorIOSparkFlex(
      String name, int id, MotorType type, frc.lib.interfaces.motor.basic.BasicMotorConfig config) {
    this(name, id, type, MotorConfig.fromBasic(config));
  }

  public MotorIOSparkFlex(String name, int id, MotorType type, MotorConfig config) {
    // Initialize MotorBase (apply... methods called in super will return silently
    // since motor == null at that point)
    super(name, config);

    this.motor = new SparkFlex(id, type);
    this.closedLoopController = motor.getClosedLoopController();
    this.motorConfig = new SparkFlexConfig();
    this.sensorType = config.feedbackType;

    // --- Configure Follower Motor
    if (config.leaderMotorID != 0) {
      motorConfig.follow(config.leaderMotorID, config.followerInverted);
    }

    // --- Basic Configuration ---
    motorConfig
        .inverted(config.inverted)
        .smartCurrentLimit((int) config.currentLimit.in(Amps))
        .idleMode(config.brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
        .voltageCompensation(config.nominalVoltage.in(Volts))
        .closedLoop
        .outputRange(config.minOutput, config.maxOutput);

    // --- Encoder Selection ---
    switch (config.feedbackType) {
      case ABSOLUTE_DATAPORT -> {
        motorConfig
            .absoluteEncoder
            .positionConversionFactor(config.positionConversionFactor)
            .velocityConversionFactor(config.velocityConversionFactor)
            .zeroOffset(0.0);
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        absoluteEncoder = motor.getAbsoluteEncoder();
      }
      default -> {
        motorConfig
            .encoder
            .positionConversionFactor(config.positionConversionFactor)
            .velocityConversionFactor(config.velocityConversionFactor);
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        internalEncoder = motor.getEncoder();
      }
    }

    // --- Initial PID/SVAG/SmartMotion Injection (All Slots) ---
    for (int i = 0; i < 4; i++) {
      ClosedLoopSlot slot = resolveSlot(i);
      motorConfig
          .closedLoop
          .p(config.kP[i], slot)
          .i(config.kI[i], slot)
          .d(config.kD[i], slot)
          .outputRange(config.minOutput, config.maxOutput, slot)
          .apply(new FeedForwardConfig().kV(config.kV[i], slot));

      // MAXMotion Configuration
      if (config.maxMotionMaxVelocity[i].in(RadiansPerSecond) > 0) {
        double rpm = config.maxMotionMaxVelocity[i].in(RotationsPerSecond) * 60.0;
        double rpmPerSec =
            config.maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond) * 60.0;
        motorConfig
            .closedLoop
            .maxMotion
            .cruiseVelocity(rpm, slot)
            .maxAcceleration(rpmPerSec, slot)
            .allowedProfileError(config.maxMotionAllowedClosedLoopError[i].in(Rotations), slot);
      }
    }

    // --- Soft Limits e Wrapping ---
    if (config.softLimitEnabled) {
      motorConfig
          .softLimit
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(config.maxPosition.in(Rotations))
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(config.minPosition.in(Rotations));
    }

    if (config.positionWrap) {
      motorConfig
          .closedLoop
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(
              config.minPosition.in(Rotations), config.maxPosition.in(Rotations));
    }

    // --- Final push of configuration to Hardware ---
    applyConfig(true);

    if (internalEncoder != null) internalEncoder.setPosition(0);
    if (externalEncoder != null) externalEncoder.setPosition(0);
  }

  @Override
  protected void updateHardwareInputs(
      frc.lib.interfaces.motor.basic.BasicMotorIO.BasicMotorIOInputs inputs) {
    inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.current = Amps.of(motor.getOutputCurrent());
    inputs.temperature = Celsius.of(motor.getMotorTemperature());
    inputs.isConnected = !motor.hasActiveFault();
    inputs.activeFaults =
        motor.hasActiveFault()
            ? frc.lib.interfaces.motor.MotorFaults.getSparkFaults(motor)
            : new String[] {};
  }

  @Override
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    updateHardwareInputs((frc.lib.interfaces.motor.basic.BasicMotorIO.BasicMotorIOInputs) inputs);

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
  }

  // --- Controle de Movimento com Arbitrary FF Injetado ---

  @Override
  public void runVelocity(AngularVelocity velocity) {
    currentMode = MotorControlMode.VELOCITY;
    targetVelocity = velocity;

    double velocityRPM = velocity.in(RPM);

    closedLoopController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void runPosition(Angle position) {
    currentMode = MotorControlMode.POSITION;
    targetPosition = position;

    closedLoopController.setSetpoint(
        position.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void runSmartPosition(Angle position) {
    currentMode = MotorControlMode.SMART_POSITION;
    targetPosition = position;

    closedLoopController.setSetpoint(
        position.in(Rotations), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void runVelocity(AngularVelocity velocity, int slot) {
    currentMode = MotorControlMode.VELOCITY;
    targetVelocity = velocity;

    ClosedLoopSlot resolvedSlot = resolveSlot(slot);
    double velocityRPM = velocity.in(RPM);

    closedLoopController.setSetpoint(velocityRPM, ControlType.kVelocity, resolvedSlot);
  }

  @Override
  public void runPosition(Angle position, int slot) {
    currentMode = MotorControlMode.POSITION;
    targetPosition = position;

    ClosedLoopSlot resolvedSlot = resolveSlot(slot);

    closedLoopController.setSetpoint(position.in(Rotations), ControlType.kPosition, resolvedSlot);
  }

  @Override
  public void runSmartPosition(Angle position, int slot) {
    currentMode = MotorControlMode.SMART_POSITION;
    targetPosition = position;

    ClosedLoopSlot resolvedSlot = resolveSlot(slot);

    closedLoopController.setSetpoint(
        position.in(Rotations), ControlType.kMAXMotionPositionControl, resolvedSlot);
  }

  // --- Low-Level Controls ---

  @Override
  public void runVoltage(Voltage volts) {
    currentMode = MotorControlMode.VOLTAGE;
    motor.setVoltage(mapVoltage(volts.in(Volts)));
  }

  @Override
  public void runPercentOutput(double percent) {
    currentMode = MotorControlMode.PERCENT;
    motor.set(mapOutput(percent));
  }

  @Override
  public void stop() {
    currentMode = MotorControlMode.IDLE;
    motor.stopMotor();
  }

  @Override
  public void setOffset(Angle offset) {
    if (internalEncoder != null) internalEncoder.setPosition(offset.in(Rotations));
    if (externalEncoder != null) externalEncoder.setPosition(offset.in(Rotations));
  }

  // --- Hardware Update via Dashboard (Tuning) ---

  @Override
  public void setBrakeMode(boolean enabled) {
    if (motor == null) return; // Previne NullPointer no Super
    motorConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    applyConfig(false);
  }

  @Override
  public void setCurrentLimit(Current current) {
    if (motor == null) return;
    motorConfig.smartCurrentLimit((int) current.in(Amps));
    applyConfig(false);
  }

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    if (motor == null) return;
    motorConfig.closedLoop.p(p, resolveSlot(slot)).i(i, resolveSlot(slot)).d(d, resolveSlot(slot));
    applyConfig(false);
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    if (motor == null) return;
    ClosedLoopSlot resolvedSlot = resolveSlot(slot);

    motorConfig.closedLoop.apply(
        new FeedForwardConfig()
            .kS(s, resolvedSlot)
            .kV(v, resolvedSlot)
            .kA(a, resolvedSlot)
            .kG(g, resolvedSlot));

    applyConfig(false);
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    if (motor == null) return;
    ClosedLoopSlot revSlot = resolveSlot(slot);
    motorConfig
        .closedLoop
        .maxMotion
        .cruiseVelocity(maxVel * 60.0, revSlot)
        .maxAcceleration(maxAccel * 60.0, revSlot)
        .allowedProfileError(allowedErr, revSlot);
    applyConfig(false);
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    if (motor == null) return;
    motorConfig.closedLoop.outputRange(min, max, resolveSlot(slot));
    applyConfig(false);
  }

  @Override
  public MotorIOInputs getMotorIOInputs() {
    return inputs;
  }

  // --- Utilidades ---

  private void applyConfig(boolean isInit) {
    ResetMode resetMode =
        isInit ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    SparkUtil.tryUntilOk(
        motor, 5, () -> motor.configure(motorConfig, resetMode, PersistMode.kPersistParameters));
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
