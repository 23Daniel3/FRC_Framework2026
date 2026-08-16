package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.advanced.MotorBase;
import frc.lib.interfaces.motor.advanced.MotorConfig;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** MotorIO implementation for CTRE Talon SRX (Phoenix 5). */
public class MotorIOTalonSRX extends MotorBase {

  private final TalonSRX motor;
  private final MotorConfig motorConfig;
  private final MotorIOInputs inputs = new MotorIOInputs();

  public MotorIOTalonSRX(String name, int id, BasicMotorConfig config) {
    this(name, id, MotorConfig.fromBasic(config));
  }

  public MotorIOTalonSRX(String name, int id, MotorConfig config) {
    super(name, config);
    this.motor = new TalonSRX(id);
    this.motorConfig = config;

    TalonSRXConfiguration srxConfig = new TalonSRXConfiguration();

    // Limits
    srxConfig.peakOutputForward = config.maxOutput;
    srxConfig.peakOutputReverse = config.minOutput;

    // Soft Limits
    if (config.softLimitEnabled) {
      srxConfig.forwardSoftLimitEnable = true;
      srxConfig.forwardSoftLimitThreshold =
          config.maxPosition.in(edu.wpi.first.units.Units.Rotations) * config.countsPerRevolution;
      srxConfig.reverseSoftLimitEnable = true;
      srxConfig.reverseSoftLimitThreshold =
          config.minPosition.in(edu.wpi.first.units.Units.Rotations) * config.countsPerRevolution;
    }

    // Motion Magic
    srxConfig.motionCruiseVelocity =
        config.maxMotionMaxVelocity[0].in(edu.wpi.first.units.Units.RotationsPerSecond)
            * config.countsPerRevolution
            / 10.0;
    srxConfig.motionAcceleration =
        config.maxMotionMaxAcceleration[0].in(edu.wpi.first.units.Units.RotationsPerSecondPerSecond)
            * config.countsPerRevolution
            / 10.0;

    motor.configAllSettings(srxConfig);
    motor.setInverted(config.inverted);
    motor.setNeutralMode(config.brakeMode ? NeutralMode.Brake : NeutralMode.Coast);

    // Current Limit
    motor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, config.currentLimit.in(Amps), config.currentLimit.in(Amps), 0));

    // Follower
    if (config.leaderMotorID != 0) {
      motor.set(TalonSRXControlMode.Follower, config.leaderMotorID);
      motor.setInverted(
          config.followerInverted ? InvertType.OpposeMaster : InvertType.FollowMaster);
    }
  }

  @Override
  protected void updateHardwareInputs(
      frc.lib.interfaces.motor.basic.BasicMotorIO.BasicMotorIOInputs inputs) {
    inputs.appliedVolts = edu.wpi.first.units.Units.Volts.of(motor.getMotorOutputVoltage());
    inputs.current = edu.wpi.first.units.Units.Amps.of(motor.getSupplyCurrent());
    inputs.temperature = edu.wpi.first.units.Units.Celsius.of(motor.getTemperature());
    inputs.isConnected = motor.getLastError() == ErrorCode.OK;

    if (!inputs.isConnected) {
      inputs.activeFaults = frc.lib.interfaces.motor.MotorFaults.getTalonSRXFaults(motor);
    } else {
      inputs.activeFaults = new String[] {};
    }
  }

  @Override
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    updateHardwareInputs((frc.lib.interfaces.motor.basic.BasicMotorIO.BasicMotorIOInputs) inputs);
    inputs.position =
        edu.wpi.first.units.Units.Rotations.of(
            motor.getSelectedSensorPosition() / motorConfig.countsPerRevolution);
    inputs.velocity =
        edu.wpi.first.units.Units.RotationsPerSecond.of(
            motor.getSelectedSensorVelocity() * 10.0 / motorConfig.countsPerRevolution);
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    this.currentMode = MotorControlMode.VELOCITY;
    this.targetVelocity = velocity;
    double rp100ms =
        velocity.in(edu.wpi.first.units.Units.RotationsPerSecond)
            * motorConfig.countsPerRevolution
            / 10.0;
    motor.set(ControlMode.Velocity, rp100ms);
  }

  @Override
  public void runPosition(Angle position) {
    this.currentMode = MotorControlMode.POSITION;
    this.targetPosition = position;
    double ticks =
        position.in(edu.wpi.first.units.Units.Rotations) * motorConfig.countsPerRevolution;
    motor.set(ControlMode.Position, ticks);
  }

  @Override
  public void runSmartPosition(Angle position) {
    this.currentMode = MotorControlMode.SMART_POSITION;
    this.targetPosition = position;
    double ticks =
        position.in(edu.wpi.first.units.Units.Rotations) * motorConfig.countsPerRevolution;
    motor.set(ControlMode.MotionMagic, ticks);
  }

  @Override
  public void runVelocity(AngularVelocity velocity, int slot) {
    motor.selectProfileSlot(slot, 0);
    runVelocity(velocity);
  }

  @Override
  public void runPosition(Angle position, int slot) {
    motor.selectProfileSlot(slot, 0);
    runPosition(position);
  }

  @Override
  public void runSmartPosition(Angle position, int slot) {
    motor.selectProfileSlot(slot, 0);
    runSmartPosition(position);
  }

  @Override
  public void runVoltage(Voltage volts) {
    this.currentMode = MotorControlMode.IDLE;
    motor.set(ControlMode.PercentOutput, volts.in(Volts) / 12.0);
  }

  @Override
  public void runPercentOutput(double percent) {
    this.currentMode = MotorControlMode.IDLE;
    motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void stop() {
    this.currentMode = MotorControlMode.IDLE;
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void setOffset(Angle offset) {
    motor.setSelectedSensorPosition(
        offset.in(edu.wpi.first.units.Units.Rotations) * motorConfig.countsPerRevolution);
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    motor.configMotionCruiseVelocity(maxVel * motorConfig.countsPerRevolution / 10.0);
    motor.configMotionAcceleration(maxAccel * motorConfig.countsPerRevolution / 10.0);
    motor.configAllowableClosedloopError(slot, allowedErr * motorConfig.countsPerRevolution);
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    motor.configPeakOutputForward(max);
    motor.configPeakOutputReverse(min);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motor.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  public void setCurrentLimit(Current current) {
    motor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, current.in(Amps), current.in(Amps), 0));
  }

  @Override
  public MotorIOInputs getMotorIOInputs() {
    return inputs;
  }

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    motor.config_kP(slot, p);
    motor.config_kI(slot, i);
    motor.config_kD(slot, d);
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    motor.config_kF(slot, v); // CTRE uses kF for feedforward
  }
}
