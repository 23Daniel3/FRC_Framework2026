package frc.lib.interfaces.motor.impl;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorControlMode;
import frc.lib.interfaces.motor.MotorFaults;
import frc.lib.interfaces.motor.basic.BasicMotorBase;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;
import frc.lib.util.security.SparkUtil;

/**
 * Basic-only IO for a REV SparkMax running in brushed mode ({@code MotorType.kBrushed}) — the
 * standard way to drive a CIM, RedLine 775pro, Mini-CIM, or BAG motor over CAN with a REV
 * controller.
 *
 * <p>Exposes exactly what a brushed motor with no built-in encoder can do: percent/voltage output,
 * current limiting, brake/coast, open-loop ramping, and telemetry (current, temperature, bus
 * voltage, faults). No closed-loop control is available here since a plain CIM/RedLine has no
 * integrated sensor.
 *
 * <p>If you wire an external quadrature/duty-cycle encoder to the SparkMax's alternate encoder port
 * and want closed-loop control on top of it, that belongs in a dedicated implementation of {@link
 * frc.lib.interfaces.motor.advanced.MotorIO} — this class is intentionally kept simple.
 */
public class MotorIOSparkMaxBrushed extends BasicMotorBase {

  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final BasicMotorIOInputs inputs = new BasicMotorIOInputs();

  public MotorIOSparkMaxBrushed(String name, int id, BasicMotorConfig config) {
    super(name, config);

    this.motor = new SparkMax(id, MotorType.kBrushed);
    this.motorConfig = new SparkMaxConfig();

    if (config.leaderMotorID != 0) {
      motorConfig.follow(config.leaderMotorID, config.followerInverted);
    }

    motorConfig
        .inverted(config.inverted)
        .smartCurrentLimit((int) config.currentLimit.in(Amps))
        .idleMode(config.brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
        .voltageCompensation(config.nominalVoltage.in(Volts))
        .openLoopRampRate(config.openLoopRampSeconds);

    applyConfig(true);
  }

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    inputs.percentOutput = motor.getAppliedOutput();
    inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.current = Amps.of(motor.getOutputCurrent());
    inputs.temperature = Celsius.of(motor.getMotorTemperature());
    inputs.isConnected = !motor.hasActiveFault();
    inputs.activeFaults =
        motor.hasActiveFault() ? MotorFaults.getSparkFaults(motor) : new String[] {};
  }

  @Override
  public void runVoltage(Voltage volts) {
    currentMode = MotorControlMode.VOLTAGE;
    motor.setVoltage(volts.in(Volts));
  }

  @Override
  public void runPercentOutput(double percent) {
    currentMode = MotorControlMode.PERCENT;
    motor.set(clampOutput(percent));
  }

  @Override
  public void stop() {
    currentMode = MotorControlMode.IDLE;
    motor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (motor == null) return;
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
  public BasicMotorIOInputs getMotorIOInputs() {
    return inputs;
  }

  private void applyConfig(boolean isInit) {
    ResetMode resetMode =
        isInit ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    SparkUtil.tryUntilOk(
        motor, 5, () -> motor.configure(motorConfig, resetMode, PersistMode.kPersistParameters));
  }
}
