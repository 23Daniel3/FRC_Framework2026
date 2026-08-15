package frc.lib.interfaces.motor.impl;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.lib.interfaces.motor.MotorControlMode;
import frc.lib.interfaces.motor.basic.BasicMotorBase;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/**
 * Basic-only IO for classic PWM speed controllers (PWMVictorSPX, PWMTalonSRX, PWMSparkMax, Victor,
 * Talon, Jaguar, ...) driving a simple brushed motor like the CIM or RedLine 775pro.
 *
 * <p>Accepts any WPILib {@link MotorController}, so it works with whichever PWM (or even a {@code
 * MotorControllerGroup}) object your subsystem already constructs.
 *
 * <p>PWM controllers give no telemetry back to the RoboRIO (no current, temperature, or fault
 * reporting) and neutral (brake/coast) behavior is set on the controller hardware itself — those
 * calls are safely ignored here rather than throwing.
 */
public class MotorIOPWM extends BasicMotorBase {

  private final MotorController motor;
  private final BasicMotorIOInputs inputs = new BasicMotorIOInputs();

  public MotorIOPWM(String name, MotorController motor, BasicMotorConfig config) {
    super(name, config);
    this.motor = motor;
    motor.setInverted(config.inverted);
  }

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    inputs.percentOutput = motor.get();
    inputs.appliedVolts = Volts.of(motor.get() * RobotController.getBatteryVoltage());
    // No CAN/telemetry link on plain PWM — assume connected if wired, since there's no way to
    // detect otherwise from software.
    inputs.isConnected = true;
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
    // Not controllable over plain PWM — neutral mode is set on the controller hardware itself
    // (DIP switch, onboard button, or CAN config if the same device also has a CAN link).
  }

  @Override
  public void setCurrentLimit(Current current) {
    // PWM controllers have no current-limiting capability; enforce it upstream (breaker/fuse) or
    // switch to a CAN-based implementation (MotorIOTalonSRX, MotorIOSparkMaxBrushed) instead.
  }

  @Override
  public BasicMotorIOInputs getMotorIOInputs() {
    return inputs;
  }
}
