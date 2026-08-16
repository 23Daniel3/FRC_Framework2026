package frc.lib.interfaces.motor.impl.pwm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.lib.interfaces.motor.MotorControlMode;
import frc.lib.interfaces.motor.basic.BasicMotorBase;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/**
 * Common base class for all PWM-driven motor controllers. Centralizes the boilerplate for basic
 * operations and explicitly empty methods for features not supported by PWM (like current limits
 * and brake mode).
 */
public abstract class MotorIOPWM extends BasicMotorBase {

  protected final MotorController motor;
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
    // Not controllable over plain PWM — neutral mode is set on the controller hardware itself.
  }

  @Override
  public void setCurrentLimit(Current current) {
    // PWM controllers have no current-limiting capability via software.
  }

  @Override
  public BasicMotorIOInputs getMotorIOInputs() {
    return inputs;
  }
}
