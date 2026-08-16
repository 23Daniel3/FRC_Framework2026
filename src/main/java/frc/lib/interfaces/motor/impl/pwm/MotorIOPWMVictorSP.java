package frc.lib.interfaces.motor.impl.pwm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.lib.interfaces.motor.MotorControlMode;
import frc.lib.interfaces.motor.basic.BasicMotorBase;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a VEX Victor SP driven via PWM. */
public class MotorIOPWMVictorSP extends BasicMotorBase {

  private final VictorSP motor;
  private final BasicMotorIOInputs inputs = new BasicMotorIOInputs();

  public MotorIOPWMVictorSP(String name, int port, BasicMotorConfig config) {
    super(name, config);
    this.motor = new VictorSP(port);
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
  public void setBrakeMode(boolean enabled) {}

  @Override
  public void setCurrentLimit(Current current) {}

  @Override
  public BasicMotorIOInputs getMotorIOInputs() {
    return inputs;
  }
}
