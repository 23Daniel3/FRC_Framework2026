package org.ciprix.interfaces.motor.pwm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import org.ciprix.interfaces.motor.basic.BasicMotorBase;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;

/**
 * Common base class for all PWM-driven motor controllers. Centralizes the boilerplate for basic
 * operations; features that PWM cannot provide (current limit, brake mode from software) are simply
 * not implemented.
 *
 * <p>A PWM controller knows nothing about the battery, so the compensation to the nominal voltage
 * is done here: the output is recomputed every cycle from the present battery voltage.
 */
public abstract class MotorIOPWM extends BasicMotorBase {

  protected final MotorController motor;

  public MotorIOPWM(String name, MotorController motor, BasicMotorConfig config) {
    super(name, config);
    this.motor = motor;
    motor.setInverted(config.inverted);
  }

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    double output = motor.get();
    inputs.percentOutput = output;
    inputs.appliedVolts = Volts.of(output * RobotController.getBatteryVoltage());
    inputs.isConnected = true;
  }

  @Override
  protected void applyVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  protected void applyPercentOutput(double percent) {
    motor.setVoltage(percent * nominalVolts);
  }

  @Override
  protected void applyStop() {
    motor.stopMotor();
  }

  /** Every cycle: the voltage compensation depends on the battery voltage of the moment. */
  @Override
  protected double getCommandRefreshSeconds() {
    return 0.0;
  }
}
