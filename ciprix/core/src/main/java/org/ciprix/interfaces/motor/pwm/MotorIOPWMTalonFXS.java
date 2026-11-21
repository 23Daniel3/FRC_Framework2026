package org.ciprix.interfaces.motor.pwm;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a CTRE Talon FX/FXS driven via PWM. */
public class MotorIOPWMTalonFXS extends MotorIOPWM {
  public MotorIOPWMTalonFXS(String name, int port, BasicMotorConfig config) {
    super(name, new PWMTalonFX(port), config);
  }
}
