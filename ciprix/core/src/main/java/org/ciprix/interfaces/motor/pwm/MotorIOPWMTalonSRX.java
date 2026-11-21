package org.ciprix.interfaces.motor.pwm;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a CTRE Talon SRX driven via PWM. */
public class MotorIOPWMTalonSRX extends MotorIOPWM {
  public MotorIOPWMTalonSRX(String name, int port, BasicMotorConfig config) {
    super(name, new PWMTalonSRX(port), config);
  }
}
