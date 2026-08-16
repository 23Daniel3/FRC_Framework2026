package frc.lib.interfaces.motor.pwm;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a CTRE Talon SRX driven via PWM. */
public class MotorIOPWMTalonSRX extends MotorIOPWM {
  public MotorIOPWMTalonSRX(String name, int port, BasicMotorConfig config) {
    super(name, new PWMTalonSRX(port), config);
  }
}
