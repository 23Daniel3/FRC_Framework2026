package frc.lib.interfaces.motor.pwm;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a classic CTRE Talon driven via PWM. */
public class MotorIOPWMTalon extends MotorIOPWM {
  public MotorIOPWMTalon(String name, int port, BasicMotorConfig config) {
    super(name, new Talon(port), config);
  }
}
