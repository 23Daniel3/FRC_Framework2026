package frc.lib.interfaces.motor.impl.pwm;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a CTRE Victor SPX driven via PWM. */
public class MotorIOPWMVictorSPX extends MotorIOPWM {
  public MotorIOPWMVictorSPX(String name, int port, BasicMotorConfig config) {
    super(name, new PWMVictorSPX(port), config);
  }
}
