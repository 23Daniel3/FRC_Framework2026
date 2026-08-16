package frc.lib.interfaces.motor.impl.pwm;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a VEX Victor SP driven via PWM. */
public class MotorIOPWMVictorSP extends MotorIOPWM {
  public MotorIOPWMVictorSP(String name, int port, BasicMotorConfig config) {
    super(name, new VictorSP(port), config);
  }
}
