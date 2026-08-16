package frc.lib.interfaces.motor.impl.pwm;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a REV SparkFlex driven via PWM. */
public class MotorIOPWMSparkFlex extends MotorIOPWM {
  public MotorIOPWMSparkFlex(String name, int port, BasicMotorConfig config) {
    super(name, new PWMSparkFlex(port), config);
  }
}
