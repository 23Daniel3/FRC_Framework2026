package frc.lib.interfaces.motor.pwm;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a REV SparkMax driven via PWM. */
public class MotorIOPWMSparkMax extends MotorIOPWM {
  public MotorIOPWMSparkMax(String name, int port, BasicMotorConfig config) {
    super(name, new PWMSparkMax(port), config);
  }
}
