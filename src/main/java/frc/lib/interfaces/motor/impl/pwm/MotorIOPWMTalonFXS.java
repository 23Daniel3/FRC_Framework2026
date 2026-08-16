package frc.lib.interfaces.motor.impl.pwm;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/** Basic IO for a CTRE Talon FX/FXS driven via PWM. */
public class MotorIOPWMTalonFXS extends MotorIOPWM {
  public MotorIOPWMTalonFXS(String name, int port, BasicMotorConfig config) {
    super(name, new PWMTalonFX(port), config);
  }
}
