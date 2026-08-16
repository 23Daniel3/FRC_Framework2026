package frc.lib.interfaces.motor.advanced;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.interfaces.motor.basic.BasicMotorController;

/**
 * Full control surface for motors with closed-loop capability (integrated or external encoder +
 * onboard/RIO PID). Extends {@link BasicMotorController}, so every advanced motor is also, by
 * construction, a basic one — {@code setBrakeMode}, {@code runVoltage}, {@code runPercentOutput},
 * {@code stop}, and {@code setCurrentLimit} are inherited, not redeclared.
 */
public interface MotorController extends BasicMotorController {

  void setOffset(Angle offset);

  void runVelocity(AngularVelocity velocity);

  void runPosition(Angle position);

  void runSmartPosition(Angle position);

  void runVelocity(AngularVelocity velocity, int slot);

  void runPosition(Angle position, int slot);

  void runSmartPosition(Angle position, int slot);
}
