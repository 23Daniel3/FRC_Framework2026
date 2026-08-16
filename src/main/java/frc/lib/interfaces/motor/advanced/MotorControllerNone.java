package frc.lib.interfaces.motor.advanced;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.interfaces.motor.basic.BasicMotorControllerNone;

/**
 * No-Op implementation (Null Object Pattern) of the {@link MotorController} interface. Used to
 * avoid NullPointerExceptions when hardware is not present.
 */
public class MotorControllerNone extends BasicMotorControllerNone implements MotorController {

  @Override
  public void setOffset(Angle offset) {}

  @Override
  public void runVelocity(AngularVelocity velocity) {}

  @Override
  public void runPosition(Angle position) {}

  @Override
  public void runSmartPosition(Angle position) {}

  @Override
  public void runVelocity(AngularVelocity velocity, int slot) {}

  @Override
  public void runPosition(Angle position, int slot) {}

  @Override
  public void runSmartPosition(Angle position, int slot) {}
}
