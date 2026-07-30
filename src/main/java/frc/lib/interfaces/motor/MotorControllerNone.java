package frc.lib.interfaces.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * No-Op implementation (Null Object Pattern) of the MotorController interface. Used to avoid
 * NullPointerExceptions when hardware is not present.
 */
public class MotorControllerNone implements MotorController {

  @Override
  public void setBrakeMode(boolean enabled) {}

  @Override
  public void setOffset(Angle offset) {}

  @Override
  public void runVoltage(Voltage volts) {}

  @Override
  public void runPercentOutput(double percent) {}

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

  @Override
  public void stop() {}

  @Override
  public void setCurrentLimit(Current current) {}
}
