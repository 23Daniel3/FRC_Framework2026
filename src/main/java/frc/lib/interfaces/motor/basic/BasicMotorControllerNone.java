package frc.lib.interfaces.motor.basic;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * No-Op implementation (Null Object Pattern) of {@link BasicMotorController}. Used to avoid
 * NullPointerExceptions when hardware is not present.
 */
public class BasicMotorControllerNone implements BasicMotorController {

  @Override
  public void setBrakeMode(boolean enabled) {}

  @Override
  public void runVoltage(Voltage volts) {}

  @Override
  public void runPercentOutput(double percent) {}

  @Override
  public void stop() {}

  @Override
  public void setCurrentLimit(Current current) {}
}
