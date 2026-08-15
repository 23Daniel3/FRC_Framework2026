package frc.lib.interfaces.motor.basic;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Minimal control surface for simple, open-loop motor controllers — the kind you'd use to drive a
 * CIM, RedLine 775pro, BAG, or any other brushed motor with no built-in closed-loop capability.
 *
 * <p>No position/velocity control is exposed here — that's {@link
 * frc.lib.interfaces.motor.advanced.MotorController}, which extends this interface.
 */
public interface BasicMotorController {

  void setBrakeMode(boolean enabled);

  void runVoltage(Voltage volts);

  void runPercentOutput(double percent);

  void stop();

  void setCurrentLimit(Current current);
}
