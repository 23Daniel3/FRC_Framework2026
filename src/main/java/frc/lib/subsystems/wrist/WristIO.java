package frc.lib.subsystems.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(Voltage volts) {}

  public default void setVelocity(AngularVelocity velocityRadPerSec) {}

  public default void stop() {}

  public default void setPercentOutput(double percentOutput) {}

  public default void setInvertPercentOutput(double percentOutput) {}

  public default void runPosition(Angle position) {}

  public default void resetEncoder() {}
}
