package frc.lib.subsystems.cradle;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface CradleIO {
  @AutoLog
  public static class CradleIOInputs {
    public MotorIOInputs motorLeftInputs = new MotorIOInputs();
    public MotorIOInputs motorRightInputs = new MotorIOInputs();
    public boolean sensorIsTrue = false;
  }

  public default void updateInputs(CradleIOInputs inputs) {}

  public default void setVoltage(Voltage volts) {}

  public default void setVelocity(AngularVelocity velocityRadPerSec) {}

  public default void setPercentOutput(double percentOutput) {}

  public default void setInvertPercentOutput(double percentOutput) {}

  public default void setStop() {}
}
