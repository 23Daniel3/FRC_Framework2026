package frc.lib.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public MotorIOInputs motorLeftInputs = new MotorIOInputs();
    public MotorIOInputs motorRightInputs = new MotorIOInputs();
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(Voltage volts) {}

  public default void setVelocity(AngularVelocity velocity) {}

  public default void setPercentOutput(double percentOutput) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void runPosition(Angle position) {}

  public default void reset() {}
}
