package frc.robot.subsystems.conveyor;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
  @AutoLog
  public static class ConveyorIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public default void updateInputs(ConveyorIOInputs inputs) {}

  public default void setCurrentLimit(Current current) {}

  public default void setVoltageCompensation(Voltage voltage) {}

  public default void runPercentOutput(double percentOutput) {}

  public default void stop() {}
}
