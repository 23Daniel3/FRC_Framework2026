package frc.robot.subsystems.kicker;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
    public boolean isSensorActive = false;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void runVelocity(AngularVelocity velocity, Voltage voltage) {}

  public default void runPercentOutput(double percentOutput) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void setCurrentLimit(Current current) {}

  public default void setVoltageCompensation(Voltage voltage) {}
}
