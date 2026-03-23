package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void stop() {}

  public default void configurePIDF(double kP, double kI, double kD, double kF) {}

  public default void configureMaxOutput(double maxOutput) {}

  public default void setOffset(Angle offset) {}

  public default void setVoltage(Voltage volts) {}

  public default void runPercentOutput(double percentOutput) {}

  public default void runPosition(Angle position) {}
}
