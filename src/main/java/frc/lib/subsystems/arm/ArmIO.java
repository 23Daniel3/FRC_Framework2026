package frc.lib.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(Voltage volts) {}

  public default void setVelocity(AngularVelocity velocity) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void configurePIDF(double kP, double kI, double kD, double kF) {}

  public default void setOffset(Angle offset) {}

  public default void setPercentOutput(double percentOutput) {}

  public default void runPosition(Angle position) {}
}
