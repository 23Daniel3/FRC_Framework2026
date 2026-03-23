package frc.lib.subsystems.funnel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void setVoltage(Voltage volts) {}

  public default void runVelocity(AngularVelocity velocity) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void resetPosition() {}

  public default void setPercentOutput(double percentOutput) {}
}
