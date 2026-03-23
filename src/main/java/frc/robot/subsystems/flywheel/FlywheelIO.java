package frc.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public MotorIOInputs leaderInputs = new MotorIOInputs();
    public MotorIOInputs followerInputs = new MotorIOInputs();
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setCurrentLimit(Current current) {}

  public default void runVelocity(AngularVelocity velocity) {}

  public default void runVoltage(Voltage voltage) {}

  public default void runPercentOutput(double percentOutput) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void configureKSVA(double kS, double kV, double kA) {}
}
