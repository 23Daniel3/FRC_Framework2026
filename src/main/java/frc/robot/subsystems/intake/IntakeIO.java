package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public MotorIOInputs rollerMotorInputs = new MotorIOInputs();
    public MotorIOInputs intakeMotorInputs = new MotorIOInputs();
    public boolean coastButtonPressed = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void resetPosition(Angle position) {}

  public default void runVelocityRollerMotor(AngularVelocity velocity) {}

  public default void runPositionIntakeMotor(Angle position) {}

  public default void runPercentOutputRollerMotor(double percentOutput) {}

  public default void runPercentOutputIntakeMotor(double percentOutput) {}

  public default void stopRollerMotor() {}

  public default void stopIntakeMotor() {}

  public default void configurePIDSVRollerMotor(
      double kP, double kI, double kD, double kS, double kV) {}

  public default void configurePIDFIntakeMotor(double kP, double kI, double kD, double kF) {}

  public default void setCurrentLimitIntakeMotor(Current current) {}

  public default void setCurrentLimitRollerMotor(Current current) {}

  public default void setVoltageCompensationIntakeMotor(Voltage voltage) {}

  public default void setBrakeMode(boolean enable) {}
}
