package frc.robot.subsystems.intake;

import frc.lib.interfaces.motor.MotorController;
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

  public default MotorController controlIntakeMotor() {
    return null;
  }

  public default MotorController controlRollerMotor() {
    return null;
  }
}
