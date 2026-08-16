package frc.robot.subsystems.intake;

import frc.lib.interfaces.motor.advanced.MotorController;
import frc.lib.interfaces.motor.advanced.MotorControllerNone;
import frc.lib.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import frc.lib.interfaces.subsystem.SubsystemIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends SubsystemIO<IntakeIOInputsAutoLogged> {
  @AutoLog
  public static class IntakeIOInputs {
    public MotorIOInputs rollerMotorInputs = new MotorIOInputs();
    public MotorIOInputs intakeMotorInputs = new MotorIOInputs();
    public boolean coastButtonPressed = false;
  }

  @Override
  public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

  public default MotorController controlIntakeMotor() {
    return new MotorControllerNone();
  }

  public default MotorController controlRollerMotor() {
    return new MotorControllerNone();
  }
}
