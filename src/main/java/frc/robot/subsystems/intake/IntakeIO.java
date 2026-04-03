package frc.robot.subsystems.intake;

import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
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
  public void updateInputs(IntakeIOInputsAutoLogged inputs);

  public MotorController controlIntakeMotor();

  public MotorController controlRollerMotor();
}
