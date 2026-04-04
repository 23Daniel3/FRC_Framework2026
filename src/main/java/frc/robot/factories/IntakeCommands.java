package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.superstructure.SuperStructure;

public final class IntakeCommands {
  
  private IntakeCommands() {}

  public static Command defaultCommand(SuperStructure superStructure, Intake intake) {
    return Commands.run(() -> intake.setRequest(superStructure.getIntakeRequest()), intake);
  }

  public static Command in(Intake intake) {
    return Commands.runOnce(() -> intake.setRequest(IntakeRequest.IN), intake);
  }

  public static Command out(Intake intake) {
    return Commands.runOnce(() -> intake.setRequest(IntakeRequest.OUT), intake);
  }

  public static Command collect(Intake intake) {
    return Commands.runOnce(() -> intake.setRequest(IntakeRequest.COLLECT), intake);
  }
}