package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants.IntakeIntention;
import frc.robot.subsystems.superstructure.SuperStructure;

public final class IntakeCommands {

  public static Command in(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setIntakeIntention(IntakeIntention.IN),
        () -> superStructure.setIntakeIntention(IntakeIntention.NON_INTENTION));
  }

  public static Command out(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setIntakeIntention(IntakeIntention.OUT),
        () -> superStructure.setIntakeIntention(IntakeIntention.NON_INTENTION));
  }

  public static Command middle(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setIntakeIntention(IntakeIntention.MIDDLE),
        () -> superStructure.setIntakeIntention(IntakeIntention.NON_INTENTION));
  }
}
