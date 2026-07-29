package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.superstructure.SuperStructure;

public final class IntakeCommands {

  private IntakeCommands() {}

  /** Default Command: Relays the target request calculated by the SuperStructure. */
  public static Command defaultCommand(SuperStructure superStructure, Intake intake) {
    return Commands.run(() -> intake.setRequest(superStructure.getIntakeRequest()), intake)
        .withName("Intake Default (Superstructure Relay)");
  }

  /** Manual Override: Retract intake. */
  public static Command in(Intake intake) {
    return Commands.run(() -> intake.setRequest(IntakeRequest.IN), intake)
        .withName("Intake Override IN");
  }

  /** Manual Override: Extend intake without spinning roller. */
  public static Command out(Intake intake) {
    return Commands.run(() -> intake.setRequest(IntakeRequest.OUT), intake)
        .withName("Intake Override OUT");
  }

  /** Manual Override: Extend and spin roller to collect. */
  public static Command collect(Intake intake) {
    return Commands.run(() -> intake.setRequest(IntakeRequest.COLLECT), intake)
        .withName("Intake Override COLLECT");
  }

  /** Manual Override: Stop intake. */
  public static Command stop(Intake intake) {
    return Commands.run(() -> intake.setRequest(IntakeRequest.STOP), intake)
        .withName("Intake Override STOP");
  }
}
