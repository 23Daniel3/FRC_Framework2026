package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants.RollerIntention;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.BooleanSupplier;

public final class RollerCommands {
  private static Command runWithInversion(
      SuperStructure superStructure,
      BooleanSupplier shiftKey,
      RollerIntention normal,
      RollerIntention inverted) {
    return Commands.run(
            () -> superStructure.setRollerIntention(shiftKey.getAsBoolean() ? inverted : normal))
        .finallyDo(() -> superStructure.setRollerIntention(RollerIntention.NON_INTENTION));
  }

  public static Command intakeShift(SuperStructure superStructure, BooleanSupplier shiftKey) {
    return runWithInversion(
        superStructure, shiftKey, RollerIntention.INTAKE, RollerIntention.OUTAKE);
  }

  public static Command intake(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setRollerIntention(RollerIntention.INTAKE),
        () -> superStructure.setRollerIntention(RollerIntention.NON_INTENTION));
  }

  public static Command outake(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setRollerIntention(RollerIntention.OUTAKE),
        () -> superStructure.setRollerIntention(RollerIntention.NON_INTENTION));
  }

  public static Command stop(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setRollerIntention(RollerIntention.STOP),
        () -> superStructure.setRollerIntention(RollerIntention.NON_INTENTION));
  }
}
