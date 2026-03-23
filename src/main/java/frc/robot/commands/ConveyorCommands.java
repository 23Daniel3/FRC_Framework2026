package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorIntention;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.BooleanSupplier;

public final class ConveyorCommands {
  private static Command runWithInversion(
      SuperStructure superStructure,
      BooleanSupplier shiftKey,
      ConveyorIntention normal,
      ConveyorIntention inverted) {
    return Commands.run(
            () -> superStructure.setConveyorIntention(shiftKey.getAsBoolean() ? inverted : normal))
        .finallyDo(() -> superStructure.setConveyorIntention(ConveyorIntention.NON_INTENTION));
  }

  public static Command runShift(SuperStructure superStructure, BooleanSupplier shiftKey) {
    return runWithInversion(
        superStructure, shiftKey, ConveyorIntention.RUN, ConveyorIntention.REVERSE);
  }

  public static Command run(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setConveyorIntention(ConveyorIntention.RUN),
        () -> superStructure.setConveyorIntention(ConveyorIntention.NON_INTENTION));
  }

  public static Command reverse(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setConveyorIntention(ConveyorIntention.REVERSE),
        () -> superStructure.setConveyorIntention(ConveyorIntention.NON_INTENTION));
  }

  public static Command stop(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setConveyorIntention(ConveyorIntention.STOP),
        () -> superStructure.setConveyorIntention(ConveyorIntention.NON_INTENTION));
  }

  public static Command wiggle(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setConveyorIntention(ConveyorIntention.WIGGLE),
        () -> superStructure.setConveyorIntention(ConveyorIntention.NON_INTENTION));
  }
}
