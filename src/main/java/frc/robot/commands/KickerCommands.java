package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.kicker.KickerConstants.KickerIntention;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.BooleanSupplier;

public final class KickerCommands {
  private static Command runWithInversion(
      SuperStructure superStructure,
      BooleanSupplier shiftKey,
      KickerIntention normal,
      KickerIntention inverted) {
    return Commands.run(
            () -> superStructure.setKickerIntention(shiftKey.getAsBoolean() ? inverted : normal))
        .finallyDo(() -> superStructure.setKickerIntention(KickerIntention.NON_INTENTION));
  }

  public static Command shootShift(SuperStructure superStructure, BooleanSupplier shiftKey) {
    return runWithInversion(
        superStructure, shiftKey, KickerIntention.IDLE_SPIN, KickerIntention.REVERSE);
  }

  public static Command shoot(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setKickerIntention(KickerIntention.SHOOT),
        () -> superStructure.setKickerIntention(KickerIntention.NON_INTENTION));
  }

  public static Command idleSpin(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setKickerIntention(KickerIntention.IDLE_SPIN),
        () -> superStructure.setKickerIntention(KickerIntention.NON_INTENTION));
  }

  public static Command reverse(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setKickerIntention(KickerIntention.REVERSE),
        () -> superStructure.setKickerIntention(KickerIntention.NON_INTENTION));
  }

  public static Command stop(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setKickerIntention(KickerIntention.STOP),
        () -> superStructure.setKickerIntention(KickerIntention.NON_INTENTION));
  }
}
