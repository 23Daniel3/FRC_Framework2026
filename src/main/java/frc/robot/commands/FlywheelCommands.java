package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelIntention;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.BooleanSupplier;

public final class FlywheelCommands {
  private static Command runWithInversion(
      SuperStructure superStructure,
      BooleanSupplier shiftKey,
      FlywheelIntention normal,
      FlywheelIntention inverted) {
    return Commands.run(
            () -> superStructure.setFlywheelIntention(shiftKey.getAsBoolean() ? inverted : normal))
        .finallyDo(() -> superStructure.setFlywheelIntention(FlywheelIntention.NON_INTENTION));
  }

  public static Command shootShift(SuperStructure superStructure, BooleanSupplier shiftKey) {
    return runWithInversion(
        superStructure, shiftKey, FlywheelIntention.IDLE_SPIN, FlywheelIntention.REVERSE);
  }

  public static Command shoot(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setFlywheelIntention(FlywheelIntention.SHOOT),
        () -> superStructure.setFlywheelIntention(FlywheelIntention.NON_INTENTION));
  }

  public static Command reverse(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setFlywheelIntention(FlywheelIntention.REVERSE),
        () -> superStructure.setFlywheelIntention(FlywheelIntention.NON_INTENTION));
  }

  public static Command stop(SuperStructure superStructure) {
    return Commands.startEnd(
        () -> superStructure.setFlywheelIntention(FlywheelIntention.STOP),
        () -> superStructure.setFlywheelIntention(FlywheelIntention.NON_INTENTION));
  }
}
