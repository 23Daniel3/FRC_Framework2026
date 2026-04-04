package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.GeneralIntention;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.BooleanSupplier;

public final class SuperStructureCommands {

  public static Command manageIntentions(
      SuperStructure superStructure,
      BooleanSupplier collectBtn,
      BooleanSupplier shootBtn,
      BooleanSupplier closedButton) {

    return Commands.run(
            () -> {
              boolean collecting = collectBtn.getAsBoolean();
              boolean shooting = shootBtn.getAsBoolean();
              Boolean closed = closedButton.getAsBoolean();

              GeneralIntention intention;

              if (closed) {
                intention = GeneralIntention.CLOSED;
              } else if (collecting && shooting) {
                intention = GeneralIntention.COLLECT_SHOOTING;
              } else if (collecting) {
                intention = GeneralIntention.COLLECT;
              } else if (shooting) {
                intention = GeneralIntention.SHOOT;
              } else {
                intention = GeneralIntention.IDLE;
              }

              superStructure.setGeneralIntention(intention);
            },
            superStructure)
        .withName("Intentions by pilot");
  }

  public static Command collect(SuperStructure superStructure) {
    return Commands.startEnd(
            () -> superStructure.setGeneralIntention(GeneralIntention.COLLECT),
            () -> superStructure.setGeneralIntention(GeneralIntention.IDLE),
            superStructure)
        .withName("Collent Intention");
  }

  public static Command shoot(SuperStructure superStructure) {
    return Commands.startEnd(
            () -> superStructure.setGeneralIntention(GeneralIntention.SHOOT),
            () -> superStructure.setGeneralIntention(GeneralIntention.IDLE),
            superStructure)
        .withName("Shoot Intention");
  }

  public static Command collectShooting(SuperStructure superStructure) {
    return Commands.startEnd(
            () -> superStructure.setGeneralIntention(GeneralIntention.COLLECT_SHOOTING),
            () -> superStructure.setGeneralIntention(GeneralIntention.IDLE),
            superStructure)
        .withName("Collent_Shooting Intention");
  }

  public static Command closed(SuperStructure superStructure) {
    return Commands.startEnd(
            () -> superStructure.setGeneralIntention(GeneralIntention.CLOSED),
            () -> superStructure.setGeneralIntention(GeneralIntention.IDLE),
            superStructure)
        .withName("Shoot Intention");
  }

  public static Command idle(SuperStructure superStructure) {
    return Commands.run(
        () -> superStructure.setGeneralIntention(GeneralIntention.IDLE), superStructure);
  }
}
