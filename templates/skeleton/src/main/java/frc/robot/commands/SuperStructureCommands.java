package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotRequest;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.BooleanSupplier;

/** Comandos do piloto para a SuperStructure. */
public final class SuperStructureCommands {
  private SuperStructureCommands() {}

  /**
   * Comando padrao da SuperStructure: traduz os botoes do piloto no {@link RobotRequest} ativo, a
   * cada ciclo. E o unico ponto de entrada da intencao do piloto na FSM geral.
   *
   * <pre>
   * eject        -> EJECT   (vence os outros botoes)
   * intake       -> INTAKE
   * (nenhum)     -> IDLE
   * </pre>
   */
  public static Command manageRequests(
      SuperStructure superStructure, BooleanSupplier intakeButton, BooleanSupplier ejectButton) {
    return Commands.run(
            () -> {
              RobotRequest request;
              if (ejectButton.getAsBoolean()) {
                request = RobotRequest.EJECT;
              } else if (intakeButton.getAsBoolean()) {
                request = RobotRequest.INTAKE;
              } else {
                request = RobotRequest.IDLE;
              }
              superStructure.setRequest(request);
            },
            superStructure)
        .withName("Requests by pilot");
  }
}
