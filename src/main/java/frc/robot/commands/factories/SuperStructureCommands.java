package frc.robot.commands.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotRequest;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.BooleanSupplier;

public final class SuperStructureCommands {

  /**
   * Default command da SuperStructure: traduz os botoes do piloto no {@link RobotRequest} ativo, a
   * cada ciclo. Este e o UNICO ponto de entrada de intencao do piloto na FSM geral.
   *
   * <p><b>Tabela de prioridade (a primeira linha verdadeira vence):</b>
   *
   * <pre>
   * close                      -> CLOSE          (recolhe tudo; vence qualquer outro botao)
   * collect + shoot            -> COLLECT_SHOOT  (coleta enquanto atira)
   * collect                    -> COLLECT
   * shoot                      -> SHOOT
   * (nenhum botao)             -> IDLE
   * </pre>
   *
   * <p>Para adicionar um modo novo: crie o RobotRequest, insira a linha na posicao de prioridade
   * correta abaixo e atualize esta tabela.
   */
  public static Command manageRequests(
      SuperStructure superStructure,
      BooleanSupplier collectBtn,
      BooleanSupplier shootBtn,
      BooleanSupplier closeButton) {

    return Commands.run(
            () -> {
              boolean collecting = collectBtn.getAsBoolean();
              boolean shooting = shootBtn.getAsBoolean();
              boolean close = closeButton.getAsBoolean();

              RobotRequest request;

              if (close) {
                request = RobotRequest.CLOSE;
              } else if (collecting && shooting) {
                request = RobotRequest.COLLECT_SHOOT;
              } else if (collecting) {
                request = RobotRequest.COLLECT;
              } else if (shooting) {
                request = RobotRequest.SHOOT;
              } else {
                request = RobotRequest.IDLE;
              }

              superStructure.setRequest(request);
            },
            superStructure)
        .withName("Requests by pilot");
  }

  public static Command collect(SuperStructure superStructure) {
    return Commands.startEnd(
            () -> superStructure.setRequest(RobotRequest.COLLECT),
            () -> superStructure.setRequest(RobotRequest.IDLE),
            superStructure)
        .withName("Collect Request");
  }

  public static Command shoot(SuperStructure superStructure) {
    return Commands.startEnd(
            () -> superStructure.setRequest(RobotRequest.SHOOT),
            () -> superStructure.setRequest(RobotRequest.IDLE),
            superStructure)
        .withName("Shoot Request");
  }

  public static Command collectShooting(SuperStructure superStructure) {
    return Commands.startEnd(
            () -> superStructure.setRequest(RobotRequest.COLLECT_SHOOT),
            () -> superStructure.setRequest(RobotRequest.IDLE),
            superStructure)
        .withName("Collect_Shooting Request");
  }

  public static Command close(SuperStructure superStructure) {
    return Commands.startEnd(
            () -> superStructure.setRequest(RobotRequest.CLOSE),
            () -> superStructure.setRequest(RobotRequest.IDLE),
            superStructure)
        .withName("Close Request");
  }

  public static Command idle(SuperStructure superStructure) {
    return Commands.run(() -> superStructure.setRequest(RobotRequest.IDLE), superStructure)
        .withName("Idle Request");
  }
}
