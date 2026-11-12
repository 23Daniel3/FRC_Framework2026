package frc.robot.subsystems.superstructure;

import frc.robot.Constants.RobotRequest;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants.RollerRequest;
import org.ciprix.interfaces.subsystem.SuperStructureBase;

/**
 * A maquina de estados geral: decide o estado do robo a partir do request do piloto e, ao entrar em
 * cada estado, diz o que cada subsistema deve fazer. Os subsistemas recebem esses requests pelo
 * comando padrao gerado ({@code RollerCommands.defaultCommand(superStructure, roller)}), que le
 * {@link #getRollerRequest()}.
 */
public class SuperStructure extends SuperStructureBase<RobotRequest, RobotState> {

  private final Roller roller;
  private RollerRequest rollerRequest = RollerRequest.STOP;

  public SuperStructure(Roller roller) {
    super("Subsystems/SuperStructure", RobotState.class, RobotState.IDLE, RobotRequest.IDLE);
    this.roller = roller;

    generalFsm.state(RobotState.IDLE).onEnter(() -> rollerRequest = RollerRequest.STOP);
    generalFsm.state(RobotState.INTAKING).onEnter(() -> rollerRequest = RollerRequest.INTAKE);
    generalFsm.state(RobotState.EJECTING).onEnter(() -> rollerRequest = RollerRequest.EJECT);

    // Request do robo -> estado de entrada (e os estados que ja o atendem).
    bindRequest(RobotRequest.IDLE, RobotState.IDLE);
    bindRequest(RobotRequest.INTAKE, RobotState.INTAKING);
    bindRequest(RobotRequest.EJECT, RobotState.EJECTING);
    generalFsm.validateComplete();
  }

  /** Request do roller, lido todo ciclo pelo comando padrao gerado. */
  public RollerRequest getRollerRequest() {
    return rollerRequest;
  }

  /** True quando o roller chegou no estado pedido (use nas transicoes da FSM geral). */
  public boolean isRollerAtGoal() {
    return roller.atGoal();
  }
}
