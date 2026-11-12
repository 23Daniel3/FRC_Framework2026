package org.ciprix.interfaces.subsystem;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Objects;
import org.ciprix.interfaces.fsm.StateMachine;
import org.ciprix.util.PeriodicTimer;
import org.ciprix.util.Report;
import org.littletonrobotics.junction.Logger;

/**
 * Base da SuperStructure: a FSM geral do robo.
 *
 * <p>O padrao continua o mesmo: o piloto escreve um request do robo ({@link #setRequest}); a FSM
 * geral decide o estado do robo; no {@code onEnter} de cada estado geral a SuperStructure define o
 * request de cada subsistema; os subsistemas recebem esses requests pelo relay (comando padrao
 * gerado por {@code @AutoCommandFactory}, que le {@code getXxxRequest()}).
 *
 * <p>Esta classe guarda apenas o que e igual em todo robo: o request atual, a FSM geral, o {@code
 * periodic()} na ordem certa, os triggers por estado e a telemetria. Os requests dos subsistemas e
 * os getters {@code getXxxRequest()} ficam na classe do robo, porque dependem dos subsistemas que
 * ele tem.
 *
 * <pre>{@code
 * public class SuperStructure extends SuperStructureBase<RobotRequest, RobotState> {
 *   private IntakeRequest intakeRequest = IntakeRequest.STOP;
 *
 *   public SuperStructure(Intake intake) {
 *     super("Subsystems/SuperStructure", RobotState.class, RobotState.IDLE, RobotRequest.IDLE);
 *
 *     generalFsm.state(RobotState.IDLE).onEnter(() -> intakeRequest = IntakeRequest.STOP);
 *     generalFsm
 *         .state(RobotState.GOING_COLLECT)
 *         .onEnter(() -> intakeRequest = IntakeRequest.COLLECT)
 *         .transitionTo(RobotState.COLLECTING, intake::atGoal);
 *     generalFsm.state(RobotState.COLLECTING);
 *
 *     bindRequest(RobotRequest.IDLE, RobotState.IDLE);
 *     bindRequest(RobotRequest.COLLECT, RobotState.GOING_COLLECT, RobotState.COLLECTING);
 *     generalFsm.validateComplete();
 *   }
 *
 *   public IntakeRequest getIntakeRequest() {
 *     return intakeRequest;
 *   }
 * }
 * }</pre>
 *
 * @param <R> enum de requests do robo (o que o piloto pede)
 * @param <S> enum de estados do robo
 */
public abstract class SuperStructureBase<R extends Enum<R>, S extends Enum<S>>
    extends SubsystemBase {

  /** FSM geral do robo. Configure os estados no construtor da subclasse. */
  protected final StateMachine<S> generalFsm;

  /** Request atual do robo, escrito pelo piloto. */
  protected R robotRequest;

  private final String superStructureName;
  private final String requestKey;
  private final String stateKey;
  private final String faultKey;

  private final Alert faultAlert;
  private boolean faultAlertActive = false;
  private boolean nullRequestReported = false;

  /**
   * @param name nome do subsistema nos logs (ex.: {@code "Subsystems/SuperStructure"}); a FSM geral
   *     e registrada como {@code <name>/RobotState}
   * @param stateEnum enum de estados do robo
   * @param initialState estado do robo ao ligar
   * @param initialRequest request do robo ao ligar
   */
  protected SuperStructureBase(String name, Class<S> stateEnum, S initialState, R initialRequest) {
    Objects.requireNonNull(initialRequest, "initialRequest");
    setName(name);
    this.superStructureName = name;
    this.robotRequest = initialRequest;
    this.generalFsm = new StateMachine<>(name + "/RobotState", stateEnum, initialState);

    this.requestKey = name + "/request";
    this.stateKey = name + "/state";
    this.faultKey = name + "/fault";
    this.faultAlert = new Alert(name + ": falha na maquina de estados", AlertType.kWarning);
  }

  /**
   * Declara por qual estado a FSM geral entra para atender um request do robo e quais estados ja o
   * atendem (ou estao a caminho). Enquanto o request estiver ativo e a FSM nao estiver em nenhum
   * desses estados, ela transiciona para {@code entryState}.
   *
   * <p>Exemplo: {@code bindRequest(SHOOT, GOING_SHOOT, SHOOTING, SHOOTING_RECOVERY)}.
   */
  @SafeVarargs
  protected final void bindRequest(R request, S entryState, S... satisfyingStates) {
    generalFsm.addRequestTransition(
        request.name(), () -> robotRequest == request, entryState, satisfyingStates);
  }

  /** Define o request do robo. E o que os comandos do piloto chamam. */
  public void setRequest(R request) {
    if (request == null) {
      if (!nullRequestReported) {
        nullRequestReported = true;
        Report.error(superStructureName + ": setRequest(null) ignorado.");
      }
      return;
    }
    this.robotRequest = request;
  }

  /** Request atual do robo. */
  public R getRequest() {
    return robotRequest;
  }

  /** Estado atual da FSM geral. */
  public S getRobotState() {
    return generalFsm.getCurrentState();
  }

  /** A FSM geral, para telemetria, testes e visualizadores. */
  public StateMachine<S> getStateMachine() {
    return generalFsm;
  }

  /**
   * Trigger que fica ativo enquanto o robo estiver em um dos estados. Exemplo: {@code
   * superStructure.inState(RobotState.SHOOTING).whileTrue(LedCommands.blink(led))}.
   */
  @SafeVarargs
  public final Trigger inState(S state, S... others) {
    return new Trigger(generalFsm.inState(state, others));
  }

  /** Trigger que fica ativo enquanto o request do robo for {@code request}. */
  public Trigger whenRequested(R request) {
    return new Trigger(() -> robotRequest == request);
  }

  /** True quando a FSM geral esta em uma rota de falha (timeout ou condicao de seguranca). */
  public boolean hasFault() {
    return generalFsm.hasFault();
  }

  /** Libera de novo o request bloqueado por uma falha, sem esperar ele ser solto. */
  public void clearFault() {
    generalFsm.clearFault();
  }

  @Override
  public final void periodic() {
    PeriodicTimer.start(superStructureName);

    beforeUpdate();
    generalFsm.update();
    afterUpdate();

    final boolean fault = generalFsm.hasFault();
    Logger.recordOutput(requestKey, robotRequest.toString());
    Logger.recordOutput(stateKey, generalFsm.getCurrentState().toString());
    Logger.recordOutput(faultKey, fault);
    if (fault != faultAlertActive) {
      faultAlertActive = fault;
      if (fault) {
        faultAlert.setText(superStructureName + ": " + generalFsm.getLastFault());
      }
      faultAlert.set(fault);
    }

    PeriodicTimer.stop(superStructureName);
  }

  /**
   * Roda todo ciclo antes da FSM geral. Use para atualizar o que as condicoes das transicoes leem
   * (calculos de tiro, zonas do campo, sensores).
   */
  protected void beforeUpdate() {}

  /** Roda todo ciclo depois da FSM geral. Use para telemetria propria do robo. */
  protected void afterUpdate() {}
}
