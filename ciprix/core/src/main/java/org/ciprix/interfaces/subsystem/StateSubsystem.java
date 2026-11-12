package org.ciprix.interfaces.subsystem;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;
import org.ciprix.interfaces.fsm.StateMachine;
import org.ciprix.util.ConstantsLogger;
import org.ciprix.util.PeriodicTimer;
import org.ciprix.util.Report;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Subsistema baseado no padrao Request → State: o mundo externo expressa intencao via um enum de
 * Request ({@link #setRequest}); a FSM interna decide como chegar la; {@link #atGoal()} indica se o
 * request atual foi atendido.
 *
 * <p>Use {@link #bindRequest} para declarar, de uma vez: (a) por qual estado a FSM entra para
 * atender um request; (b) qual estado final o satisfaz; (c) quais estados intermediarios nao devem
 * ser interrompidos pela transicao global. Com isso, {@code atGoal()} e derivado automaticamente —
 * nao e mais necessario sobrescrever com um switch manual.
 *
 * <p><b>Backup do copiloto:</b> {@link #override} sobrepoe o request por um unico ciclo. Enquanto o
 * copiloto segura o botao, o subsistema segue o override; o request da SuperStructure continua
 * guardado e volta a valer sozinho no ciclo seguinte ao soltar. {@code atGoal()} continua relativo
 * ao request da SuperStructure, entao a FSM geral nunca avanca por causa de um override.
 */
public abstract class StateSubsystem<
        R extends Enum<R>, S extends Enum<S>, I extends LoggableInputs, T extends SubsystemIO<I>>
    extends SubsystemBase {

  protected final StateMachine<S> fsm;
  protected final I inputs;
  protected final T io;

  /** Request de quem comanda o subsistema normalmente (a SuperStructure, pelo relay). */
  protected R currentRequest;

  /** Override transitorio do copiloto: vale por um ciclo e e limpo no fim do {@code periodic()}. */
  private R transientOverrideRequest = null;

  private R lastOverrideRequest = null;
  private boolean overrideActive = false;

  /** Mapa request → estado que o satisfaz, preenchido por {@link #bindRequest}. */
  private final Map<R, S> requestGoals;

  // Nome e chaves de log calculados uma vez: o loop nao concatena strings.
  private final String subsystemName;
  private final String requestKey;
  private final String activeRequestKey;
  private final String overrideKey;
  private final String atGoalKey;
  private final String faultKey;
  private final String statusKey;

  private final Alert faultAlert;
  private boolean faultAlertActive = false;
  private boolean nullRequestReported = false;

  // Status em texto: so e remontado quando algo muda.
  private String status = "";
  private String statusDetail = "";
  private R statusRequest;
  private S statusState;
  private boolean statusAtGoal;
  private boolean statusOverride;
  private boolean statusFault;
  private boolean statusDirty = true;

  public StateSubsystem(
      String name, I inputs, T io, Class<S> stateEnum, S initialState, R initialRequest) {
    this(name, inputs, io, stateEnum, initialState, initialRequest, null);
  }

  public StateSubsystem(
      String name,
      I inputs,
      T io,
      Class<S> stateEnum,
      S initialState,
      R initialRequest,
      Class<?> constantsClass) {
    Objects.requireNonNull(initialRequest, "initialRequest");
    setName(name);
    this.subsystemName = name;
    this.inputs = inputs;
    this.io = io;
    this.currentRequest = initialRequest;
    this.requestGoals = new EnumMap<>(initialRequest.getDeclaringClass());
    this.fsm = new StateMachine<>(name, stateEnum, initialState);

    this.requestKey = name + "/request";
    this.activeRequestKey = name + "/activeRequest";
    this.overrideKey = name + "/overrideActive";
    this.atGoalKey = name + "/atGoal";
    this.faultKey = name + "/fault";
    this.statusKey = name + "/status";
    this.faultAlert = new Alert(name + ": falha na maquina de estados", AlertType.kWarning);

    if (constantsClass != null) {
      ConstantsLogger.logConstants(constantsClass, name);
    }
  }

  /**
   * Declara como um request e atendido pela FSM.
   *
   * <p>Registra uma transicao global para {@code entryState} sempre que {@code request} estiver
   * ativo e a FSM nao estiver no {@code entryState}, no {@code goalState} nem em nenhum dos {@code
   * intermediateStates}. Tambem registra {@code goalState} como o estado que faz {@link #atGoal()}
   * retornar true para esse request.
   *
   * <p>Exemplo (Shooter): {@code bindRequest(SHOOT, FLYWHEEL_RAMPING, SHOOTING, KICKER_RAMPING)} —
   * request SHOOT entra por FLYWHEEL_RAMPING, e satisfeito em SHOOTING, e KICKER_RAMPING nao deve
   * ser interrompido.
   *
   * @param request o request a mapear
   * @param entryState estado de entrada para atender o request
   * @param goalState estado que satisfaz o request ({@code atGoal() == true})
   * @param intermediateStates estados intermediarios que ja estao "a caminho" do goal
   */
  @SafeVarargs
  protected final void bindRequest(R request, S entryState, S goalState, S... intermediateStates) {
    requestGoals.put(request, goalState);

    // goalState + intermediarios; o proprio entryState e excluido pela FSM automaticamente.
    final S[] satisfied = Arrays.copyOf(intermediateStates, intermediateStates.length + 1);
    satisfied[intermediateStates.length] = goalState;

    fsm.addRequestTransition(request.name(), () -> isRequest(request), entryState, satisfied);
  }

  /** Define o request do subsistema. E o que o relay da SuperStructure chama todo ciclo. */
  public void setRequest(R request) {
    if (request == null) {
      reportNullRequest("setRequest");
      return;
    }
    this.currentRequest = request;
  }

  /**
   * Backup do copiloto: sobrepoe o request por <b>um ciclo</b>. Chame todo ciclo enquanto o botao
   * estiver pressionado (os comandos gerados por {@code @AutoCommandFactory} ja fazem isso). Ao
   * parar de chamar, o subsistema volta sozinho para o request da SuperStructure.
   */
  public void override(R request) {
    if (request == null) {
      reportNullRequest("override");
      return;
    }
    this.transientOverrideRequest = request;
  }

  /** Request de quem comanda o subsistema normalmente (sem considerar o override do copiloto). */
  public R getRequest() {
    return currentRequest;
  }

  /** Request que a FSM esta seguindo de fato: o override do copiloto, se houver, ou o normal. */
  public R getActiveRequest() {
    if (transientOverrideRequest != null) return transientOverrideRequest;
    return overrideActive ? lastOverrideRequest : currentRequest;
  }

  /** True enquanto o copiloto estiver sobrepondo o request deste subsistema. */
  public boolean isOverrideActive() {
    return overrideActive || transientOverrideRequest != null;
  }

  public S getState() {
    return fsm.getCurrentState();
  }

  public I getInputs() {
    return inputs;
  }

  /** A FSM do subsistema, para telemetria, testes e visualizadores. */
  public StateMachine<S> getStateMachine() {
    return fsm;
  }

  /** True quando o request que a FSM esta seguindo (considerando o override) e {@code request}. */
  protected boolean isRequest(R request) {
    return (transientOverrideRequest != null ? transientOverrideRequest : currentRequest)
        == request;
  }

  protected boolean notInState(S state) {
    return getState() != state;
  }

  /**
   * Trigger que fica ativo enquanto o subsistema estiver em um dos estados. Exemplo: {@code
   * shooter.inState(ShooterState.SHOOTING).onTrue(LedCommands.blink(led))}.
   */
  @SafeVarargs
  public final Trigger inState(S state, S... others) {
    return new Trigger(fsm.inState(state, others));
  }

  /** Trigger que fica ativo enquanto o request da SuperStructure estiver atendido. */
  public Trigger atGoalTrigger() {
    return new Trigger(this::atGoal);
  }

  /**
   * True quando a FSM esta em uma rota de falha: um estado estourou o {@code timeoutAfter} ou uma
   * condicao de seguranca disparou, e o request que falhou ainda esta bloqueado.
   */
  public boolean hasFault() {
    return fsm.hasFault();
  }

  /** Libera de novo o request bloqueado por uma falha, sem esperar ele ser solto. */
  public void clearFault() {
    fsm.clearFault();
  }

  /**
   * Texto curto com o que o subsistema esta fazendo, por exemplo {@code "SHOOT | KICKER_RAMPING"}.
   * Publicado em {@code <nome>/status}.
   */
  public String getStatus() {
    return status;
  }

  /** Acrescenta um detalhe do proprio subsistema ao status (ex.: "3120 rpm"). Vazio remove. */
  protected void setStatusDetail(String detail) {
    String value = detail == null ? "" : detail;
    if (!value.equals(statusDetail)) {
      statusDetail = value;
      statusDirty = true;
    }
  }

  @Override
  public final void periodic() {
    PeriodicTimer.start(subsystemName);

    io.updateInputs(inputs);
    Logger.processInputs(subsystemName, inputs);

    // O override pedido depois do ultimo ciclo vale durante este periodic() inteiro.
    overrideActive = transientOverrideRequest != null;
    lastOverrideRequest = transientOverrideRequest;

    fsm.update();

    sPeriodic();

    final boolean atGoal = atGoal();
    final boolean fault = fsm.hasFault();
    final R activeRequest = overrideActive ? lastOverrideRequest : currentRequest;

    Logger.recordOutput(requestKey, currentRequest.toString());
    Logger.recordOutput(activeRequestKey, activeRequest.toString());
    Logger.recordOutput(overrideKey, overrideActive);
    Logger.recordOutput(atGoalKey, atGoal);
    Logger.recordOutput(faultKey, fault);

    updateFaultAlert(fault);
    updateStatus(activeRequest, atGoal, fault);

    // Fim do ciclo: o override e transitorio. Se o copiloto continuar segurando o botao, o
    // comando chama override() de novo antes do proximo periodic().
    transientOverrideRequest = null;

    PeriodicTimer.stop(subsystemName);
  }

  protected void sPeriodic() {}

  /**
   * Retorna true se o request atual foi atendido. A implementacao padrao usa o mapa construido por
   * {@link #bindRequest}: o request e considerado atendido quando a FSM esta no {@code goalState}
   * vinculado a ele. Requests sem vinculo retornam true (sem intencao = satisfeito). Sobrescreva
   * apenas se o subsistema precisar de uma nocao de goal que nao seja "estar em um estado".
   *
   * <p>O goal e sempre o do request normal ({@link #getRequest()}), nunca o do override do
   * copiloto.
   */
  public boolean atGoal() {
    S goal = requestGoals.get(currentRequest);
    return goal == null || getState() == goal;
  }

  private void updateFaultAlert(boolean fault) {
    if (fault == faultAlertActive) return;
    faultAlertActive = fault;
    if (fault) {
      faultAlert.setText(subsystemName + ": " + fsm.getLastFault());
    }
    faultAlert.set(fault);
  }

  private void updateStatus(R activeRequest, boolean atGoal, boolean fault) {
    final S state = getState();
    if (!statusDirty
        && activeRequest == statusRequest
        && state == statusState
        && atGoal == statusAtGoal
        && overrideActive == statusOverride
        && fault == statusFault) {
      return;
    }
    statusRequest = activeRequest;
    statusState = state;
    statusAtGoal = atGoal;
    statusOverride = overrideActive;
    statusFault = fault;
    statusDirty = false;

    StringBuilder sb = new StringBuilder(48);
    sb.append(activeRequest).append(" | ").append(state);
    if (overrideActive) sb.append(" | OVERRIDE");
    if (fault) sb.append(" | FALHA: ").append(fsm.getLastFault());
    if (!statusDetail.isEmpty()) sb.append(" | ").append(statusDetail);
    status = sb.toString();
    Logger.recordOutput(statusKey, status);
  }

  private void reportNullRequest(String method) {
    if (nullRequestReported) return;
    nullRequestReported = true;
    Report.error(
        subsystemName + ": " + method + "(null) ignorado. Um request nunca pode ser null.");
  }
}
