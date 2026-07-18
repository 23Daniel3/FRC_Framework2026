package frc.lib.interfaces.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.fsm.StateMachine;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
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
 */
public abstract class StateSubsystem<
        R extends Enum<R>, S extends Enum<S>, I extends LoggableInputs, T extends SubsystemIO<I>>
    extends SubsystemBase {

  protected final StateMachine<S> fsm;
  protected final I inputs;
  protected final T io;
  protected R currentRequest;

  /** Mapa request → estado que o satisfaz, preenchido por {@link #bindRequest}. */
  private final Map<R, S> requestGoals = new HashMap<>();

  public StateSubsystem(
      String name, I inputs, T io, Class<S> stateEnum, S initialState, R initialRequest) {
    setName(name);
    this.inputs = inputs;
    this.io = io;
    this.currentRequest = initialRequest;

    this.fsm = new StateMachine<>(name, stateEnum, initialState);
  }

  public StateSubsystem(
      String name,
      I inputs,
      T io,
      Class<S> stateEnum,
      S initialState,
      R initialRequest,
      Class<?> constantsClass) {
    setName(name);
    this.inputs = inputs;
    this.io = io;
    this.currentRequest = initialRequest;
    this.fsm = new StateMachine<>(name, stateEnum, initialState);

    ConstantsLogger.logConstants(constantsClass, name);
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
  protected final void bindRequest(
      R request, S entryState, S goalState, S... intermediateStates) {
    requestGoals.put(request, goalState);

    final Set<S> satisfied = new HashSet<>();
    satisfied.add(goalState);
    Collections.addAll(satisfied, intermediateStates);

    // O framework ja ignora transicoes cujo alvo e o estado atual, entao o proprio
    // entryState fica automaticamente excluido.
    fsm.addGlobalTransition(
        entryState, () -> isRequest(request) && !satisfied.contains(getState()));
  }

  public void setRequest(R request) {
    this.currentRequest = request;
  }

  public R getRequest() {
    return currentRequest;
  }

  public S getState() {
    return fsm.getCurrentState();
  }

  public I getInputs() {
    return inputs;
  }

  protected boolean isRequest(R request) {
    return currentRequest == request;
  }

  protected boolean notInState(S state) {
    return getState() != state;
  }

  @Override
  public final void periodic() {
    PeriodicTimer.start(getName());

    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    fsm.update();

    sPeriodic();

    Logger.recordOutput(getName() + "/request", currentRequest.toString());
    Logger.recordOutput(getName() + "/atGoal", atGoal());

    PeriodicTimer.stop(getName());
  }

  protected void sPeriodic() {}

  /**
   * Retorna true se o request atual foi atendido. A implementacao padrao usa o mapa construido por
   * {@link #bindRequest}: o request e considerado atendido quando a FSM esta no {@code goalState}
   * vinculado a ele. Requests sem vinculo retornam true (sem intencao = satisfeito). Sobrescreva
   * apenas se o subsistema precisar de uma nocao de goal que nao seja "estar em um estado".
   */
  public boolean atGoal() {
    S goal = requestGoals.get(currentRequest);
    return goal == null || getState() == goal;
  }
}
