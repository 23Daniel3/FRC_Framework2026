package frc.lib.interfaces.fsm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Maquina de estados generica com logging automatico via AdvantageKit.
 *
 * <p><b>Ordem de avaliacao por ciclo ({@link #update()}):</b>
 *
 * <ol>
 *   <li>{@code onUpdate()} do estado atual;
 *   <li>Transicoes <b>globais</b>, na ordem de registro (a primeira verdadeira vence);
 *   <li>Transicoes <b>locais</b> do estado atual, na ordem de registro.
 * </ol>
 *
 * <p>Transicoes globais tem prioridade sobre as locais. No maximo <b>uma</b> transicao ocorre por
 * ciclo (evita loops infinitos, mas significa que cadeias A → B → C levam um ciclo por salto).
 *
 * <p><b>Convencao de callbacks:</b> use {@code onEnter} para efeitos discretos (parar motor, mudar
 * modo, resetar controlador) e {@code onUpdate} para setpoints que seguem um alvo dinamico (ex.:
 * RPM de shot-on-the-move) — um setpoint aplicado apenas no {@code onEnter} fica congelado no valor
 * do momento da transicao.
 *
 * <p><b>Observacao importante:</b> o {@code onEnter} do estado inicial <b>nao</b> e executado no
 * construtor nem no primeiro {@link #update()}; ele roda apenas quando ha transicao para esse
 * estado (ou via {@link #forceState(Enum)}).
 */
public class StateMachine<S extends Enum<S>> {

  private final String name;
  private final Class<S> enumClass;
  private final Map<S, State<S>> states;
  private final List<Transition> globalTransitions = new ArrayList<>();
  private S current;
  private double stateEnterTimestampSec;

  public StateMachine(String name, Class<S> enumClass, S initialState) {
    this.name = name;
    this.enumClass = enumClass;
    this.states = new EnumMap<>(enumClass);
    this.current = initialState;
    this.stateEnterTimestampSec = nowSeconds();
  }

  /** Timestamp atual em segundos da FPGA (compatível com SimHooks.stepTiming nos testes). */
  private static double nowSeconds() {
    return Timer.getFPGATimestamp();
  }

  public StateConfig state(S stateEnum) {
    if (states.containsKey(stateEnum)) {
      DriverStation.reportError(
          "FSM '" + name + "': estado " + stateEnum + " configurado duas vezes (sobrescrevendo).",
          true);
    }
    StateConfig config = new StateConfig();
    states.put(stateEnum, config);
    return config;
  }

  public void addState(S key, State<S> state) {
    states.put(key, state);
  }

  public void update() {
    // 1. No máximo UMA transição por ciclo (cadeias A → B → C custam um ciclo por salto)
    S next = checkTransitions();
    if (next != null && next != current) {
      transitionTo(next);
    }

    // 2. Executa a lógica do estado atual (garante que o hardware seja atualizado)
    State<S> state = states.get(current);
    if (state != null) {
      state.onUpdate();
    }

    // Logs
    Logger.recordOutput("FSM/" + name + "/State", current.toString());
    Logger.recordOutput("FSM/" + name + "/TimeInState", getTimeInState());
  }

  private S checkTransitions() {
    // Prioridade 1: Transições Globais (Interrupções externas)
    for (Transition t : globalTransitions) {
      if (t.target != current && t.condition.getAsBoolean()) {
        return t.target;
      }
    }

    // Prioridade 2: Transições Locais (Fluxo lógico do estado)
    State<S> state = states.get(current);
    return (state != null) ? state.nextState() : null;
  }

  public StateMachine<S> addGlobalTransition(S target, BooleanSupplier condition) {
    globalTransitions.add(new Transition(target, condition));
    return this;
  }

  /**
   * Registra uma transicao global do tipo "request": enquanto {@code requestActive} for verdadeiro
   * e a maquina NAO estiver no estado de entrada nem em nenhum dos estados que ja satisfazem o
   * request, transiciona para {@code entryState}.
   *
   * <p>Substitui o padrao manual e fragil de {@code addGlobalTransition(ENTRY, () -> request == X
   * && state != A && state != B ...)}, no qual esquecer um estado da lista causa re-entrada
   * indevida no {@code onEnter}.
   *
   * @param requestActive condicao que indica que o request esta ativo
   * @param entryState estado pelo qual a maquina entra para atender o request
   * @param satisfyingStates estados que ja atendem (ou estao a caminho de atender) o request; o
   *     proprio {@code entryState} e excluido automaticamente
   */
  @SafeVarargs
  public final StateMachine<S> addRequestTransition(
      BooleanSupplier requestActive, S entryState, S... satisfyingStates) {
    final Set<S> satisfied =
        satisfyingStates.length == 0
            ? EnumSet.of(entryState)
            : EnumSet.of(satisfyingStates[0], satisfyingStates);
    satisfied.add(entryState); // o entryState nunca deve re-disparar a propria transicao
    return addGlobalTransition(
        entryState, () -> requestActive.getAsBoolean() && !satisfied.contains(current));
  }

  public void forceState(S next) {
    if (next != current) transitionTo(next);
  }

  public S getCurrentState() {
    return current;
  }

  /** Tempo no estado atual, em segundos. */
  public double getTimeInState() {
    return nowSeconds() - stateEnterTimestampSec;
  }

  /**
   * Verifica se todos os valores do enum possuem um estado registrado, reportando um warning no
   * DriverStation para cada estado ausente. Chame ao final do construtor do subsistema para pegar
   * "esqueci de configurar o estado X" em bancada em vez de em competicao.
   *
   * @return true se todos os estados estao registrados
   */
  public boolean validateComplete() {
    boolean complete = true;
    for (S s : enumClass.getEnumConstants()) {
      if (!states.containsKey(s)) {
        DriverStation.reportWarning(
            "FSM '" + name + "': estado " + s + " do enum nao foi configurado.", false);
        complete = false;
      }
    }
    return complete;
  }

  private void transitionTo(S next) {
    Logger.recordOutput("FSM/" + name + "/Transition", current + " -> " + next);

    State<S> currentState = states.get(current);
    if (currentState != null) currentState.onExit();

    current = next;
    stateEnterTimestampSec = nowSeconds();

    State<S> nextState = states.get(current);
    if (nextState != null) nextState.onEnter();
  }

  public class StateConfig implements State<S> {
    private Runnable onEnter = () -> {};
    private Runnable onUpdate = () -> {};
    private Runnable onExit = () -> {};
    private final List<Transition> transitions = new ArrayList<>();

    public StateConfig onEnter(Runnable action) {
      this.onEnter = action;
      return this;
    }

    public StateConfig onUpdate(Runnable action) {
      this.onUpdate = action;
      return this;
    }

    public StateConfig onExit(Runnable action) {
      this.onExit = action;
      return this;
    }

    public StateConfig transitionTo(S targetState, BooleanSupplier condition) {
      transitions.add(new Transition(targetState, condition));
      return this;
    }

    /** Transiciona apos {@code seconds} segundos no estado. */
    public StateConfig transitionAfter(double seconds, S targetState) {
      return transitionTo(targetState, () -> getTimeInState() >= seconds);
    }

    @Override
    public void onEnter() {
      onEnter.run();
    }

    @Override
    public void onUpdate() {
      onUpdate.run();
    }

    @Override
    public void onExit() {
      onExit.run();
    }

    @Override
    public S nextState() {
      for (Transition t : transitions) {
        if (t.condition.getAsBoolean()) {
          return t.target;
        }
      }
      return null;
    }
  }

  private class Transition {
    final S target;
    final BooleanSupplier condition;

    Transition(S target, BooleanSupplier condition) {
      this.target = target;
      this.condition = condition;
    }
  }
}
