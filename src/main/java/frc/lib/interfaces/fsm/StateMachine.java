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
 * Generic state machine with automatic logging via AdvantageKit.
 *
 * <p><b>Evaluation order per cycle ({@link #update()}):</b>
 *
 * <ol>
 *   <li>{@code onUpdate()} of the current state;
 *   <li><b>Global</b> transitions, in registration order (first true wins);
 *   <li><b>Local</b> transitions of the current state, in registration order.
 * </ol>
 *
 * <p>Global transitions take priority over local ones. At most <b>one</b> transition occurs per
 * cycle (prevents infinite loops, but means chains A → B → C cost one cycle per hop).
 *
 * <p><b>Callback convention:</b> use {@code onEnter} for discrete effects (stop motor, change mode,
 * reset controller) and {@code onUpdate} for setpoints that track a dynamic target (e.g.,
 * shot-on-the-move RPM) — a setpoint applied only in {@code onEnter} is frozen at the value from
 * the moment of the transition.
 *
 * <p><b>Important note:</b> the {@code onEnter} of the initial state is <b>not</b> executed in the
 * constructor or on the first {@link #update()}; it only runs when transitioning into that state
 * (or via {@link #forceState(Enum)}).
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

  /** Current FPGA timestamp in seconds (compatible with SimHooks.stepTiming in tests). */
  private static double nowSeconds() {
    return Timer.getFPGATimestamp();
  }

  public StateConfig state(S stateEnum) {
    if (states.containsKey(stateEnum)) {
      DriverStation.reportError(
          "FSM '" + name + "': state " + stateEnum + " configured twice (overwriting).", true);
    }
    StateConfig config = new StateConfig();
    states.put(stateEnum, config);
    return config;
  }

  public void addState(S key, State<S> state) {
    states.put(key, state);
  }

  public void update() {
    // 1. At most ONE transition per cycle (chains A → B → C cost one cycle per hop)
    S next = checkTransitions();
    if (next != null && next != current) {
      transitionTo(next);
    }

    // 2. Runs the current state logic (ensures hardware is updated)
    State<S> state = states.get(current);
    if (state != null) {
      state.onUpdate();
    }

    // Logs
    Logger.recordOutput("FSM/" + name + "/State", current.toString());
    Logger.recordOutput("FSM/" + name + "/TimeInState", getTimeInState());
  }

  private S checkTransitions() {
    // Priority 1: Global transitions (external interrupts)
    for (Transition t : globalTransitions) {
      if (t.target != current && t.condition.getAsBoolean()) {
        return t.target;
      }
    }

    // Priority 2: Local transitions (state logical flow)
    State<S> state = states.get(current);
    return (state != null) ? state.nextState() : null;
  }

  public StateMachine<S> addGlobalTransition(S target, BooleanSupplier condition) {
    globalTransitions.add(new Transition(target, condition));
    return this;
  }

  /**
   * Registers a global "request" transition: while {@code requestActive} is true and the machine is
   * NOT in the entry state or any of the states that already satisfy the request, it transitions to
   * {@code entryState}.
   *
   * <p>Replaces the manual and fragile pattern of {@code addGlobalTransition(ENTRY, () -> request
   * == X && state != A && state != B ...)}, where forgetting a state in the list causes unintended
   * re-entry into {@code onEnter}.
   *
   * @param requestActive condition indicating that the request is active
   * @param entryState the state the machine enters to serve the request
   * @param satisfyingStates states that already satisfy (or are on the way to satisfying) the
   *     request; {@code entryState} itself is excluded automatically
   */
  @SafeVarargs
  public final StateMachine<S> addRequestTransition(
      BooleanSupplier requestActive, S entryState, S... satisfyingStates) {
    final Set<S> satisfied =
        satisfyingStates.length == 0
            ? EnumSet.of(entryState)
            : EnumSet.of(satisfyingStates[0], satisfyingStates);
    satisfied.add(entryState); // entryState must never re-trigger its own transition
    return addGlobalTransition(
        entryState, () -> requestActive.getAsBoolean() && !satisfied.contains(current));
  }

  public void forceState(S next) {
    if (next != current) transitionTo(next);
  }

  public S getCurrentState() {
    return current;
  }

  /** Time spent in the current state, in seconds. */
  public double getTimeInState() {
    return nowSeconds() - stateEnterTimestampSec;
  }

  /**
   * Verifies that all enum values have a registered state, reporting a DriverStation warning for
   * each missing state. Call at the end of the subsystem constructor to catch "forgot to configure
   * state X" on the bench rather than at competition.
   *
   * @return true if all states are registered
   */
  public boolean validateComplete() {
    boolean complete = true;
    for (S s : enumClass.getEnumConstants()) {
      if (!states.containsKey(s)) {
        DriverStation.reportWarning(
            "FSM '" + name + "': state " + s + " of the enum was not configured.", false);
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

    /** Transitions after {@code seconds} seconds in the current state. */
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
