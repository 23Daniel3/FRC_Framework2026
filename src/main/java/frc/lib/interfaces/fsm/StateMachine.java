package frc.lib.interfaces.fsm;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * A generic, fluent State Machine implementation for FRC robots.
 *
 * @param <S> The Enum type defining the states.
 */
public class StateMachine<S extends Enum<S>> {

  private final String name;
  private final Map<S, State<S>> states;
  private final List<Transition> globalTransitions = new ArrayList<>();
  private S current;
  private double stateEnterTimestamp;

  /**
   * Creates a new StateMachine.
   *
   * @param name The name for logging purposes.
   * @param enumClass The class of the Enum used for states.
   * @param initialState The starting state.
   */
  public StateMachine(String name, Class<S> enumClass, S initialState) {
    this.name = name;
    this.states = new EnumMap<>(enumClass);
    this.current = initialState;
    this.stateEnterTimestamp = Logger.getTimestamp();
  }

  /**
   * Starts the configuration for a specific state using a fluent API.
   *
   * @param stateEnum The state to configure.
   * @return A configuration builder for this state.
   */
  public StateConfig state(S stateEnum) {
    StateConfig config = new StateConfig();
    states.put(stateEnum, config);
    return config;
  }

  /** Manually adds a full State implementation (legacy/complex mode). */
  public void addState(S key, State<S> state) {
    states.put(key, state);
  }

  /** Should be called in the subsystem's periodic method. */
  public void update() {
    State<S> state = states.get(current);

    Logger.recordOutput("FSM/" + name + "/State", current.toString());
    Logger.recordOutput("FSM/" + name + "/TimeInState", getTimeInState());

    if (state == null) return;

    state.onUpdate();

    for (Transition t : globalTransitions) {
      if (t.condition.getAsBoolean()) {
        transitionTo(t.target);
        return;
      }
    }

    S next = state.nextState();
    if (next != null && next != current) {
      transitionTo(next);
    }
  }

  public StateMachine<S> addGlobalTransition(S target, BooleanSupplier condition) {
    globalTransitions.add(new Transition(target, condition));
    return this;
  }

  /** Forces a transition to a specific state (failsafe). */
  public void forceState(S next) {
    if (next != current) transitionTo(next);
  }

  public S getCurrentState() {
    return current;
  }

  public double getTimeInState() {
    return Logger.getTimestamp() - stateEnterTimestamp;
  }

  private void transitionTo(S next) {
    Logger.recordOutput("FSM/" + name + "/Transition", current + " -> " + next);

    State<S> currentState = states.get(current);
    if (currentState != null) currentState.onExit();

    current = next;
    stateEnterTimestamp = Logger.getTimestamp();

    State<S> nextState = states.get(current);
    if (nextState != null) nextState.onEnter();
  }

  // --- Inner Helper Classes for Fluent API ---

  /** Internal class to build states using lambdas. */
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

    /**
     * Adds a transition condition.
     *
     * @param targetState The state to go to.
     * @param condition The condition (boolean supplier) that triggers the transition.
     */
    public StateConfig transitionTo(S targetState, BooleanSupplier condition) {
      transitions.add(new Transition(targetState, condition));
      return this;
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
