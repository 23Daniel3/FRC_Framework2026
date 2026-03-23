package frc.lib.interfaces.fsm;

/**
 * Represents a single state in a State Machine.
 *
 * @param <S> The Enum type defining the states.
 */
public interface State<S extends Enum<S>> {

  /** Executed once when the state is entered. */
  default void onEnter() {}

  /** Executed periodically while the state is active. */
  default void onUpdate() {}

  /** Executed once when the state is exited. */
  default void onExit() {}

  /**
   * Determines the next state to transition to.
   *
   * @return The next state enum, or null to stay in the current state.
   */
  default S nextState() {
    return null;
  }
}
