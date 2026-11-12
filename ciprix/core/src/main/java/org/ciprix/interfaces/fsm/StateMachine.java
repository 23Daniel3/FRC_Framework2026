package org.ciprix.interfaces.fsm;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;
import org.littletonrobotics.junction.Logger;

/**
 * Generic state machine with automatic logging via AdvantageKit.
 *
 * <p><b>Evaluation order per cycle ({@link #update()}):</b>
 *
 * <ol>
 *   <li><b>Machine security</b> conditions ({@link #addSecurityTransition});
 *   <li><b>Global</b> transitions (requests and {@link #addGlobalTransition}), in registration
 *       order;
 *   <li><b>Local</b> transitions of the current state: its security conditions, then its normal
 *       transitions in registration order (only after every {@code waitFor} was satisfied), then
 *       its timeout;
 *   <li>{@code onUpdate()} of the state that is current after step 1 to 3.
 * </ol>
 *
 * <p>The first transition that is true wins and at most <b>one</b> transition occurs per cycle
 * (prevents infinite loops, but means chains A → B → C cost one cycle per hop).
 *
 * <p><b>Callback convention:</b> use {@code onEnter} for discrete effects (stop motor, change mode,
 * reset controller) and {@code onUpdate} for setpoints that track a dynamic target (e.g.,
 * shot-on-the-move RPM) — a setpoint applied only in {@code onEnter} is frozen at the value from
 * the moment of the transition.
 *
 * <p><b>Initial state:</b> the {@code onEnter} of the initial state is <b>not</b> executed in the
 * constructor or on the first {@link #update()}; it only runs when transitioning into that state
 * (or via {@link #forceState(Enum)}). Call {@link #runInitialOnEnter()} to opt in.
 *
 * <p><b>Security state and faults:</b> a transition created with {@code timeoutAfter}, {@code
 * transitionToSecurityState} or {@link #addSecurityTransition} is a <i>fault path</i>. When one of
 * them fires, the machine reports it and blocks the request that was being served, so it does not
 * bounce straight back into the state that just failed. The request is accepted again once it is
 * released (or after {@link #clearFault()}).
 *
 * <p>The machine has no dependency on the command framework: it only needs {@link #update()} to be
 * called once per loop.
 */
public class StateMachine<S extends Enum<S>> {

  /** What caused a transition. Decides how it is logged and whether it latches a fault. */
  private enum Kind {
    NORMAL,
    REQUEST,
    TIMEOUT,
    SECURITY,
    FORCED
  }

  /** Mermaid id of the pseudo state used to draw transitions that can start anywhere. */
  private static final String ANY_STATE = "ANY_STATE";

  private final String name;
  private final Class<S> enumClass;
  private final S initial;
  private final Map<S, State<S>> states;
  private final Map<S, StateConfig> configs;
  private final List<Transition> securityTransitions = new ArrayList<>();
  private final List<Transition> globalTransitions = new ArrayList<>();
  private final StateStats[] stats;

  // Log keys are built once: the loop never concatenates strings.
  private final String stateKey;
  private final String timeKey;
  private final String transitionKey;
  private final String reasonKey;
  private final String faultKey;
  private final String faultCountKey;
  private final String lastFaultKey;
  private final String graphKey;
  private final String[][] statsKeys;

  private S current;
  private S previous;
  private S securityState;
  private double stateEnterTimestampSec;
  private boolean runInitialOnEnter;
  private boolean started;

  private boolean securityHold;
  private boolean inFaultState;
  private boolean faultBlockedRequests;
  private int blockedRequests;
  private int faultCount;
  private int transitionCount;
  private String lastFault = "";
  private boolean missingSecurityStateReported;

  public StateMachine(String name, Class<S> enumClass, S initialState) {
    this.name = name;
    this.enumClass = enumClass;
    this.initial = initialState;
    this.states = new EnumMap<>(enumClass);
    this.configs = new EnumMap<>(enumClass);
    this.current = initialState;
    this.previous = initialState;

    String prefix = "FSM/" + name + "/";
    this.stateKey = prefix + "State";
    this.timeKey = prefix + "TimeInState";
    this.transitionKey = prefix + "Transition";
    this.reasonKey = prefix + "TransitionReason";
    this.faultKey = prefix + "Fault";
    this.faultCountKey = prefix + "FaultCount";
    this.lastFaultKey = prefix + "LastFault";
    this.graphKey = prefix + "Graph";

    S[] values = enumClass.getEnumConstants();
    this.stats = new StateStats[values.length];
    this.statsKeys = new String[values.length][];
    for (S value : values) {
      String statsPrefix = prefix + "Stats/" + value.name() + "/";
      stats[value.ordinal()] = new StateStats();
      statsKeys[value.ordinal()] =
          new String[] {
            statsPrefix + "Count",
            statsPrefix + "LastSec",
            statsPrefix + "AverageSec",
            statsPrefix + "MaxSec",
            statsPrefix + "IrregularCount"
          };
    }

    this.stateEnterTimestampSec = RobotClock.now();
  }

  // ---------------------------------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------------------------------

  /** Starts the configuration of a state. Every value of the enum must be configured once. */
  public StateConfig state(S stateEnum) {
    if (states.containsKey(stateEnum)) {
      Report.error(
          "FSM '" + name + "': state " + stateEnum + " configured twice (overwriting).", true);
    }
    StateConfig config = new StateConfig(stateEnum);
    states.put(stateEnum, config);
    configs.put(stateEnum, config);
    return config;
  }

  /** Registers a hand written {@link State}. Prefer {@link #state(Enum)}. */
  public void addState(S key, State<S> state) {
    states.put(key, state);
    configs.remove(key);
  }

  /**
   * Defines the security state of the machine: where it goes when a state times out ({@code
   * timeoutAfter(seconds)}) or a security condition becomes true. Usually the state that stops the
   * mechanism.
   */
  public StateMachine<S> securityState(S state) {
    this.securityState = state;
    return this;
  }

  /**
   * Makes the first {@link #update()} run the {@code onEnter} of the initial state. Without this
   * call the initial state is assumed to already describe the robot at boot.
   */
  public StateMachine<S> runInitialOnEnter() {
    this.runInitialOnEnter = true;
    return this;
  }

  /** Registers a transition that can fire from any state. */
  public StateMachine<S> addGlobalTransition(S target, BooleanSupplier condition) {
    return addGlobalTransition(target, condition, 0.0);
  }

  /**
   * Registers a transition that can fire from any state, only after {@code condition} has been true
   * for {@code seconds} without interruption.
   */
  public StateMachine<S> addGlobalTransition(S target, BooleanSupplier condition, double seconds) {
    globalTransitions.add(new Transition(target, condition, seconds, Kind.NORMAL, "", null));
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
    return addRequestTransition("request", requestActive, entryState, satisfyingStates);
  }

  /**
   * Same as {@link #addRequestTransition(BooleanSupplier, Enum, Enum[])}, with the name of the
   * request. The name appears in the transition log and in the diagram.
   */
  @SafeVarargs
  public final StateMachine<S> addRequestTransition(
      String requestName, BooleanSupplier requestActive, S entryState, S... satisfyingStates) {
    final Set<S> satisfied = EnumSet.of(entryState); // entryState never re-triggers itself
    for (S state : satisfyingStates) {
      satisfied.add(state);
    }
    globalTransitions.add(
        new Transition(entryState, requestActive, 0.0, Kind.REQUEST, requestName, satisfied));
    return this;
  }

  /**
   * Registers a machine wide security condition: while it is true the machine goes to (and stays
   * in) the security state defined with {@link #securityState(Enum)}, whatever the request is.
   * Typical use: a hardware problem that makes the mechanism unsafe to move.
   */
  public StateMachine<S> addSecurityTransition(BooleanSupplier condition) {
    securityTransitions.add(new Transition(null, condition, 0.0, Kind.SECURITY, "security", null));
    return this;
  }

  // ---------------------------------------------------------------------------------------------
  // Runtime
  // ---------------------------------------------------------------------------------------------

  /** Runs one cycle of the machine. Call exactly once per robot loop. */
  public void update() {
    final double now = RobotClock.now();

    if (!started) {
      started = true;
      if (runInitialOnEnter) {
        State<S> initialState = states.get(current);
        if (initialState != null) initialState.onEnter();
      }
      // The configuration is complete at this point, so the diagram is logged once.
      Logger.recordOutput(graphKey, toMermaid());
    }

    // 1. At most ONE transition per cycle (chains A → B → C cost one cycle per hop)
    boolean hold = false;
    Transition fired = null;

    for (int i = 0; i < securityTransitions.size(); i++) {
      Transition t = securityTransitions.get(i);
      // Always evaluated: while the machine is unsafe it must go to, and stay in, security.
      if (!t.poll(now)) continue;
      S target = t.target();
      if (target == null) {
        reportMissingSecurityState();
      } else if (target == current) {
        hold = true;
      } else if (fired == null) {
        fired = t;
      }
    }
    securityHold = hold;

    final boolean globalsMayFire = fired == null && !hold;
    for (int i = 0; i < globalTransitions.size(); i++) {
      fired = consider(fired, globalTransitions.get(i), now, globalsMayFire);
    }

    if (fired == null && !hold) {
      StateConfig config = configs.get(current);
      if (config != null) {
        fired = config.findTransition(now);
      } else {
        State<S> custom = states.get(current);
        S next = custom != null ? custom.nextState() : null;
        if (next != null && next != current) {
          applyTransition(next, Kind.NORMAL, "", now);
        }
      }
    }

    if (fired != null) {
      applyTransition(fired.target(), fired.kind, fired.label, now);
    }

    // 2. Runs the current state logic (ensures hardware is updated)
    State<S> state = states.get(current);
    if (state != null) {
      state.onUpdate();
    }

    // Logs
    Logger.recordOutput(stateKey, current.toString());
    Logger.recordOutput(timeKey, now - stateEnterTimestampSec);
    Logger.recordOutput(faultKey, hasFault());
  }

  /**
   * Evaluates one transition keeping the conditions lazy: once a winner exists, the remaining
   * conditions are only read when they carry state of their own (a debounce timer or a blocked
   * request).
   */
  private Transition consider(Transition winner, Transition t, double now, boolean mayFire) {
    boolean eligible = mayFire && winner == null && t.canFireFrom(current);
    if (!eligible && t.debounceSeconds <= 0.0 && !t.blockedByFault) {
      return winner;
    }
    boolean active = t.poll(now);
    return eligible && active ? t : winner;
  }

  /** Forces the machine into {@code next}, running {@code onExit} and {@code onEnter}. */
  public void forceState(S next) {
    if (next != current) applyTransition(next, Kind.FORCED, "forced", RobotClock.now());
  }

  private void applyTransition(S next, Kind kind, String label, double now) {
    final S from = current;
    recordExit(from, now);

    Logger.recordOutput(transitionKey, from + " -> " + next);
    Logger.recordOutput(reasonKey, label);

    State<S> fromState = states.get(from);
    if (fromState != null) fromState.onExit();

    previous = from;
    current = next;
    stateEnterTimestampSec = now;
    transitionCount++;

    if (kind == Kind.TIMEOUT || kind == Kind.SECURITY) {
      latchFault(from, next, label);
    } else {
      inFaultState = false;
      if (kind == Kind.FORCED) unblockRequests();
    }

    StateConfig config = configs.get(next);
    if (config != null) config.resetRuntime();

    State<S> nextState = states.get(next);
    if (nextState != null) nextState.onEnter();
  }

  private void latchFault(S from, S to, String label) {
    inFaultState = true;
    faultCount++;
    lastFault = from + " -> " + to + " (" + label + ")";

    // Only the request that was being served through the failed state is blocked. A request that
    // has nothing to do with it (for example STOP arriving at the same time) stays free.
    unblockRequests();
    for (int i = 0; i < globalTransitions.size(); i++) {
      Transition t = globalTransitions.get(i);
      if (t.kind == Kind.REQUEST && t.satisfied.contains(from) && t.condition.getAsBoolean()) {
        t.blockedByFault = true;
        blockedRequests++;
      }
    }
    faultBlockedRequests = blockedRequests > 0;

    Logger.recordOutput(faultCountKey, faultCount);
    Logger.recordOutput(lastFaultKey, lastFault);
    Report.warning(
        "FSM '"
            + name
            + "': "
            + lastFault
            + (faultBlockedRequests ? ". The request stays blocked until it is released." : "."));
  }

  private void unblockRequests() {
    for (int i = 0; i < globalTransitions.size(); i++) {
      globalTransitions.get(i).blockedByFault = false;
    }
    blockedRequests = 0;
  }

  private void recordExit(S state, double now) {
    final double duration = now - stateEnterTimestampSec;
    final StateStats s = stats[state.ordinal()];
    s.count++;
    s.lastSeconds = duration;
    s.totalSeconds += duration;
    s.maxSeconds = Math.max(s.maxSeconds, duration);
    s.minSeconds = Math.min(s.minSeconds, duration);

    final String[] keys = statsKeys[state.ordinal()];
    StateConfig config = configs.get(state);
    if (config != null && config.hasRegularTime()) {
      if (duration < config.regularMinSeconds || duration > config.regularMaxSeconds) {
        s.irregularCount++;
      }
      Logger.recordOutput(keys[4], s.irregularCount);
    }
    Logger.recordOutput(keys[0], s.count);
    Logger.recordOutput(keys[1], s.lastSeconds);
    Logger.recordOutput(keys[2], s.getAverageSeconds());
    Logger.recordOutput(keys[3], s.maxSeconds);
  }

  private void reportMissingSecurityState() {
    if (missingSecurityStateReported) return;
    missingSecurityStateReported = true;
    Report.error(
        "FSM '"
            + name
            + "': a security transition fired but no security state was defined. Call"
            + " fsm.securityState(...).");
  }

  // ---------------------------------------------------------------------------------------------
  // Queries
  // ---------------------------------------------------------------------------------------------

  public String getName() {
    return name;
  }

  public S getCurrentState() {
    return current;
  }

  /** State the machine was in before the last transition (the initial state before any). */
  public S getPreviousState() {
    return previous;
  }

  /** Time spent in the current state, in seconds. */
  public double getTimeInState() {
    return RobotClock.now() - stateEnterTimestampSec;
  }

  /** True when the machine is in {@code state} or in any of {@code others}. */
  @SafeVarargs
  public final boolean isIn(S state, S... others) {
    if (current == state) return true;
    for (S other : others) {
      if (current == other) return true;
    }
    return false;
  }

  /**
   * Condition that is true while the machine is in one of the given states. It is the building
   * block of the {@code inState(...)} triggers of the subsystems and of the SuperStructure.
   */
  @SafeVarargs
  public final BooleanSupplier inState(S state, S... others) {
    final Set<S> set = EnumSet.of(state, others);
    return () -> set.contains(current);
  }

  /**
   * True while a fault path is in effect: a machine security condition is holding the machine, a
   * request is still blocked by a timeout or security transition, or (when no request was involved)
   * the machine is still in the state it was sent to.
   */
  public boolean hasFault() {
    return securityHold || blockedRequests > 0 || (inFaultState && !faultBlockedRequests);
  }

  /** Accepts again the requests blocked by a fault, without waiting for them to be released. */
  public void clearFault() {
    unblockRequests();
    inFaultState = false;
  }

  /** Description of the last fault path that fired, or an empty string. */
  public String getLastFault() {
    return lastFault;
  }

  /** How many times a fault path (timeout or security) fired since boot. */
  public int getFaultCount() {
    return faultCount;
  }

  /** How many transitions happened since boot. */
  public int getTransitionCount() {
    return transitionCount;
  }

  /** Timing statistics of {@code state}, updated every time the machine leaves it. */
  public StateStats getStats(S state) {
    return stats[state.ordinal()];
  }

  /**
   * True when the current state declared {@code regularTime(min, max)} and the machine has already
   * been in it for longer than {@code max}.
   */
  public boolean isOvertime() {
    StateConfig config = configs.get(current);
    return config != null && config.hasRegularTime() && getTimeInState() > config.regularMaxSeconds;
  }

  /**
   * Verifies that all enum values have a registered state, reporting a DriverStation warning for
   * each missing state. Call at the end of the subsystem constructor to catch "forgot to configure
   * state X" on the bench rather than at competition.
   *
   * <p>It also warns about states that no transition can reach and about fault paths that need a
   * security state that was never defined.
   *
   * @return true if all states are registered
   */
  public boolean validateComplete() {
    boolean complete = true;
    for (S s : enumClass.getEnumConstants()) {
      if (!states.containsKey(s)) {
        Report.warning("FSM '" + name + "': state " + s + " of the enum was not configured.");
        complete = false;
      }
    }

    boolean needsSecurityState = !securityTransitions.isEmpty();
    boolean hasCustomStates = false;
    Set<S> reachable = EnumSet.of(initial);
    for (Transition t : globalTransitions) {
      if (t.explicitTarget != null) reachable.add(t.explicitTarget);
    }
    for (S s : enumClass.getEnumConstants()) {
      StateConfig config = configs.get(s);
      if (config == null) {
        hasCustomStates |= states.containsKey(s);
        continue;
      }
      for (Transition t : config.allTransitions()) {
        if (t.explicitTarget != null) {
          reachable.add(t.explicitTarget);
        } else {
          needsSecurityState = true;
        }
      }
    }

    if (needsSecurityState) {
      if (securityState == null) {
        Report.warning(
            "FSM '"
                + name
                + "': a timeout or security transition has no target. Call"
                + " fsm.securityState(...).");
        complete = false;
      } else {
        reachable.add(securityState);
      }
    }

    // Hand written states have transitions the machine cannot see, so the check is skipped.
    if (!hasCustomStates) {
      for (S s : enumClass.getEnumConstants()) {
        if (states.containsKey(s) && !reachable.contains(s)) {
          Report.warning(
              "FSM '" + name + "': no transition leads to state " + s + " (only forceState).");
        }
      }
    }
    return complete;
  }

  /**
   * Describes the machine as a Mermaid state diagram. It is logged once under {@code
   * FSM/<name>/Graph}; paste the text into any Mermaid viewer to see the machine.
   */
  public String toMermaid() {
    StringBuilder sb = new StringBuilder(512);
    sb.append("stateDiagram-v2\n");
    sb.append("  [*] --> ").append(initial.name()).append('\n');
    for (S s : enumClass.getEnumConstants()) {
      StateConfig config = configs.get(s);
      if (config == null) continue;
      for (Transition t : config.allTransitions()) {
        appendEdge(sb, s.name(), t);
      }
    }
    if (!securityTransitions.isEmpty() || !globalTransitions.isEmpty()) {
      sb.append("  state \"any state\" as ").append(ANY_STATE).append('\n');
      for (Transition t : securityTransitions) {
        appendEdge(sb, ANY_STATE, t);
      }
      for (Transition t : globalTransitions) {
        appendEdge(sb, ANY_STATE, t);
      }
    }
    return sb.toString();
  }

  private void appendEdge(StringBuilder sb, String from, Transition t) {
    S target = t.target();
    if (target == null) return;
    sb.append("  ").append(from).append(" --> ").append(target.name());
    String label = t.describe();
    if (!label.isEmpty()) sb.append(" : ").append(label);
    sb.append('\n');
  }

  /** Formats seconds without locale surprises (no decimal comma) and without a useless ".0". */
  private static String formatSeconds(double seconds) {
    if (seconds == Math.rint(seconds) && Math.abs(seconds) < 1e9) {
      return Long.toString((long) seconds);
    }
    return Double.toString(seconds);
  }

  // ---------------------------------------------------------------------------------------------
  // State configuration
  // ---------------------------------------------------------------------------------------------

  public class StateConfig implements State<S> {
    private final S self;
    private Runnable onEnter = () -> {};
    private Runnable onUpdate = () -> {};
    private Runnable onExit = () -> {};
    private final List<Transition> security = new ArrayList<>();
    private final List<Transition> transitions = new ArrayList<>();
    private final List<BooleanSupplier> gates = new ArrayList<>();
    private Transition timeout;
    private boolean gatesOpen;
    private double regularMinSeconds = Double.NaN;
    private double regularMaxSeconds = Double.NaN;

    StateConfig(S self) {
      this.self = self;
    }

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

    /** Transitions to {@code targetState} when {@code condition} is true. */
    public StateConfig transitionTo(S targetState, BooleanSupplier condition) {
      transitions.add(new Transition(targetState, condition, 0.0, Kind.NORMAL, "", null));
      return this;
    }

    /**
     * Transitions to {@code targetState} only after {@code condition} has been true for {@code
     * seconds} without interruption. Filters sensors that flicker: a single false reading restarts
     * the count.
     */
    public StateConfig transitionTo(S targetState, BooleanSupplier condition, double seconds) {
      transitions.add(new Transition(targetState, condition, seconds, Kind.NORMAL, "", null));
      return this;
    }

    /**
     * Transitions to {@code targetState} as soon as the state is allowed to leave: on the next
     * cycle, or when every {@code waitFor(...)} of this state was satisfied.
     */
    public StateConfig transitionTo(S targetState) {
      return transitionTo(targetState, () -> true);
    }

    /** Transitions after {@code seconds} seconds in the current state. */
    public StateConfig transitionAfter(double seconds, S targetState) {
      transitions.add(
          new Transition(
              targetState,
              () -> getTimeInState() >= seconds,
              0.0,
              Kind.NORMAL,
              "after " + formatSeconds(seconds) + " s",
              null));
      return this;
    }

    /**
     * Same as {@link #transitionAfter(double, Enum)}: the state lasts at most {@code seconds} and
     * then continues to {@code finalState} as part of the normal flow.
     */
    public StateConfig withTimeout(double seconds, S finalState) {
      return transitionAfter(seconds, finalState);
    }

    /**
     * Fault path: if the machine is still in this state after {@code seconds}, something went
     * wrong, and it goes to {@code securityState}. The event is reported and the request that was
     * being served stays blocked until it is released.
     */
    public StateConfig timeoutAfter(double seconds, S securityState) {
      if (timeout != null) {
        Report.warning("FSM '" + name + "': state " + self + " has two timeouts (using the last).");
      }
      timeout =
          new Transition(
              securityState,
              () -> getTimeInState() >= seconds,
              0.0,
              Kind.TIMEOUT,
              "timeout " + formatSeconds(seconds) + " s",
              null);
      return this;
    }

    /** Same as {@link #timeoutAfter(double, Enum)}, going to the security state of the machine. */
    public StateConfig timeoutAfter(double seconds) {
      return timeoutAfter(seconds, null);
    }

    /**
     * Fault path: while in this state, goes to {@code securityState} as soon as {@code condition}
     * is true. Security conditions are checked before the normal transitions of the state and are
     * not held back by {@code waitFor(...)}.
     */
    public StateConfig transitionToSecurityState(S securityState, BooleanSupplier condition) {
      security.add(new Transition(securityState, condition, 0.0, Kind.SECURITY, "security", null));
      return this;
    }

    /**
     * Same as {@link #transitionToSecurityState(Enum, BooleanSupplier)}, going to the security
     * state of the machine.
     */
    public StateConfig transitionToSecurityState(BooleanSupplier condition) {
      return transitionToSecurityState(null, condition);
    }

    /**
     * Holds the normal transitions of this state until {@code condition} is true. Once it was true
     * the state is free to leave, even if the condition becomes false again. Global transitions (a
     * new request, for example) and fault paths are not held back.
     *
     * <p>Example: {@code fsm.state(KICKING).waitFor(() -> ballLeft).transitionTo(IDLE)}.
     */
    public StateConfig waitFor(BooleanSupplier condition) {
      gates.add(condition);
      return this;
    }

    /** Holds the normal transitions of this state for at least {@code seconds}. */
    public StateConfig waitFor(double seconds) {
      return waitFor(() -> getTimeInState() >= seconds);
    }

    /**
     * Declares how long this state normally lasts. Every visit outside the window is counted in the
     * statistics ({@code Stats/<STATE>/IrregularCount}) and {@link StateMachine#isOvertime()}
     * becomes true while the state runs longer than {@code maxSeconds}. It never changes the flow
     * of the machine: use {@code timeoutAfter} for that.
     */
    public StateConfig regularTime(double minSeconds, double maxSeconds) {
      this.regularMinSeconds = minSeconds;
      this.regularMaxSeconds = maxSeconds;
      return this;
    }

    boolean hasRegularTime() {
      return !Double.isNaN(regularMaxSeconds);
    }

    List<Transition> allTransitions() {
      List<Transition> all = new ArrayList<>(security);
      all.addAll(transitions);
      if (timeout != null) all.add(timeout);
      return all;
    }

    /** Called when the machine enters this state: timers and waits start from zero. */
    void resetRuntime() {
      gatesOpen = false;
      for (int i = 0; i < security.size(); i++) {
        security.get(i).trueSince = Double.NaN;
      }
      for (int i = 0; i < transitions.size(); i++) {
        transitions.get(i).trueSince = Double.NaN;
      }
    }

    private boolean gatesOpen() {
      if (gatesOpen) return true;
      for (int i = 0; i < gates.size(); i++) {
        if (!gates.get(i).getAsBoolean()) return false;
      }
      gatesOpen = true;
      return true;
    }

    Transition findTransition(double now) {
      Transition winner = null;
      for (int i = 0; i < security.size(); i++) {
        winner = considerFaultPath(winner, security.get(i), now);
      }
      final boolean mayLeave = gatesOpen();
      for (int i = 0; i < transitions.size(); i++) {
        winner = consider(winner, transitions.get(i), now, mayLeave);
      }
      if (winner == null && timeout != null) {
        winner = considerFaultPath(null, timeout, now);
      }
      return winner;
    }

    /** Like {@code consider}, but reports a fault path whose security state was never defined. */
    private Transition considerFaultPath(Transition winner, Transition t, double now) {
      if (t.target() == null) {
        if (t.poll(now)) reportMissingSecurityState();
        return winner;
      }
      return consider(winner, t, now, true);
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
      Transition t = findTransition(RobotClock.now());
      return t != null ? t.target() : null;
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Internals
  // ---------------------------------------------------------------------------------------------

  private final class Transition {
    /** Null means "the security state of the machine", resolved when the transition fires. */
    final S explicitTarget;

    final BooleanSupplier condition;
    final double debounceSeconds;
    final Kind kind;
    final String label;

    /** Request transitions only: states from which the request is already being served. */
    final Set<S> satisfied;

    boolean blockedByFault;
    double trueSince = Double.NaN;

    Transition(
        S explicitTarget,
        BooleanSupplier condition,
        double debounceSeconds,
        Kind kind,
        String label,
        Set<S> satisfied) {
      this.explicitTarget = explicitTarget;
      this.condition = condition;
      this.debounceSeconds = debounceSeconds;
      this.kind = kind;
      this.label = label;
      this.satisfied = satisfied;
    }

    S target() {
      return explicitTarget != null ? explicitTarget : securityState;
    }

    boolean canFireFrom(S state) {
      S target = target();
      return target != null && target != state && (satisfied == null || !satisfied.contains(state));
    }

    /** Reads the condition and advances the debounce timer. True when the transition may fire. */
    boolean poll(double now) {
      boolean active = condition.getAsBoolean();
      if (blockedByFault) {
        if (active) {
          active = false; // still the request that failed: keep it blocked
        } else {
          blockedByFault = false; // released: the next time it is asked, it is served again
          blockedRequests--;
        }
      }
      if (!active) {
        trueSince = Double.NaN;
        return false;
      }
      if (debounceSeconds <= 0.0) return true;
      if (Double.isNaN(trueSince)) trueSince = now;
      return now - trueSince >= debounceSeconds;
    }

    String describe() {
      if (debounceSeconds > 0.0) {
        String held = "held " + formatSeconds(debounceSeconds) + " s";
        return label.isEmpty() ? held : label + ", " + held;
      }
      return label;
    }
  }

  /** Timing statistics of one state. */
  public static final class StateStats {
    private int count;
    private int irregularCount;
    private double lastSeconds;
    private double totalSeconds;
    private double maxSeconds;
    private double minSeconds = Double.POSITIVE_INFINITY;

    StateStats() {}

    /** How many times the machine left this state. */
    public int getCount() {
      return count;
    }

    /** Duration of the last completed visit, in seconds. */
    public double getLastSeconds() {
      return lastSeconds;
    }

    /** Average duration of the completed visits, in seconds (0 before the first one). */
    public double getAverageSeconds() {
      return count == 0 ? 0.0 : totalSeconds / count;
    }

    /** Longest completed visit, in seconds. */
    public double getMaxSeconds() {
      return maxSeconds;
    }

    /** Shortest completed visit, in seconds (0 before the first one). */
    public double getMinSeconds() {
      return count == 0 ? 0.0 : minSeconds;
    }

    /** Visits that ended outside the window declared with {@code regularTime(min, max)}. */
    public int getIrregularCount() {
      return irregularCount;
    }
  }
}
