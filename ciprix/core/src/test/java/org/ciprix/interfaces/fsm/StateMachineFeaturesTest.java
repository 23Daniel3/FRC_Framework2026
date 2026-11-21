package org.ciprix.interfaces.fsm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests of the timing and safety features of the FSM: debounced transitions, timeouts, the security
 * state with its request latch, waitFor, statistics and the diagram.
 *
 * <p>They use a manual clock ({@link RobotClock#setSource}) and capture the reports ({@link
 * Report#setSink}), so they are pure Java: no HAL, no robot.
 */
class StateMachineFeaturesTest {

  private enum S {
    IDLE,
    MOVING,
    DONE,
    SAFE
  }

  private double now;
  private final List<String> reports = new ArrayList<>();

  @BeforeEach
  void setup() {
    now = 100.0;
    RobotClock.setSource(() -> now);
    reports.clear();
    Report.setSink((isError, message, printTrace) -> reports.add(message));
  }

  @AfterEach
  void teardown() {
    RobotClock.resetSource();
    Report.resetSink();
  }

  /**
   * Runs {@code cycles} loops of 20 ms. The thresholds used in the tests sit between two cycles (90
   * ms, 190 ms) on purpose, so floating point rounding of the clock cannot move the cycle in which
   * they expire.
   */
  private void run(StateMachine<S> fsm, int cycles) {
    for (int i = 0; i < cycles; i++) {
      now += 0.02;
      fsm.update();
    }
  }

  private StateMachine<S> machine() {
    return new StateMachine<>("test", S.class, S.IDLE);
  }

  // -------------------------------------------------------------------------------------------
  // Debounced transitions
  // -------------------------------------------------------------------------------------------

  @Test
  void debouncedTransitionNeedsUninterruptedTruth() {
    StateMachine<S> fsm = machine();
    boolean[] sensor = {false};
    fsm.state(S.IDLE).transitionTo(S.MOVING, () -> sensor[0], 0.09);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    sensor[0] = true;
    run(fsm, 4); // 60 ms true (the first reading starts the count)
    assertEquals(S.IDLE, fsm.getCurrentState());

    sensor[0] = false; // one flicker restarts the count
    run(fsm, 1);
    sensor[0] = true;
    run(fsm, 5); // 80 ms true again
    assertEquals(S.IDLE, fsm.getCurrentState(), "the flicker must restart the count");

    run(fsm, 1); // 100 ms
    assertEquals(S.MOVING, fsm.getCurrentState());
  }

  @Test
  void debounceTimerRestartsWhenTheStateIsEnteredAgain() {
    StateMachine<S> fsm = machine();
    boolean[] sensor = {true};
    boolean[] back = {false};
    fsm.state(S.IDLE).transitionTo(S.MOVING, () -> sensor[0], 0.09);
    fsm.state(S.MOVING).transitionTo(S.IDLE, () -> back[0]);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    run(fsm, 6);
    assertEquals(S.MOVING, fsm.getCurrentState());

    back[0] = true;
    run(fsm, 1);
    back[0] = false;
    assertEquals(S.IDLE, fsm.getCurrentState());
    run(fsm, 3);
    assertEquals(S.IDLE, fsm.getCurrentState(), "the count starts from zero on every visit");
    run(fsm, 3);
    assertEquals(S.MOVING, fsm.getCurrentState());
  }

  @Test
  void debouncedGlobalTransitionKeepsCountingAcrossStates() {
    StateMachine<S> fsm = machine();
    boolean[] stop = {false};
    boolean[] go = {false};
    fsm.state(S.IDLE).transitionTo(S.MOVING, () -> go[0]);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);
    fsm.addGlobalTransition(S.DONE, () -> stop[0], 0.09);

    stop[0] = true;
    run(fsm, 3);
    go[0] = true;
    run(fsm, 1); // IDLE -> MOVING while the global is still counting
    assertEquals(S.MOVING, fsm.getCurrentState());
    run(fsm, 2);
    assertEquals(S.DONE, fsm.getCurrentState(), "100 ms of truth in total, across two states");
  }

  // -------------------------------------------------------------------------------------------
  // Normal timed flow
  // -------------------------------------------------------------------------------------------

  @Test
  void withTimeoutIsNormalFlowAndNotAFault() {
    StateMachine<S> fsm = machine();
    fsm.state(S.IDLE).withTimeout(0.1, S.DONE);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    run(fsm, 4);
    assertEquals(S.IDLE, fsm.getCurrentState());
    run(fsm, 2);
    assertEquals(S.DONE, fsm.getCurrentState());
    assertFalse(fsm.hasFault());
    assertEquals(0, fsm.getFaultCount());
    assertTrue(reports.isEmpty());
  }

  // -------------------------------------------------------------------------------------------
  // Timeout and security state
  // -------------------------------------------------------------------------------------------

  /** Machine in the usual subsystem shape: one request served through MOVING until DONE. */
  private StateMachine<S> requestMachine(BooleanSupplier go, BooleanSupplier arrived) {
    StateMachine<S> fsm = machine();
    fsm.securityState(S.SAFE);
    fsm.state(S.IDLE);
    fsm.state(S.MOVING).transitionTo(S.DONE, arrived).timeoutAfter(0.19);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);
    fsm.addRequestTransition("GO", go, S.MOVING, S.DONE);
    fsm.addRequestTransition("STOP", () -> !go.getAsBoolean(), S.IDLE);
    return fsm;
  }

  @Test
  void timeoutSendsTheMachineToTheSecurityStateAndReportsIt() {
    boolean[] go = {true};
    StateMachine<S> fsm = requestMachine(() -> go[0], () -> false);

    run(fsm, 1);
    assertEquals(S.MOVING, fsm.getCurrentState());
    run(fsm, 9);
    assertEquals(S.MOVING, fsm.getCurrentState(), "180 ms: not yet");
    run(fsm, 1);
    assertEquals(S.SAFE, fsm.getCurrentState());
    assertTrue(fsm.hasFault());
    assertEquals(1, fsm.getFaultCount());
    assertEquals("MOVING -> SAFE (timeout 0.19 s)", fsm.getLastFault());
    assertEquals(1, reports.size());
  }

  @Test
  void failedRequestStaysBlockedUntilItIsReleased() {
    boolean[] go = {true};
    StateMachine<S> fsm = requestMachine(() -> go[0], () -> false);

    run(fsm, 11); // IDLE -> MOVING -> (timeout) SAFE
    assertEquals(S.SAFE, fsm.getCurrentState());

    run(fsm, 50); // the request is still held: no bouncing back into MOVING
    assertEquals(S.SAFE, fsm.getCurrentState());
    assertEquals(1, fsm.getFaultCount(), "exactly one attempt");

    go[0] = false; // released: STOP is served normally and the fault ends
    run(fsm, 1);
    assertEquals(S.IDLE, fsm.getCurrentState());
    assertFalse(fsm.hasFault());

    go[0] = true; // asked again: a new attempt is allowed
    run(fsm, 1);
    assertEquals(S.MOVING, fsm.getCurrentState());
  }

  @Test
  void normalTransitionWinsOverTheTimeoutOnTheSameCycle() {
    boolean[] go = {true};
    boolean[] arrived = {false};
    StateMachine<S> fsm = requestMachine(() -> go[0], () -> arrived[0]);

    run(fsm, 10);
    arrived[0] = true; // arrives exactly when the timeout expires
    run(fsm, 1);
    assertEquals(S.DONE, fsm.getCurrentState());
    assertFalse(fsm.hasFault());
  }

  @Test
  void requestThatWasNotBeingServedIsNotBlocked() {
    StateMachine<S> fsm = machine();
    boolean[] go = {false};
    fsm.state(S.IDLE);
    // MOVING is entered by hand and is not part of the GO request path.
    fsm.state(S.MOVING).timeoutAfter(0.1, S.SAFE);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);
    fsm.addRequestTransition("GO", () -> go[0], S.DONE);

    fsm.forceState(S.MOVING);
    go[0] = false;
    run(fsm, 6);
    assertEquals(S.SAFE, fsm.getCurrentState());
    assertTrue(fsm.hasFault(), "no request involved: the fault lasts while in the security state");

    go[0] = true;
    run(fsm, 1);
    assertEquals(S.DONE, fsm.getCurrentState());
    assertFalse(fsm.hasFault());
  }

  @Test
  void clearFaultAcceptsTheRequestAgain() {
    boolean[] go = {true};
    StateMachine<S> fsm = requestMachine(() -> go[0], () -> false);

    run(fsm, 11);
    assertEquals(S.SAFE, fsm.getCurrentState());
    fsm.clearFault();
    assertFalse(fsm.hasFault());
    run(fsm, 1);
    assertEquals(S.MOVING, fsm.getCurrentState());
  }

  @Test
  void stateSecurityConditionWinsOverNormalTransitions() {
    StateMachine<S> fsm = machine();
    boolean[] jammed = {false};
    fsm.state(S.IDLE)
        .transitionToSecurityState(S.SAFE, () -> jammed[0])
        .waitFor(() -> false) // even a closed waitFor does not hold a fault path back
        .transitionTo(S.DONE, () -> true);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    run(fsm, 3);
    assertEquals(S.IDLE, fsm.getCurrentState());
    jammed[0] = true;
    run(fsm, 1);
    assertEquals(S.SAFE, fsm.getCurrentState());
    assertTrue(fsm.hasFault());
    assertEquals("IDLE -> SAFE (security)", fsm.getLastFault());
  }

  @Test
  void machineSecurityConditionHoldsTheMachineInTheSecurityState() {
    boolean[] go = {true};
    boolean[] problem = {false};
    StateMachine<S> fsm = requestMachine(() -> go[0], () -> false);
    fsm.addSecurityTransition(() -> problem[0]);

    run(fsm, 1);
    assertEquals(S.MOVING, fsm.getCurrentState());

    problem[0] = true;
    run(fsm, 1);
    assertEquals(S.SAFE, fsm.getCurrentState());

    go[0] = false; // even a new request cannot take the machine out while the problem persists
    run(fsm, 5);
    assertEquals(S.SAFE, fsm.getCurrentState());
    assertTrue(fsm.hasFault());

    problem[0] = false;
    run(fsm, 1);
    assertEquals(S.IDLE, fsm.getCurrentState());
    assertFalse(fsm.hasFault());
  }

  @Test
  void faultPathWithoutSecurityStateIsReportedInsteadOfThrowing() {
    StateMachine<S> fsm = machine();
    fsm.state(S.IDLE).timeoutAfter(0.05);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    assertFalse(fsm.validateComplete(), "the missing security state is a configuration error");
    reports.clear();

    run(fsm, 10);
    assertEquals(S.IDLE, fsm.getCurrentState());
    assertEquals(1, reports.size(), "reported once, not every cycle");
  }

  @Test
  void forceStateRemovesTheBlock() {
    boolean[] go = {true};
    StateMachine<S> fsm = requestMachine(() -> go[0], () -> false);

    run(fsm, 11);
    assertEquals(S.SAFE, fsm.getCurrentState());
    fsm.forceState(S.IDLE);
    assertFalse(fsm.hasFault());
    run(fsm, 1);
    assertEquals(S.MOVING, fsm.getCurrentState());
  }

  // -------------------------------------------------------------------------------------------
  // waitFor
  // -------------------------------------------------------------------------------------------

  @Test
  void waitForHoldsTheStateUntilTheConditionWasTrueOnce() {
    StateMachine<S> fsm = machine();
    boolean[] ready = {false};
    fsm.state(S.IDLE).waitFor(() -> ready[0]).transitionTo(S.DONE);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    run(fsm, 5);
    assertEquals(S.IDLE, fsm.getCurrentState());
    ready[0] = true;
    run(fsm, 1);
    assertEquals(S.DONE, fsm.getCurrentState());
  }

  @Test
  void waitForStaysOpenAfterItWasSatisfied() {
    StateMachine<S> fsm = machine();
    boolean[] ready = {false};
    boolean[] leave = {false};
    fsm.state(S.IDLE).waitFor(() -> ready[0]).transitionTo(S.DONE, () -> leave[0]);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    ready[0] = true;
    run(fsm, 1);
    ready[0] = false; // the wait was already satisfied
    leave[0] = true;
    run(fsm, 1);
    assertEquals(S.DONE, fsm.getCurrentState());
  }

  @Test
  void waitForSecondsIsAMinimumTimeInTheState() {
    StateMachine<S> fsm = machine();
    fsm.state(S.IDLE).waitFor(0.1).transitionTo(S.DONE, () -> true);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    run(fsm, 4);
    assertEquals(S.IDLE, fsm.getCurrentState());
    run(fsm, 2);
    assertEquals(S.DONE, fsm.getCurrentState());
  }

  @Test
  void waitForDoesNotHoldGlobalTransitions() {
    StateMachine<S> fsm = machine();
    boolean[] request = {false};
    fsm.state(S.IDLE).waitFor(() -> false).transitionTo(S.DONE);
    fsm.state(S.MOVING);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);
    fsm.addRequestTransition(() -> request[0], S.MOVING);

    run(fsm, 3);
    assertEquals(S.IDLE, fsm.getCurrentState());
    request[0] = true;
    run(fsm, 1);
    assertEquals(S.MOVING, fsm.getCurrentState(), "a new request always gets through");
  }

  // -------------------------------------------------------------------------------------------
  // Statistics
  // -------------------------------------------------------------------------------------------

  @Test
  void statisticsTrackTheTimeSpentInEachState() {
    StateMachine<S> fsm = machine();
    boolean[] toggle = {false};
    fsm.state(S.IDLE).transitionTo(S.MOVING, () -> toggle[0]);
    fsm.state(S.MOVING).transitionTo(S.IDLE, () -> !toggle[0]);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    toggle[0] = true;
    run(fsm, 1); // leaves IDLE after 20 ms
    run(fsm, 4);
    toggle[0] = false;
    run(fsm, 1); // leaves MOVING after 100 ms
    toggle[0] = true;
    run(fsm, 1);
    run(fsm, 9);
    toggle[0] = false;
    run(fsm, 1); // leaves MOVING after 200 ms

    StateMachine.StateStats moving = fsm.getStats(S.MOVING);
    assertEquals(2, moving.getCount());
    assertEquals(0.2, moving.getLastSeconds(), 1e-9);
    assertEquals(0.15, moving.getAverageSeconds(), 1e-9);
    assertEquals(0.2, moving.getMaxSeconds(), 1e-9);
    assertEquals(0.1, moving.getMinSeconds(), 1e-9);
    assertEquals(4, fsm.getTransitionCount());
    assertEquals(S.MOVING, fsm.getPreviousState());
  }

  @Test
  void regularTimeCountsVisitsOutsideTheWindowWithoutChangingTheFlow() {
    StateMachine<S> fsm = machine();
    boolean[] leave = {false};
    fsm.state(S.IDLE).regularTime(0.05, 0.1).transitionTo(S.MOVING, () -> leave[0]);
    fsm.state(S.MOVING).transitionTo(S.IDLE);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    run(fsm, 4);
    assertFalse(fsm.isOvertime());
    run(fsm, 3);
    assertTrue(fsm.isOvertime(), "140 ms in a state that normally takes at most 100 ms");
    assertEquals(S.IDLE, fsm.getCurrentState(), "regularTime never forces a transition");

    leave[0] = true;
    run(fsm, 1); // too long: irregular
    leave[0] = false;
    run(fsm, 1); // back to IDLE
    run(fsm, 3);
    leave[0] = true;
    run(fsm, 1); // 80 ms: regular
    leave[0] = false;
    run(fsm, 1); // back to IDLE
    leave[0] = true;
    run(fsm, 1); // 20 ms: too short, irregular

    assertEquals(3, fsm.getStats(S.IDLE).getCount());
    assertEquals(2, fsm.getStats(S.IDLE).getIrregularCount());
  }

  // -------------------------------------------------------------------------------------------
  // Queries, initial state and diagram
  // -------------------------------------------------------------------------------------------

  @Test
  void inStateFollowsTheMachine() {
    StateMachine<S> fsm = machine();
    fsm.state(S.IDLE).transitionTo(S.MOVING);
    fsm.state(S.MOVING).transitionTo(S.DONE);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);
    BooleanSupplier busy = fsm.inState(S.MOVING, S.DONE);

    assertFalse(busy.getAsBoolean());
    assertTrue(fsm.isIn(S.IDLE));
    run(fsm, 1);
    assertTrue(busy.getAsBoolean());
    assertTrue(fsm.isIn(S.SAFE, S.MOVING));
    run(fsm, 1);
    assertTrue(busy.getAsBoolean());
    assertFalse(fsm.isIn(S.IDLE, S.MOVING));
  }

  @Test
  void initialOnEnterRunsOnlyWhenAsked() {
    int[] enters = {0};
    StateMachine<S> silent = machine();
    silent.state(S.IDLE).onEnter(() -> enters[0]++);
    run(silent, 3);
    assertEquals(0, enters[0]);

    StateMachine<S> optIn = machine().runInitialOnEnter();
    optIn.state(S.IDLE).onEnter(() -> enters[0]++);
    run(optIn, 3);
    assertEquals(1, enters[0], "exactly once, on the first update");
  }

  @Test
  void diagramListsEveryKindOfTransition() {
    boolean[] go = {true};
    StateMachine<S> fsm = requestMachine(() -> go[0], () -> false);
    fsm.state(S.DONE).transitionAfter(1.5, S.IDLE);

    String diagram = fsm.toMermaid();
    assertTrue(diagram.startsWith("stateDiagram-v2\n  [*] --> IDLE\n"), diagram);
    assertTrue(diagram.contains("  MOVING --> DONE\n"), diagram);
    assertTrue(diagram.contains("  MOVING --> SAFE : timeout 0.19 s\n"), diagram);
    assertTrue(diagram.contains("  DONE --> IDLE : after 1.5 s\n"), diagram);
    assertTrue(diagram.contains("  ANY_STATE --> MOVING : GO\n"), diagram);
    assertTrue(diagram.contains("  ANY_STATE --> IDLE : STOP\n"), diagram);
  }

  @Test
  void validateCompleteWarnsAboutStatesNothingLeadsTo() {
    StateMachine<S> fsm = machine();
    fsm.state(S.IDLE).transitionTo(S.MOVING);
    fsm.state(S.MOVING).transitionTo(S.IDLE);
    fsm.state(S.DONE);
    fsm.state(S.SAFE);

    assertTrue(fsm.validateComplete(), "every state is configured");
    assertEquals(2, reports.size(), "DONE and SAFE are orphans");
    assertTrue(reports.get(0).contains("DONE"));
    assertTrue(reports.get(1).contains("SAFE"));
  }
}
