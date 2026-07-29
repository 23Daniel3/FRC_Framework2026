package frc.lib.interfaces.fsm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Testes do nucleo da FSM — l 100% logica pura, roda sem robo. Protege as garantias que o template
 * promete: ordem de avaliacao, um salto por ciclo, exclusoes das request transitions e unidades de
 * tempo em segundos.
 */
class StateMachineTest {

  private enum S {
    A,
    B,
    C
  }

  @BeforeAll
  static void initHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setup() {
    // HAL simulado: necessario porque a FSM le o timestamp da FPGA (Logger.getTimestamp).
    SimHooks.pauseTiming(); // tempo determinista nos testes de transitionAfter
    SimHooks.restartTiming();
  }

  @AfterEach
  void teardown() {
    SimHooks.resumeTiming();
  }

  @Test
  void localTransitionFiresWhenConditionTrue() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    boolean[] go = {false};
    fsm.state(S.A).transitionTo(S.B, () -> go[0]);
    fsm.state(S.B);
    fsm.state(S.C);

    fsm.update();
    assertEquals(S.A, fsm.getCurrentState());

    go[0] = true;
    fsm.update();
    assertEquals(S.B, fsm.getCurrentState());
  }

  @Test
  void onEnterAndOnExitRunExactlyOncePerTransition() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    int[] enterB = {0};
    int[] exitA = {0};
    fsm.state(S.A).onExit(() -> exitA[0]++).transitionTo(S.B, () -> true);
    fsm.state(S.B).onEnter(() -> enterB[0]++);
    fsm.state(S.C);

    fsm.update(); // A -> B
    fsm.update(); // permanece em B
    fsm.update();

    assertEquals(1, exitA[0]);
    assertEquals(1, enterB[0]);
  }

  @Test
  void globalTransitionPreemptsLocal() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    fsm.state(S.A).transitionTo(S.B, () -> true); // local quer B
    fsm.state(S.B);
    fsm.state(S.C);
    fsm.addGlobalTransition(S.C, () -> true); // global quer C

    fsm.update();
    assertEquals(S.C, fsm.getCurrentState(), "global deve ter prioridade sobre local");
  }

  @Test
  void atMostOneTransitionPerUpdate() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    fsm.state(S.A).transitionTo(S.B, () -> true);
    fsm.state(S.B).transitionTo(S.C, () -> true);
    fsm.state(S.C);

    fsm.update();
    assertEquals(S.B, fsm.getCurrentState(), "cadeias custam um ciclo por salto");
    fsm.update();
    assertEquals(S.C, fsm.getCurrentState());
  }

  @Test
  void requestTransitionSkipsEntryAndSatisfyingStates() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    int[] enterB = {0};
    boolean[] request = {true};
    fsm.state(S.A);
    fsm.state(S.B).onEnter(() -> enterB[0]++).transitionTo(S.C, () -> true);
    fsm.state(S.C);
    // request entra por B; C ja satisfaz o request
    fsm.addRequestTransition(() -> request[0], S.B, S.C);

    fsm.update(); // A -> B (entrada)
    assertEquals(S.B, fsm.getCurrentState());
    fsm.update(); // B -> C via local; a global NAO deve puxar de volta para B
    assertEquals(S.C, fsm.getCurrentState());
    fsm.update();
    fsm.update();
    assertEquals(S.C, fsm.getCurrentState(), "estado que satisfaz o request nao e interrompido");
    assertEquals(1, enterB[0], "sem re-entrada no estado de entrada");
  }

  @Test
  void transitionAfterUsesSeconds() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    fsm.state(S.A).transitionAfter(0.5, S.B);
    fsm.state(S.B);
    fsm.state(S.C);

    fsm.update();
    assertEquals(S.A, fsm.getCurrentState(), "nao pode disparar imediatamente (bug de us vs s)");

    SimHooks.stepTiming(0.6);
    fsm.update();
    assertEquals(S.B, fsm.getCurrentState());
  }

  @Test
  void timeInStateResetsOnTransition() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    fsm.state(S.A).transitionTo(S.B, () -> true);
    fsm.state(S.B);
    fsm.state(S.C);

    SimHooks.stepTiming(2.0);
    assertTrue(fsm.getTimeInState() >= 1.95, "tempo decorrido deve acumular aproximadamente 2.0s");
    fsm.update(); // A -> B
    assertTrue(fsm.getTimeInState() < 0.5, "tempo no estado deve resetar apos transicao");
  }

  @Test
  void validateCompleteDetectsMissingStates() {
    StateMachine<S> incomplete = new StateMachine<>("t1", S.class, S.A);
    incomplete.state(S.A);
    incomplete.state(S.B);
    assertFalse(incomplete.validateComplete(), "S.C nao foi configurado");

    StateMachine<S> complete = new StateMachine<>("t2", S.class, S.A);
    complete.state(S.A);
    complete.state(S.B);
    complete.state(S.C);
    assertTrue(complete.validateComplete());
  }

  @Test
  void forceStateRunsLifecycleCallbacks() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    int[] exitA = {0};
    int[] enterC = {0};
    fsm.state(S.A).onExit(() -> exitA[0]++);
    fsm.state(S.B);
    fsm.state(S.C).onEnter(() -> enterC[0]++);

    fsm.forceState(S.C);
    assertEquals(S.C, fsm.getCurrentState());
    assertEquals(1, exitA[0]);
    assertEquals(1, enterC[0]);
  }

  @Test
  void initialStateOnEnterIsNotCalledAtInitialization() {
    StateMachine<S> fsm = new StateMachine<>("t", S.class, S.A);
    int[] enterA = {0};

    fsm.state(S.A).onEnter(() -> enterA[0]++);
    fsm.state(S.B);
    fsm.state(S.C);

    fsm.update();
    assertEquals(0, enterA[0], "onEnter do estado inicial nao roda automaticamente");

    fsm.forceState(S.B);
    fsm.forceState(S.A);
    assertEquals(1, enterA[0], "onEnter roda quando A e reentrado por transicao");
  }
}
