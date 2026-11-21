package org.ciprix.interfaces.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Testes do padrao Request → State do StateSubsystem: verifica que bindRequest registra as
 * transicoes globais com as exclusoes certas e que atGoal() derivado bate com o mapa declarado. Usa
 * um subsistema fake sem hardware (IO vazio + "sensor" simulado por flag).
 */
class StateSubsystemTest {

  private enum Req {
    RUN,
    STOP
  }

  private enum St {
    RUNNING,
    STOPPING,
    STOPPED
  }

  /** Inputs minimos: LoggableInputs vazio (nada a logar num fake). */
  private static class FakeInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {}

    @Override
    public void fromLog(LogTable table) {}
  }

  private static class FakeIO implements SubsystemIO<FakeInputs> {
    @Override
    public void updateInputs(FakeInputs inputs) {}
  }

  private static class FakeSubsystem extends StateSubsystem<Req, St, FakeInputs, FakeIO> {
    boolean spinStopped = false; // sensor simulado
    int stoppingEnters = 0;
    int runningEnters = 0;

    FakeSubsystem(String name) {
      super(name, new FakeInputs(), new FakeIO(), St.class, St.STOPPED, Req.STOP);

      fsm.state(St.RUNNING).onEnter(() -> runningEnters++);
      fsm.state(St.STOPPING)
          .onEnter(() -> stoppingEnters++)
          .transitionTo(St.STOPPED, () -> spinStopped);
      fsm.state(St.STOPPED);

      bindRequest(Req.RUN, St.RUNNING, St.RUNNING);
      bindRequest(Req.STOP, St.STOPPING, St.STOPPED);

      assertTrue(fsm.validateComplete());
    }

    /** Avanca um ciclo da FSM sem passar pelo scheduler/Logger. */
    void tick() {
      fsm.update();
    }
  }

  @BeforeAll
  static void initHAL() {
    HAL.initialize(500, 0);
  }

  @Test
  void initialRequestIsSatisfiedAtBoot() {
    FakeSubsystem s = new FakeSubsystem("Test/Boot");
    assertEquals(St.STOPPED, s.getState());
    assertTrue(s.atGoal(), "STOP com estado STOPPED deve nascer satisfeito");
  }

  @Test
  void requestDrivesFsmAndAtGoalFollows() {
    FakeSubsystem s = new FakeSubsystem("Test/Flow");

    s.setRequest(Req.RUN);
    assertFalse(s.atGoal(), "request mudou, estado ainda nao");
    s.tick();
    assertEquals(St.RUNNING, s.getState());
    assertTrue(s.atGoal());

    s.setRequest(Req.STOP);
    s.tick();
    assertEquals(St.STOPPING, s.getState());
    assertFalse(s.atGoal(), "STOPPING e intermediario: goal e STOPPED");

    s.spinStopped = true;
    s.tick();
    assertEquals(St.STOPPED, s.getState());
    assertTrue(s.atGoal());
  }

  @Test
  void intermediateStateIsNotReentered() {
    FakeSubsystem s = new FakeSubsystem("Test/NoReentry");

    s.setRequest(Req.RUN);
    s.tick(); // STOPPED -> RUNNING
    s.setRequest(Req.STOP);
    // spinStopped continua false: o mecanismo "ainda gira" por varios ciclos
    s.tick(); // RUNNING -> STOPPING
    s.tick();
    s.tick();

    assertEquals(St.STOPPING, s.getState());
    assertEquals(1, s.stoppingEnters, "a transicao global nao pode re-executar o onEnter");
  }

  @Test
  void switchingRequestMidTravelRedirectsFsm() {
    FakeSubsystem s = new FakeSubsystem("Test/Switch");

    s.setRequest(Req.RUN);
    s.tick(); // STOPPED -> RUNNING
    s.setRequest(Req.STOP);
    s.tick(); // RUNNING -> STOPPING (spinStopped=false segura aqui)
    assertEquals(St.STOPPING, s.getState());

    s.setRequest(Req.RUN); // piloto muda de ideia no meio do caminho
    s.tick();
    assertEquals(St.RUNNING, s.getState());
    assertTrue(s.atGoal());
    assertEquals(2, s.runningEnters);
  }

  // ---------------------------------------------------------------------------------------------
  // Override transitorio (backup do copiloto)
  // ---------------------------------------------------------------------------------------------

  @Test
  void overrideWinsForOneCycleAndTheRequestComesBackByItself() {
    FakeSubsystem s = new FakeSubsystem("Test/Override");
    s.periodic();
    assertEquals(St.STOPPED, s.getState());

    s.override(Req.RUN); // copiloto apertou o botao
    assertTrue(s.isOverrideActive());
    assertEquals(Req.RUN, s.getActiveRequest());
    assertEquals(Req.STOP, s.getRequest(), "o request da SuperStructure continua guardado");

    s.periodic();
    assertEquals(St.RUNNING, s.getState());
    assertTrue(s.isOverrideActive());
    assertEquals(Req.RUN, s.getActiveRequest());

    s.override(Req.RUN); // continua segurando
    s.periodic();
    assertEquals(St.RUNNING, s.getState());
    assertEquals(1, s.runningEnters, "manter o override nao re-executa o onEnter");

    s.periodic(); // soltou: nenhum override() neste ciclo
    assertFalse(s.isOverrideActive());
    assertEquals(Req.STOP, s.getActiveRequest());
    assertEquals(St.STOPPING, s.getState(), "volta sozinho para o request da SuperStructure");
  }

  @Test
  void atGoalStaysRelativeToTheSuperStructureRequestDuringOverride() {
    FakeSubsystem s = new FakeSubsystem("Test/OverrideGoal");
    s.periodic();
    assertTrue(s.atGoal());

    s.override(Req.RUN);
    s.periodic();
    assertEquals(St.RUNNING, s.getState());
    assertFalse(s.atGoal(), "RUNNING atende o override, nao o STOP pedido pela SuperStructure");
  }

  @Test
  void relayKeepsUpdatingTheRequestWhileTheOverrideIsHeld() {
    FakeSubsystem s = new FakeSubsystem("Test/OverrideRelay");
    s.spinStopped = true;

    // O relay continua rodando durante o override: o request muda por baixo sem afetar a FSM.
    s.setRequest(Req.RUN);
    s.override(Req.STOP);
    s.periodic();
    assertEquals(St.STOPPED, s.getState(), "STOPPED ja atende o override STOP");
    assertEquals(Req.RUN, s.getRequest());

    s.periodic(); // soltou o botao: passa a valer o RUN que o relay deixou guardado
    assertEquals(St.RUNNING, s.getState());
    assertTrue(s.atGoal());
  }

  @Test
  void nullRequestsAreIgnoredAndReportedOnce() {
    java.util.List<String> reports = new java.util.ArrayList<>();
    org.ciprix.util.Report.setSink((isError, message, printTrace) -> reports.add(message));
    try {
      FakeSubsystem s = new FakeSubsystem("Test/Null");
      s.setRequest(Req.RUN);
      s.setRequest(null);
      s.override(null);
      s.setRequest(null);

      assertEquals(Req.RUN, s.getRequest());
      assertFalse(s.isOverrideActive());
      assertEquals(1, reports.size());
      s.periodic(); // nao pode lancar excecao
      assertEquals(St.RUNNING, s.getState());
    } finally {
      org.ciprix.util.Report.resetSink();
    }
  }

  @Test
  void inStateTriggerAndStatusFollowTheSubsystem() {
    FakeSubsystem s = new FakeSubsystem("Test/Trigger");
    var running = s.inState(St.RUNNING);
    var stopped = s.inState(St.STOPPED, St.STOPPING);

    s.periodic();
    assertFalse(running.getAsBoolean());
    assertTrue(stopped.getAsBoolean());
    assertEquals("STOP | STOPPED", s.getStatus());

    s.override(Req.RUN);
    s.periodic();
    assertTrue(running.getAsBoolean());
    assertFalse(stopped.getAsBoolean());
    assertEquals("RUN | RUNNING | OVERRIDE", s.getStatus());
  }
}
