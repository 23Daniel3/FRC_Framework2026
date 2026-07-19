package frc.lib.interfaces.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Testes do padrao Request → State do StateSubsystem: verifica que bindRequest registra as
 * transicoes globais com as exclusoes certas e que atGoal() derivado bate com o mapa declarado.
 * Usa um subsistema fake sem hardware (IO vazio + "sensor" simulado por flag).
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

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0));
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
}
