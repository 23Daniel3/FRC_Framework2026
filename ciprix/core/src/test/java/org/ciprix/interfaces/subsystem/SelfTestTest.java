package org.ciprix.interfaces.subsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.ciprix.TestEnvironment;
import org.ciprix.interfaces.motor.MotorIOSim;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import org.ciprix.sim.SimMechanism;
import org.ciprix.util.PersistentAlert.Level;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The pit self-test must catch what breaks between matches, and nothing else. */
class SelfTestTest {

  private final TestEnvironment env = new TestEnvironment();
  private final MotorIOInputs inputs = new MotorIOInputs();
  private final List<Boolean> runningCalls = new ArrayList<>();

  @BeforeEach
  void setUp() {
    env.install();
  }

  @AfterEach
  void tearDown() {
    env.uninstall();
  }

  /** A roller with a bit of friction, like a real one: it draws about 1 A turning freely. */
  private static SimMechanism roller() {
    return SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001).withFriction(0.06, 0.0);
  }

  private SelfTest declare(MotorIOSim motor) {
    SelfTest test = new SelfTest("Subsystems/Intake");
    test.motor("Roller", motor::getMotorController, () -> inputs)
        .output(0.3)
        .forSeconds(1.0)
        .currentBetween(Amps.of(0.3), Amps.of(30))
        .mustTurn(RotationsPerSecond.of(3));
    return test;
  }

  /** Runs the command the way the scheduler would, after the subsystem read its inputs. */
  private void run(Command command, MotorIOSim motor) {
    command.initialize();
    for (int i = 0; i < 200 && !command.isFinished(); i++) {
      env.cycle();
      motor.updateInputs(inputs);
      command.execute();
    }
    command.end(false);
  }

  @Test
  void healthyMotorPasses() {
    MotorIOSim motor = MotorIOSim.talonFX("Intake/Roller", new MotorConfig(), roller());
    SelfTest test = declare(motor);
    run(test.command(null, runningCalls::add), motor);

    assertTrue(test.passed(), test.getResults().toString());
    assertTrue(test.getResults().get(0).detail().startsWith("ok"));
    assertEquals(List.of(true, false), runningCalls, "pauses and resumes the state machine");
    assertEquals(Level.INFO, env.activeAlerts().get("Self-test passed: Intake"));
    env.cycle();
    motor.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVolts.baseUnitMagnitude(), 1e-9, "stopped at the end");
  }

  @Test
  void disconnectedMotorFails() {
    MotorIOSim motor = MotorIOSim.talonFX("Intake/Roller", new MotorConfig(), roller());
    motor.setConnected(false);
    SelfTest test = declare(motor);
    run(test.command(null, null), motor);

    assertFalse(test.passed());
    assertTrue(test.getResults().get(0).detail().contains("not answering"));
    Map<String, Level> alerts = env.activeAlerts();
    assertTrue(
        alerts.keySet().stream().anyMatch(text -> text.startsWith("Self-test failed: Intake")),
        alerts.toString());
  }

  @Test
  void motorThatDrawsNoCurrentFails() {
    // No friction and no load: like a motor whose phase wire came loose, it draws nothing.
    MotorIOSim motor =
        MotorIOSim.talonFX(
            "Intake/Roller",
            new MotorConfig(),
            SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001));
    SelfTest test = declare(motor);
    run(test.command(null, null), motor);
    assertTrue(test.getResults().get(0).detail().contains("disconnected?"));
  }

  @Test
  void jammedMechanismFails() {
    MotorIOSim motor =
        MotorIOSim.talonFX("Intake/Roller", new MotorConfig(), roller().withLoad(() -> 1e6));
    SelfTest test = declare(motor);
    run(test.command(null, null), motor);
    assertTrue(test.getResults().get(0).detail().contains("binding?"));
  }

  @Test
  void motorThatDoesNotTurnFails() {
    MotorIOSim motor =
        MotorIOSim.talonFX("Intake/Roller", new MotorConfig(), roller().withLoad(() -> 1e6));
    SelfTest test = new SelfTest("Subsystems/Intake");
    test.motor("Roller", motor::getMotorController, () -> inputs)
        .output(0.3)
        .currentBetween(Amps.of(0.3), Amps.of(60))
        .mustTurn(RotationsPerSecond.of(3));
    run(test.command(null, null), motor);
    assertTrue(test.getResults().get(0).detail().contains("coupler"));
  }

  @Test
  void motorsAreCheckedOneAfterTheOther() {
    MotorIOSim first = MotorIOSim.talonFX("Intake/Roller", new MotorConfig(), roller());
    MotorIOSim second = MotorIOSim.talonFX("Intake/Feeder", new MotorConfig(), roller());
    MotorIOInputs secondInputs = new MotorIOInputs();
    SelfTest test = new SelfTest("Subsystems/Intake");
    test.motor("Roller", first::getMotorController, () -> inputs).output(0.3);
    test.motor("Feeder", second::getMotorController, () -> secondInputs).output(0.3);

    Command command = test.command(null, null);
    command.initialize();
    boolean overlap = false;
    for (int i = 0; i < 200 && !command.isFinished(); i++) {
      env.cycle();
      first.updateInputs(inputs);
      second.updateInputs(secondInputs);
      command.execute();
      overlap |=
          Math.abs(inputs.appliedVolts.baseUnitMagnitude()) > 0.1
              && Math.abs(secondInputs.appliedVolts.baseUnitMagnitude()) > 0.1;
    }
    command.end(false);
    assertFalse(overlap, "never both at once");
    assertEquals(2, test.getResults().size());
    assertTrue(test.passed(), test.getResults().toString());
  }

  @Test
  void interruptedTestStopsTheMotorsAndDoesNotPass() {
    MotorIOSim motor = MotorIOSim.talonFX("Intake/Roller", new MotorConfig(), roller());
    SelfTest test = declare(motor);
    Command command = test.command(null, runningCalls::add);
    command.initialize();
    for (int i = 0; i < 10; i++) {
      env.cycle();
      motor.updateInputs(inputs);
      command.execute();
    }
    command.end(true);
    env.cycle();
    motor.updateInputs(inputs);
    assertFalse(test.passed());
    assertEquals(0.0, inputs.appliedVolts.baseUnitMagnitude(), 1e-9);
    assertEquals(List.of(true, false), runningCalls);
  }
}
