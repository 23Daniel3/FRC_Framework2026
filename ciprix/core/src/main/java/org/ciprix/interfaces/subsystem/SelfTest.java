package org.ciprix.interfaces.subsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.ciprix.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import org.ciprix.interfaces.motor.basic.BasicMotorController;
import org.ciprix.interfaces.motor.basic.BasicMotorIO.BasicMotorIOInputs;
import org.ciprix.util.PersistentAlert;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;
import org.littletonrobotics.junction.Logger;

/**
 * Self-test of a subsystem, for the pits: drives each motor for a moment and checks that it
 * answers, draws a sensible current and really turns. It finds a disconnected motor or wire, a
 * broken coupler, a dead encoder and a binding mechanism before the match does.
 *
 * <p>Declared once in the subsystem constructor, next to the state machine:
 *
 * <pre>{@code
 * selfTest()
 *     .motor("Roller", io::controlRollerMotor, () -> inputs.rollerMotorInputs)
 *     .output(0.3)
 *     .forSeconds(1.0)
 *     .currentBetween(Amps.of(0.5), Amps.of(30))
 *     .mustTurn(RotationsPerSecond.of(5));
 * }</pre>
 *
 * <p>and started from the dashboard or a button (the robot must be enabled):
 *
 * <pre>{@code
 * SmartDashboard.putData("SelfTest/Intake", intake.selfTestCommand());
 * }</pre>
 *
 * <p>While it runs, the state machine of the subsystem is paused, so nothing fights the test; when
 * it ends the machine starts again from its initial state. Each result is published under {@code
 * SelfTest/<subsystem>/<motor>} and a failure raises an alert on the dashboard. Only check
 * mechanisms that can turn freely for a moment: an arm or an elevator would hit its hard stop.
 */
public final class SelfTest {

  /** Fraction of the check spent getting up to speed before measuring. */
  private static final double SETTLE_FRACTION = 0.4;

  /** One motor to check. Every setting has a default that suits a light roller. */
  public static final class MotorCheck {
    private final String label;
    private final Supplier<? extends BasicMotorController> controller;
    private final Supplier<? extends BasicMotorIOInputs> inputs;
    private double output = 0.2;
    private double seconds = 1.0;
    private double minAmps = 0.2;
    private double maxAmps = 40.0;
    private double minSpeedRps = 0.0;

    private MotorCheck(
        String label,
        Supplier<? extends BasicMotorController> controller,
        Supplier<? extends BasicMotorIOInputs> inputs) {
      this.label = label;
      this.controller = controller;
      this.inputs = inputs;
    }

    /** Percent output applied during the check, in [-1, 1]. Default 0.2. */
    public MotorCheck output(double percent) {
      this.output = Math.max(-1.0, Math.min(1.0, percent));
      return this;
    }

    /** How long the motor is driven. Default 1 s. */
    public MotorCheck forSeconds(double seconds) {
      this.seconds = Math.max(0.1, seconds);
      return this;
    }

    /**
     * Range of the average current once the motor is up to speed. Too little means a motor or a
     * wire that is not connected; too much means a binding mechanism. Default 0.2 A to 40 A.
     */
    public MotorCheck currentBetween(Current min, Current max) {
      this.minAmps = Math.abs(min.in(Amps));
      this.maxAmps = Math.abs(max.in(Amps));
      return this;
    }

    /**
     * Speed the mechanism must reach (only for motors with a sensor). A motor that draws current
     * but does not turn has a broken coupler, a slipping belt or a dead encoder. Not checked by
     * default.
     */
    public MotorCheck mustTurn(AngularVelocity minSpeed) {
      this.minSpeedRps = Math.abs(minSpeed.in(RotationsPerSecond));
      return this;
    }
  }

  /** Result of one check. */
  public record Result(String motor, boolean passed, String detail) {}

  private final String displayName;
  private final String logPrefix;
  private final List<MotorCheck> checks = new ArrayList<>();
  private final List<Result> results = new ArrayList<>();
  private PersistentAlert failedAlert = null;
  private PersistentAlert passedAlert = null;

  /**
   * @param subsystemName name of the subsystem, as in its log keys (for example {@code
   *     "Subsystems/Intake"}); the last part is used in the alerts
   */
  public SelfTest(String subsystemName) {
    int slash = subsystemName.lastIndexOf('/');
    this.displayName = slash >= 0 ? subsystemName.substring(slash + 1) : subsystemName;
    this.logPrefix = "SelfTest/" + displayName + "/";
  }

  /**
   * Adds a motor to the test. The suppliers are read while the test runs, so they can point at the
   * IO and the inputs of the subsystem.
   *
   * @param label short name shown in the results, for example {@code "Roller"}
   * @param controller the controller of the motor, usually {@code io::controlRollerMotor}
   * @param inputs the inputs of the motor, usually {@code () -> inputs.rollerMotorInputs}
   */
  public MotorCheck motor(
      String label,
      Supplier<? extends BasicMotorController> controller,
      Supplier<? extends BasicMotorIOInputs> inputs) {
    MotorCheck check = new MotorCheck(label, controller, inputs);
    checks.add(check);
    return check;
  }

  /** True when no motor was declared. */
  public boolean isEmpty() {
    return checks.isEmpty();
  }

  /** Results of the last run, in the order the motors were declared. */
  public List<Result> getResults() {
    return Collections.unmodifiableList(results);
  }

  /** True when the last run finished and every motor passed. */
  public boolean passed() {
    if (results.isEmpty() || results.size() < checks.size()) {
      return false;
    }
    for (Result result : results) {
      if (!result.passed()) {
        return false;
      }
    }
    return true;
  }

  /**
   * The command that runs the test, one motor after the other.
   *
   * @param requirement the subsystem (interrupts its default command while the test runs); may be
   *     null in unit tests
   * @param running told {@code true} when the test starts and {@code false} when it ends, so the
   *     subsystem can pause its state machine
   */
  public Command command(Subsystem requirement, Consumer<Boolean> running) {
    Command command = new Runner(running);
    if (requirement != null) {
      command.addRequirements(requirement);
    }
    return command;
  }

  /** Judges one motor from what was measured once it was up to speed. */
  static Result evaluate(
      MotorCheck check,
      boolean alwaysConnected,
      int samples,
      double averageAmps,
      double fastestRps,
      boolean hasSensor) {
    if (!alwaysConnected) {
      return new Result(check.label, false, "not answering (CAN or power)");
    }
    if (samples == 0) {
      return new Result(check.label, false, "no measurement");
    }
    if (averageAmps < check.minAmps) {
      return new Result(
          check.label,
          false,
          format(
              "%.1f A, expected at least %.1f A: motor or wire disconnected?",
              averageAmps, check.minAmps));
    }
    if (averageAmps > check.maxAmps) {
      return new Result(
          check.label,
          false,
          format(
              "%.1f A, expected at most %.1f A: mechanism binding?", averageAmps, check.maxAmps));
    }
    if (hasSensor && check.minSpeedRps > 0.0 && fastestRps < check.minSpeedRps) {
      return new Result(
          check.label,
          false,
          format(
              "turned at %.1f rps, expected %.1f rps: broken coupler or encoder?",
              fastestRps, check.minSpeedRps));
    }
    return new Result(
        check.label,
        true,
        hasSensor
            ? format("ok: %.1f A, %.1f rps", averageAmps, fastestRps)
            : format("ok: %.1f A", averageAmps));
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }

  private void publish(boolean interrupted) {
    StringBuilder failures = new StringBuilder();
    for (Result result : results) {
      Logger.recordOutput(logPrefix + result.motor(), result.detail());
      if (!result.passed()) {
        if (failures.length() > 0) {
          failures.append(" | ");
        }
        failures.append(result.motor()).append(": ").append(result.detail());
        Report.warning("Self-test " + displayName + "/" + result.motor() + ": " + result.detail());
      }
    }
    boolean allPassed = !interrupted && passed();
    Logger.recordOutput(logPrefix + "Passed", allPassed);

    if (failedAlert == null) {
      failedAlert = new PersistentAlert("Self-test", PersistentAlert.Level.ERROR);
      passedAlert = new PersistentAlert("Self-test", PersistentAlert.Level.INFO);
    }
    if (failures.length() > 0) {
      failedAlert.setText("Self-test failed: " + displayName + ": " + failures);
    }
    failedAlert.set(failures.length() > 0);
    passedAlert.setText("Self-test passed: " + displayName);
    passedAlert.set(allPassed);
  }

  /** Drives the motors one after the other and measures them. */
  private final class Runner extends Command {
    private final Consumer<Boolean> running;

    private int index;
    private double checkStart;
    private boolean finished;

    // Measurements of the motor being checked.
    private int samples;
    private double currentSum;
    private double fastestRps;
    private boolean alwaysConnected;
    private boolean hasSensor;

    Runner(Consumer<Boolean> running) {
      this.running = running == null ? value -> {} : running;
      setName("SelfTest " + displayName);
    }

    @Override
    public void initialize() {
      results.clear();
      index = 0;
      finished = checks.isEmpty();
      running.accept(true);
      if (!finished) {
        startCheck();
      }
    }

    private void startCheck() {
      checkStart = RobotClock.now();
      samples = 0;
      currentSum = 0.0;
      fastestRps = 0.0;
      alwaysConnected = true;
      hasSensor = false;
    }

    @Override
    public void execute() {
      if (finished) {
        return;
      }
      MotorCheck check = checks.get(index);
      double elapsed = RobotClock.now() - checkStart;
      // Every cycle: the motor layer only talks to the hardware when the command changes.
      check.controller.get().runPercentOutput(check.output);

      if (elapsed >= check.seconds * SETTLE_FRACTION) {
        BasicMotorIOInputs measured = check.inputs.get();
        samples++;
        currentSum += Math.abs(measured.current.in(Amps));
        alwaysConnected &= measured.isConnected;
        if (measured instanceof MotorIOInputs withSensor) {
          hasSensor = true;
          fastestRps = Math.max(fastestRps, Math.abs(withSensor.velocity.in(RotationsPerSecond)));
        }
      }

      if (elapsed >= check.seconds) {
        check.controller.get().stop();
        double average = samples > 0 ? currentSum / samples : 0.0;
        results.add(evaluate(check, alwaysConnected, samples, average, fastestRps, hasSensor));
        index++;
        if (index >= checks.size()) {
          finished = true;
        } else {
          startCheck();
        }
      }
    }

    @Override
    public boolean isFinished() {
      return finished;
    }

    @Override
    public void end(boolean interrupted) {
      for (MotorCheck check : checks) {
        check.controller.get().stop();
      }
      publish(interrupted || !finished);
      running.accept(false);
    }
  }
}
