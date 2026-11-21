package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.measure.Angle;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.ciprix.TestEnvironment;
import org.ciprix.interfaces.motor.advanced.MotorBase;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import org.ciprix.util.PersistentAlert.Level;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The intent rules every vendor shares: what reaches the hardware, when, and how the automatic
 * un-jam gives the motor back to the robot code.
 */
class MotorBaseTest {

  /** A motor controller that only records what it is told and reports what the test wants. */
  private static final class FakeMotor extends MotorBase {
    final List<String> calls = new ArrayList<>();

    double volts = 0.0;
    double amps = 0.0;
    double velocityRps = 0.0;
    double positionRotations = 0.0;
    boolean connected = true;

    FakeMotor(String name, MotorConfig config) {
      super(name, config);
    }

    String lastCall() {
      return calls.isEmpty() ? null : calls.get(calls.size() - 1);
    }

    /** The units library converts to radians and back: 60 rps may arrive as 59.99999999999999. */
    private static double round(double value) {
      return Math.round(value * 1e6) / 1e6;
    }

    @Override
    protected void applyVoltage(double value) {
      calls.add("voltage " + round(value));
    }

    @Override
    protected void applyPercentOutput(double percent) {
      calls.add("percent " + round(percent));
    }

    @Override
    protected void applyStop() {
      calls.add("stop");
    }

    @Override
    protected void applyVelocity(double rotationsPerSecond, int slot) {
      calls.add("velocity " + round(rotationsPerSecond) + " slot " + slot);
    }

    @Override
    protected void applyPosition(double rotations, int slot) {
      calls.add("position " + round(rotations) + " slot " + slot);
    }

    @Override
    protected void applySmartPosition(double rotations, int slot) {
      calls.add("smart " + round(rotations) + " slot " + slot);
    }

    @Override
    protected void applyBrakeMode(boolean enabled) {
      calls.add("brake " + enabled);
    }

    @Override
    protected void applyCurrentLimit(double limitAmps) {
      calls.add("limit " + limitAmps);
    }

    @Override
    protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
      inputs.appliedVolts = Volts.of(volts);
      inputs.current = Amps.of(amps);
      inputs.temperature = Celsius.of(30.0);
      inputs.isConnected = connected;
    }

    @Override
    protected void updateHardwareInputs(MotorIOInputs inputs) {
      updateHardwareInputs((BasicMotorIOInputs) inputs);
      inputs.velocity = RotationsPerSecond.of(velocityRps);
      inputs.position = Rotations.of(positionRotations);
    }

    @Override
    public void setOffset(Angle offset) {}

    @Override
    public void applyHardwarePID(int slot, double p, double i, double d) {}

    @Override
    public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {}

    @Override
    public void applyHardwareSmartMotion(
        int slot, double maxVel, double maxAccel, double allowedErr) {}

    @Override
    public void applyHardwareOutputRange(int slot, double min, double max) {}
  }

  private final TestEnvironment env = new TestEnvironment();
  private final MotorIOInputs inputs = new MotorIOInputs();

  @BeforeEach
  void setUp() {
    env.install();
  }

  @AfterEach
  void tearDown() {
    env.uninstall();
  }

  /** One robot cycle: library services, then the subsystem reading its motor. */
  private void cycle(FakeMotor motor) {
    env.cycle();
    motor.updateInputs(inputs);
  }

  private void cycles(FakeMotor motor, double seconds) {
    int count = (int) Math.round(seconds / 0.02);
    for (int i = 0; i < count; i++) {
      cycle(motor);
    }
  }

  @Test
  void repeatedCommandReachesTheHardwareOnce() {
    FakeMotor motor = new FakeMotor("Shooter/Leader", new MotorConfig());
    for (int i = 0; i < 10; i++) {
      motor.runVelocity(RotationsPerSecond.of(50.0)); // a state that repeats it every cycle
      cycle(motor);
    }
    assertEquals(List.of("velocity 50.0 slot 0"), motor.calls);
  }

  @Test
  void unchangedCommandIsSentAgainEveryHalfSecond() {
    FakeMotor motor = new FakeMotor("Shooter/Leader", new MotorConfig());
    motor.runVelocity(RotationsPerSecond.of(50.0));
    cycles(motor, 1.1); // sent at 0 s, again at 0.5 s and at 1.0 s
    assertEquals(3, motor.calls.size(), motor.calls.toString());
    assertEquals("velocity 50.0 slot 0", motor.lastCall());
  }

  @Test
  void newValueOrSlotIsSentAtOnce() {
    FakeMotor motor = new FakeMotor("Shooter/Leader", new MotorConfig());
    motor.runVelocity(RotationsPerSecond.of(50.0));
    motor.runVelocity(RotationsPerSecond.of(60.0));
    motor.runVelocity(RotationsPerSecond.of(60.0), 1);
    motor.runPosition(Rotations.of(2.0));
    motor.runSmartPosition(Rotations.of(3.0), 2);
    motor.stop();
    assertEquals(
        List.of(
            "velocity 50.0 slot 0",
            "velocity 60.0 slot 0",
            "velocity 60.0 slot 1",
            "position 2.0 slot 0",
            "smart 3.0 slot 2",
            "stop"),
        motor.calls);
  }

  @Test
  void invalidSlotFallsBackToSlotZeroWithOneReport() {
    FakeMotor motor = new FakeMotor("Shooter/Leader", new MotorConfig());
    motor.runVelocity(RotationsPerSecond.of(10.0), 7);
    motor.runVelocity(RotationsPerSecond.of(20.0), 9);
    assertEquals("velocity 20.0 slot 0", motor.lastCall());
    assertEquals(1, env.reports.size(), env.reports.toString());
  }

  @Test
  void motorThatWasNeverCommandedIsNeverTouched() {
    // A follower: its inputs are read, but the robot code never commands it.
    FakeMotor follower = new FakeMotor("Shooter/Follower", new MotorConfig());
    cycles(follower, 2.0);
    assertTrue(follower.calls.isEmpty(), follower.calls.toString());
  }

  @Test
  void outputRangeScalesOpenLoopCommands() {
    FakeMotor motor = new FakeMotor("Intake/Roller", new MotorConfig().outputRange(-0.5, 0.5));
    motor.runPercentOutput(1.0);
    assertEquals("percent 0.5", motor.lastCall());
    motor.runPercentOutput(-0.5);
    assertEquals("percent -0.25", motor.lastCall());
    motor.runVoltage(Volts.of(12.0));
    assertEquals("voltage 6.0", motor.lastCall());
  }

  @Test
  void openLoopCommandsNeverExceedTheirLimits() {
    FakeMotor motor = new FakeMotor("Intake/Roller", new MotorConfig());
    motor.runPercentOutput(2.0);
    assertEquals("percent 1.0", motor.lastCall());
    motor.runVoltage(Volts.of(30.0));
    assertEquals("voltage 12.0", motor.lastCall(), "never above the nominal voltage");
  }

  @Test
  void brakeModeAndCurrentLimitAreOnlyWrittenWhenTheyChange() {
    FakeMotor motor =
        new FakeMotor("Intake/Pivot", new MotorConfig().brakeMode().currentLimit(Amps.of(40)));
    motor.setBrakeMode(true);
    motor.setCurrentLimit(Amps.of(40));
    assertTrue(motor.calls.isEmpty(), "already configured that way");

    motor.setBrakeMode(false);
    motor.setBrakeMode(false);
    motor.setCurrentLimit(Amps.of(30));
    motor.setCurrentLimit(Amps.of(30));
    assertEquals(List.of("brake false", "limit 30.0"), motor.calls);
  }

  @Test
  void atSetpointUsesTheConfiguredTolerances() {
    FakeMotor motor =
        new FakeMotor(
            "Shooter/Leader",
            new MotorConfig()
                .withVelocityTolerance(RotationsPerSecond.of(2.0))
                .withPositionTolerance(Rotations.of(0.05)));
    motor.runVelocity(RotationsPerSecond.of(50.0));
    motor.velocityRps = 49.0;
    cycle(motor);
    assertTrue(inputs.atSetpoint);
    motor.velocityRps = 45.0;
    cycle(motor);
    assertFalse(inputs.atSetpoint);

    motor.runPosition(Rotations.of(1.0));
    motor.positionRotations = 0.97;
    cycle(motor);
    assertTrue(inputs.atSetpoint);
  }

  @Test
  void unjamReversesThenGivesTheMotorBackToTheRobotCode() {
    FakeMotor motor =
        new FakeMotor(
            "Intake/Roller", new MotorConfig().withStallReversal(Amps.of(40), 0.1, 0.2, 0.3));
    motor.runVelocity(RotationsPerSecond.of(50.0));
    assertEquals("velocity 50.0 slot 0", motor.lastCall());

    // Jammed: driven, not moving, high current.
    motor.volts = 6.0;
    motor.amps = 60.0;
    motor.velocityRps = 0.0;
    cycles(motor, 0.14);
    assertTrue(inputs.stalled);
    assertTrue(inputs.unjamming);
    assertTrue(inputs.hasProblem);
    assertEquals("percent -0.3", motor.lastCall(), "reverses against the applied voltage");
    int callsDuringUnjam = motor.calls.size();

    // The state machine keeps commanding during the reversal: remembered, not sent.
    motor.runVelocity(RotationsPerSecond.of(55.0));
    motor.volts = -3.6;
    motor.amps = 10.0;
    motor.velocityRps = -5.0;
    cycles(motor, 0.1);
    assertTrue(inputs.unjamming);
    assertEquals(callsDuringUnjam, motor.calls.size(), motor.calls.toString());

    // The reversal is over: back to what the robot code asked for last.
    cycles(motor, 0.12);
    assertFalse(inputs.unjamming);
    assertEquals("velocity 55.0 slot 0", motor.lastCall());
  }

  @Test
  void stopDuringTheUnjamStopsAtOnce() {
    FakeMotor motor =
        new FakeMotor(
            "Intake/Roller", new MotorConfig().withStallReversal(Amps.of(40), 0.1, 1.0, 0.3));
    motor.runPercentOutput(0.5);
    motor.volts = 6.0;
    motor.amps = 60.0;
    cycles(motor, 0.14);
    assertTrue(inputs.unjamming);

    motor.stop();
    cycle(motor);
    assertFalse(inputs.unjamming);
    assertEquals("stop", motor.lastCall());
  }

  @Test
  void healthMonitorOnlyJudgesMotorsThatAreRead() {
    FakeMotor neverRead = new FakeMotor("Shooter/Follower", new MotorConfig());
    neverRead.connected = false;
    FakeMotor missing = new FakeMotor("Intake/Roller", new MotorConfig());
    missing.connected = false;

    cycles(missing, 6.0);
    Map<String, Level> alerts = env.activeAlerts();
    assertEquals(
        Level.ERROR, alerts.get("Hardware disconnected: Intake/Roller"), alerts.toString());
    assertEquals("Disconnected", inputs.problem);

    missing.connected = true;
    cycles(missing, 0.04); // the monitor runs before the subsystem reads the motor
    assertNull(env.activeAlerts().get("Hardware disconnected: Intake/Roller"));
    assertFalse(inputs.hasProblem);
  }
}
