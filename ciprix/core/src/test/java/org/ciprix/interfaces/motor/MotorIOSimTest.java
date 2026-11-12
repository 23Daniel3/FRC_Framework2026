package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ciprix.TestEnvironment;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import org.ciprix.sim.SimBattery;
import org.ciprix.sim.SimMechanism;
import org.ciprix.util.PersistentAlert.Level;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * A simulated motor behaves like the real controller with the same configuration, so state machines
 * and gains can be tried without the robot.
 */
class MotorIOSimTest {

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

  private void run(MotorIOSim motor, double seconds) {
    int cycles = (int) Math.round(seconds / 0.02);
    for (int i = 0; i < cycles; i++) {
      env.cycle();
      motor.updateInputs(inputs);
    }
  }

  /** Volts per rotation per second of the motor shaft: the ideal kV of a Talon. */
  private static double talonKv(DCMotor motor) {
    return 2.0 * Math.PI / motor.KvRadPerSecPerVolt;
  }

  @Test
  void talonVelocityLoopReachesTheTarget() {
    DCMotor kraken = DCMotor.getKrakenX60(1);
    MotorConfig config =
        new MotorConfig()
            .pid(0, 0.3, 0.0, 0.0)
            .svag(0, 0.0, talonKv(kraken), 0.0, 0.0)
            .withVelocityTolerance(RotationsPerSecond.of(1.0));
    MotorIOSim flywheel =
        MotorIOSim.talonFX("Shooter/Flywheel", config, SimMechanism.flywheel(kraken, 1.0, 0.002));

    flywheel.runVelocity(RotationsPerSecond.of(60.0));
    run(flywheel, 2.0);
    assertEquals(60.0, inputs.velocity.in(RotationsPerSecond), 1.0);
    assertTrue(inputs.atSetpoint);
  }

  @Test
  void sparkVelocityLoopUsesRpmGains() {
    DCMotor vortex = DCMotor.getNeoVortex(1);
    double kvPerRpm = talonKv(vortex) / 60.0; // what REV Hardware Client calls kV
    MotorConfig config =
        new MotorConfig()
            .pid(0, 0.0002, 0.0, 0.0)
            .svag(0, 0.0, kvPerRpm, 0.0, 0.0)
            .withVelocityTolerance(RotationsPerSecond.of(1.0));
    MotorIOSim kicker =
        MotorIOSim.spark("Shooter/Kicker", config, SimMechanism.roller(vortex, 1.0, 0.001));

    kicker.runVelocity(RotationsPerSecond.of(40.0));
    run(kicker, 2.0);
    assertEquals(40.0, inputs.velocity.in(RotationsPerSecond), 1.0);
  }

  /** Arm of 2 kg and 40 cm on a 50:1 NEO Vortex, with the sensor in arm rotations. */
  private static MotorConfig armConfig() {
    DCMotor vortex = DCMotor.getNeoVortex(1);
    double gearing = 50.0;
    double holdingTorque = 2.0 * 9.80665 * 0.4 / 2.0;
    double kG = holdingTorque / gearing / vortex.KtNMPerAmp * vortex.rOhms;
    return new MotorConfig()
        .conversionFactors(1.0 / gearing, 1.0 / gearing)
        .pid(0, 60.0, 0.0, 2.0)
        .svag(0, 0.0, 0.0, 0.0, kG)
        .armGravity()
        .withPositionTolerance(Rotations.of(0.01));
  }

  private static SimMechanism arm() {
    return SimMechanism.arm(DCMotor.getNeoVortex(1), 50.0, 2.0, 0.4)
        .withLimits(-0.25, 0.25)
        .withStartPosition(0.0);
  }

  @Test
  void armPositionLoopHoldsTheTargetAgainstGravity() {
    MotorIOSim pivot = MotorIOSim.talonFX("Intake/Pivot", armConfig(), arm());
    pivot.runPosition(Rotations.of(0.1));
    run(pivot, 2.0);
    assertEquals(0.1, inputs.position.in(Rotations), 0.005);
    assertTrue(inputs.atSetpoint);
  }

  @Test
  void motionProfileMovesTheArmSmoothly() {
    MotorConfig config =
        armConfig()
            .smartMotion(
                0,
                RotationsPerSecond.of(0.5),
                RotationsPerSecondPerSecond.of(2.0),
                Rotations.of(0.0));
    MotorIOSim pivot = MotorIOSim.talonFX("Intake/Pivot", config, arm());

    pivot.runSmartPosition(Rotations.of(0.2));
    run(pivot, 0.1);
    assertTrue(inputs.position.in(Rotations) < 0.05, "follows the profile instead of jumping");

    double fastest = 0.0;
    for (int i = 0; i < 100; i++) {
      run(pivot, 0.02);
      fastest = Math.max(fastest, inputs.velocity.in(RotationsPerSecond));
    }
    assertTrue(fastest < 0.6, "about the cruise velocity: " + fastest);
    assertEquals(0.2, inputs.position.in(Rotations), 0.005);
  }

  @Test
  void disabledRobotPutsTheMotorInNeutral() {
    MotorIOSim roller =
        MotorIOSim.talonFX(
            "Intake/Roller",
            new MotorConfig(),
            SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001));
    roller.runVoltage(Volts.of(6.0));
    run(roller, 0.5);
    assertEquals(6.0, inputs.appliedVolts.in(Volts), 1e-6);

    env.enabled = false;
    run(roller, 0.1);
    assertEquals(0.0, inputs.appliedVolts.in(Volts), 1e-9);

    env.enabled = true;
    run(roller, 0.1);
    assertEquals(6.0, inputs.appliedVolts.in(Volts), 1e-6, "the command is still there");
  }

  @Test
  void jamIsDetectedAndFreedAutomatically() {
    boolean[] jammed = {false};
    MotorIOSim roller =
        MotorIOSim.talonFX(
            "Intake/Roller",
            new MotorConfig().withStallReversal(Amps.of(30), 0.2, 0.3, 0.5),
            SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001)
                .withLoad(() -> jammed[0] ? 1e6 : 0.0));
    roller.runPercentOutput(0.8);
    run(roller, 0.5);
    assertTrue(inputs.velocity.in(RotationsPerSecond) > 10.0);

    jammed[0] = true;
    for (int i = 0; i < 50 && !inputs.unjamming; i++) {
      run(roller, 0.02);
    }
    assertTrue(inputs.unjamming, "a game piece got stuck: the roller reverses");
    assertTrue(inputs.stalled);
    run(roller, 0.04);
    assertTrue(inputs.appliedVolts.in(Volts) < 0.0, "driven backwards");

    jammed[0] = false; // the reversal freed it
    run(roller, 0.5);
    assertFalse(inputs.unjamming);
    assertEquals(0.8 * 12.0, inputs.appliedVolts.in(Volts), 1e-6, "back to the robot command");
    assertTrue(inputs.velocity.in(RotationsPerSecond) > 10.0);
  }

  @Test
  void injectedDisconnectionReachesTheDriveTeam() {
    MotorIOSim roller =
        MotorIOSim.talonFX(
            "Intake/Roller",
            new MotorConfig(),
            SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001));
    run(roller, 1.0);
    assertTrue(env.activeAlerts().isEmpty());

    roller.setConnected(false);
    run(roller, 0.06);
    assertTrue(inputs.hasProblem);
    assertEquals("Disconnected", inputs.problem);
    assertEquals(Level.ERROR, env.activeAlerts().get("Hardware disconnected: Intake/Roller"));

    roller.setConnected(true);
    run(roller, 0.06);
    assertTrue(env.activeAlerts().isEmpty());
  }

  @Test
  void hotMotorIsAProblem() {
    MotorIOSim roller =
        MotorIOSim.talonFX(
            "Intake/Roller",
            new MotorConfig(),
            SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001));
    roller.setTemperatureCelsius(95.0);
    run(roller, 0.04);
    assertEquals("Over temperature", inputs.problem, "the default warning is 85 °C");
  }

  @Test
  void accelerationPullsTheBatteryDown() {
    MotorIOSim flywheel =
        MotorIOSim.talonFX(
                "Shooter/Flywheel",
                new MotorConfig(),
                SimMechanism.flywheel(DCMotor.getKrakenX60(2), 1.0, 0.01))
            .withFollowers(1);
    flywheel.runPercentOutput(1.0);
    run(flywheel, 0.2);
    assertTrue(SimBattery.getTotalCurrentAmps() > 10.0, "" + SimBattery.getTotalCurrentAmps());
    assertTrue(SimBattery.getVoltage() < 12.5);
  }

  @Test
  void offsetChangesTheReportedPosition() {
    MotorIOSim roller =
        MotorIOSim.talonFX(
            "Intake/Roller",
            new MotorConfig().brakeMode(),
            SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001).withFriction(0.05, 0.0));
    roller.runVoltage(Volts.of(3.0));
    run(roller, 0.3);
    roller.stop();
    run(roller, 0.5);
    assertEquals(0.0, inputs.velocity.in(RotationsPerSecond), 1e-9, "stopped");
    assertTrue(Math.abs(inputs.position.in(Rotations)) > 1.0);

    roller.setOffset(Rotations.of(2.0));
    run(roller, 0.02);
    assertEquals(2.0, inputs.position.in(Rotations), 1e-6);
  }
}
