package org.ciprix.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.Test;

/** The physics must match the motor model, so tuning in simulation carries over to the robot. */
class SimMechanismTest {

  private static final double DT = 0.001;

  private static void run(SimMechanism mechanism, double volts, double seconds, double limit) {
    int steps = (int) Math.round(seconds / DT);
    for (int i = 0; i < steps; i++) {
      mechanism.step(volts, DT, limit, 12.5);
    }
  }

  @Test
  void rollerReachesTheFreeSpeedOfTheMotor() {
    DCMotor kraken = DCMotor.getKrakenX60(1);
    double gearing = 3.0;
    SimMechanism roller = SimMechanism.roller(kraken, gearing, 0.001);

    run(roller, 12.0, 3.0, 0.0);

    double expectedRps = 12.0 * kraken.KvRadPerSecPerVolt / gearing / (2.0 * Math.PI);
    assertEquals(expectedRps, roller.getVelocityRps(), expectedRps * 0.005);
    assertEquals(
        expectedRps * gearing, roller.getRotorVelocityRps(), expectedRps * gearing * 0.005);
    assertEquals(0.0, roller.getCurrentAmps(), 1.0, "no load: almost no current at free speed");
  }

  @Test
  void stalledMotorDrawsTheStallCurrentOrTheLimit() {
    DCMotor neo = DCMotor.getNEO(1);
    SimMechanism jammed = SimMechanism.roller(neo, 10.0, 0.01).withLoad(() -> 1e6);

    run(jammed, 12.0, 0.2, 0.0);
    assertEquals(0.0, jammed.getVelocityRps(), 1e-9);
    assertEquals(12.0 / neo.rOhms, jammed.getCurrentAmps(), 1.0);

    run(jammed, 12.0, 0.2, 40.0);
    assertEquals(40.0, jammed.getCurrentAmps(), 1e-6, "the controller holds the limit");
    assertTrue(jammed.getAppliedVolts() < 12.0, "by lowering the voltage");
  }

  @Test
  void armFallsToItsHardStopWhenUnpowered() {
    SimMechanism arm =
        SimMechanism.arm(DCMotor.getNeoVortex(1), 50.0, 4.0, 0.5)
            .withLimits(-0.25, 0.25)
            .withStartPosition(0.0);

    for (int i = 0; i < 3000; i++) {
      arm.stepNeutral(DT, false);
    }
    assertEquals(-0.25, arm.getPositionRotations(), 1e-9);
    assertEquals(0.0, arm.getVelocityRps(), 1e-9);
  }

  @Test
  void elevatorHoldsWithTheGravityVoltage() {
    DCMotor motor = DCMotor.getKrakenX60(2);
    double gearing = 9.0;
    double mass = 6.0;
    double radius = 0.025;
    SimMechanism elevator =
        SimMechanism.elevator(motor, gearing, mass, radius)
            .withLimits(0.0, 10.0)
            .withStartPosition(5.0);

    // Voltage whose torque exactly balances the weight: kG of a real elevator.
    double weightTorque = mass * 9.80665 * radius;
    double holdVolts = weightTorque / (gearing * motor.KtNMPerAmp) * motor.rOhms;

    run(elevator, holdVolts, 1.0, 0.0);
    assertEquals(5.0, elevator.getPositionRotations(), 1e-3);
  }

  @Test
  void brakeStopsFasterThanCoast() {
    SimMechanism braked = SimMechanism.flywheel(DCMotor.getKrakenX60(1), 1.0, 0.005);
    SimMechanism coasting = SimMechanism.flywheel(DCMotor.getKrakenX60(1), 1.0, 0.005);
    braked.withFriction(0.01, 0.0);
    coasting.withFriction(0.01, 0.0);
    braked.setState(0.0, 50.0);
    coasting.setState(0.0, 50.0);

    for (int i = 0; i < 300; i++) {
      braked.stepNeutral(DT, true);
      coasting.stepNeutral(DT, false);
    }
    assertTrue(braked.getVelocityRps() < coasting.getVelocityRps());
    assertTrue(braked.getVelocityRps() >= 0.0, "friction never pushes it backwards");
  }

  @Test
  void batteryLimitsTheVoltage() {
    SimMechanism roller = SimMechanism.roller(DCMotor.getKrakenX60(1), 1.0, 0.001);
    for (int i = 0; i < 2000; i++) {
      roller.step(24.0, DT, 0.0, 10.0);
    }
    assertEquals(10.0, roller.getAppliedVolts(), 1e-6);
  }
}
