package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ciprix.interfaces.motor.SoftwareClosedLoop.Style;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.junit.jupiter.api.Test;

/**
 * The software loop computes what the real controllers compute with the same gains, so the values
 * tuned in simulation are a good start on the robot.
 */
class SoftwareClosedLoopTest {

  private static final double DT = 0.001;
  private static final MotorControlMode VELOCITY = MotorControlMode.VELOCITY;
  private static final MotorControlMode POSITION = MotorControlMode.POSITION;
  private static final MotorControlMode SMART = MotorControlMode.SMART_POSITION;

  @Test
  void velocityIsFeedforwardPlusProportional() {
    MotorConfig config = new MotorConfig().pid(0, 0.5, 0.0, 0.0).svag(0, 0.1, 0.12, 0.0, 0.0);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);

    double volts = loop.calculate(VELOCITY, 50.0, 0, 0.0, 40.0, DT);
    assertEquals(0.1 + 0.12 * 50.0 + 0.5 * 10.0, volts, 1e-9);
  }

  @Test
  void outputStaysInsideTheOutputRange() {
    MotorConfig config = new MotorConfig().svag(0, 0.0, 0.12, 0.0, 0.0);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    assertEquals(12.0, loop.calculate(VELOCITY, 200.0, 0, 0.0, 0.0, DT), 1e-9);

    loop.setOutputRange(-0.5, 0.5);
    assertEquals(6.0, loop.calculate(VELOCITY, 200.0, 0, 0.0, 0.0, DT), 1e-9);
    assertEquals(-6.0, loop.calculate(VELOCITY, -200.0, 0, 0.0, 0.0, DT), 1e-9);
  }

  @Test
  void positionGainIsInVoltsPerRotation() {
    MotorConfig config = new MotorConfig().pid(0, 24.0, 0.0, 0.0);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    assertEquals(6.0, loop.calculate(POSITION, 0.5, 0, 0.25, 0.0, DT), 1e-9);
  }

  @Test
  void positionDerivativeDampsTheVelocity() {
    MotorConfig config = new MotorConfig().pid(0, 0.0, 0.0, 2.0);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    // At the target but moving at 1 rotation per second: the derivative pushes back.
    assertEquals(-2.0, loop.calculate(POSITION, 0.0, 0, 0.0, 1.0, DT), 1e-9);
  }

  @Test
  void armGravityFollowsTheCosine() {
    MotorConfig config = new MotorConfig().svag(0, 0.0, 0.0, 0.0, 0.5).armGravity();
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    assertEquals(0.5, loop.calculate(POSITION, 0.0, 0, 0.0, 0.0, DT), 1e-9, "horizontal");
    assertEquals(0.0, loop.calculate(POSITION, 0.25, 0, 0.25, 0.0, DT), 1e-9, "vertical");
    assertEquals(-0.5, loop.calculate(POSITION, 0.5, 0, 0.5, 0.0, DT), 1e-9, "other side");
  }

  @Test
  void elevatorGravityIsConstant() {
    MotorConfig config = new MotorConfig().svag(0, 0.0, 0.0, 0.0, 0.4);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    assertEquals(0.4, loop.calculate(POSITION, 3.0, 0, 3.0, 0.0, DT), 1e-9);
    assertEquals(0.4, loop.calculate(POSITION, 7.0, 0, 7.0, 0.0, DT), 1e-9);
  }

  @Test
  void sparkGainsAreInRpmAndDutyCycle() {
    // kP in duty cycle per RPM, kV in volts per RPM, as typed in REV Hardware Client.
    MotorConfig config = new MotorConfig().pid(0, 1e-4, 0.0, 0.0).svag(0, 0.0, 0.002, 0.0, 0.0);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.SPARK);

    // 10 rps = 600 RPM wanted, 540 RPM measured: 60 RPM of error.
    double volts = loop.calculate(VELOCITY, 10.0, 0, 0.0, 9.0, DT);
    assertEquals(1e-4 * 60.0 * 12.0 + 0.002 * 600.0, volts, 1e-9);
  }

  @Test
  void onlyTalonAddsGravityToAVelocityLoop() {
    MotorConfig config = new MotorConfig().svag(0, 0.0, 0.0, 0.0, 1.0);
    SoftwareClosedLoop talon = new SoftwareClosedLoop(config, Style.VOLTS);
    SoftwareClosedLoop spark = new SoftwareClosedLoop(config, Style.SPARK);
    assertEquals(1.0, talon.calculate(VELOCITY, 0.0, 0, 0.0, 0.0, DT), 1e-9);
    assertEquals(0.0, spark.calculate(VELOCITY, 0.0, 0, 0.0, 0.0, DT), 1e-9);
  }

  @Test
  void integratorStartsAgainWhenTheModeChanges() {
    MotorConfig config = new MotorConfig().pid(0, 0.0, 10.0, 0.0);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);

    double volts = 0.0;
    for (int i = 0; i < 100; i++) {
      volts = loop.calculate(VELOCITY, 1.0, 0, 0.0, 0.0, DT); // 1 rps of error for 0.1 s
    }
    assertEquals(1.0, volts, 1e-9);

    assertEquals(0.0, loop.calculate(POSITION, 0.0, 0, 0.0, 0.0, DT), 1e-9, "nothing leaks");
    assertEquals(0.01, loop.calculate(VELOCITY, 1.0, 0, 0.0, 0.0, DT), 1e-9);
  }

  @Test
  void integratorDoesNotWindUpPastTheWholeOutput() {
    MotorConfig config = new MotorConfig().pid(0, 0.0, 100.0, 0.0);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    for (int i = 0; i < 1000; i++) {
      loop.calculate(VELOCITY, 10.0, 0, 0.0, 0.0, DT); // would integrate to 1000 V
    }
    double volts = 0.0;
    for (int i = 0; i < 20; i++) {
      volts = loop.calculate(VELOCITY, -10.0, 0, 0.0, 0.0, DT);
    }
    assertTrue(volts < 0.0, "reverses quickly instead of unwinding 1000 V: " + volts);
  }

  @Test
  void motionProfileMovesTheSetpointGradually() {
    MotorConfig config =
        new MotorConfig()
            .smartMotion(
                0,
                RotationsPerSecond.of(1.0),
                RotationsPerSecondPerSecond.of(2.0),
                Rotations.of(0.0));
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    assertTrue(loop.hasProfile(0));

    for (int i = 0; i < 250; i++) {
      loop.calculate(SMART, 1.0, 0, 0.0, 0.0, DT);
    }
    // 0.25 s at 2 rot/s²: 0.5 rot/s and 0.0625 rot.
    assertEquals(0.5, loop.getProfileVelocity(), 1e-3);
    assertEquals(0.0625, loop.getProfilePosition(), 1e-3);

    for (int i = 0; i < 1500; i++) {
      loop.calculate(SMART, 1.0, 0, 0.0, 0.0, DT);
      assertTrue(loop.getProfileVelocity() <= 1.0 + 1e-9, "never above the cruise velocity");
    }
    assertEquals(1.0, loop.getProfilePosition(), 1e-6);
    assertEquals(0.0, loop.getProfileVelocity(), 1e-6);
  }

  @Test
  void slotWithoutProfileBehavesLikeAPlainPositionLoop() {
    MotorConfig config = new MotorConfig().pid(0, 24.0, 0.0, 0.0);
    SoftwareClosedLoop smart = new SoftwareClosedLoop(config, Style.VOLTS);
    SoftwareClosedLoop plain = new SoftwareClosedLoop(config, Style.VOLTS);
    assertFalse(smart.hasProfile(0));
    assertEquals(
        plain.calculate(POSITION, 0.5, 0, 0.25, 0.0, DT),
        smart.calculate(SMART, 0.5, 0, 0.25, 0.0, DT),
        1e-9);
  }

  @Test
  void wrappedPositionTakesTheShortestWay() {
    MotorConfig config =
        new MotorConfig().pid(0, 10.0, 0.0, 0.0).withPositionWrap(Rotations.of(0), Rotations.of(1));
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    // From 0.05 to 0.95 is 0.1 rotation backwards, not 0.9 forwards.
    assertEquals(-1.0, loop.calculate(POSITION, 0.95, 0, 0.05, 0.0, DT), 1e-9);
  }

  @Test
  void eachSlotKeepsItsOwnGains() {
    MotorConfig config = new MotorConfig().pid(0, 1.0, 0.0, 0.0).pid(1, 3.0, 0.0, 0.0);
    SoftwareClosedLoop loop = new SoftwareClosedLoop(config, Style.VOLTS);
    assertEquals(1.0, loop.calculate(POSITION, 1.0, 0, 0.0, 0.0, DT), 1e-9);
    assertEquals(3.0, loop.calculate(POSITION, 1.0, 1, 0.0, 0.0, DT), 1e-9);

    loop.setPID(1, 5.0, 0.0, 0.0);
    assertEquals(5.0, loop.calculate(POSITION, 1.0, 1, 0.0, 0.0, DT), 1e-9, "tuned live");
  }
}
