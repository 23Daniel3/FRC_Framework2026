package org.ciprix.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.ciprix.TestEnvironment;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** A bug in one library service must never take the robot program down in a match. */
class RobotLoopTest {

  private final TestEnvironment env = new TestEnvironment();

  @BeforeEach
  void setUp() {
    env.install();
  }

  @AfterEach
  void tearDown() {
    env.uninstall();
  }

  @Test
  void actionsRunInRegistrationOrder() {
    List<String> ran = new ArrayList<>();
    RobotLoop.addPeriodic("First", () -> ran.add("first"));
    RobotLoop.addPeriodic("Second", () -> ran.add("second"));
    RobotLoop.runOnce();
    assertEquals(List.of("first", "second"), ran);
  }

  @Test
  void failingActionIsReportedOnceAndTheOthersKeepRunning() {
    List<String> ran = new ArrayList<>();
    RobotLoop.addPeriodic("Before", () -> ran.add("before"));
    RobotLoop.addPeriodic(
        "Broken",
        () -> {
          throw new IllegalStateException("boom");
        });
    RobotLoop.addPeriodic("After", () -> ran.add("after"));

    for (int i = 0; i < 3; i++) {
      RobotLoop.runOnce();
    }
    assertEquals(List.of("before", "after", "before", "after", "before", "after"), ran);
    assertEquals(1, env.reports.size(), env.reports.toString());
    assertTrue(env.reports.get(0).contains("Broken"), env.reports.get(0));
  }

  @Test
  void actionMayRegisterAnotherAction() {
    List<String> ran = new ArrayList<>();
    RobotLoop.addPeriodic(
        "Registrar",
        () -> {
          if (ran.isEmpty()) {
            RobotLoop.addPeriodic("Late", () -> ran.add("late"));
          }
          ran.add("registrar");
        });
    RobotLoop.runOnce();
    assertEquals(List.of("registrar", "late"), ran);
  }

  @Test
  void nullActionIsIgnoredAndReported() {
    RobotLoop.addPeriodic("Nothing", null);
    assertEquals(0, RobotLoop.size());
    assertEquals(1, env.reports.size());
  }
}
