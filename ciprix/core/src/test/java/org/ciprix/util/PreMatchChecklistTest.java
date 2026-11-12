package org.ciprix.util;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ciprix.TestEnvironment;
import org.ciprix.interfaces.HardwareHealthMonitor;
import org.ciprix.util.PersistentAlert.Level;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The drive team must see what is still wrong while waiting in the queue, and only then. */
class PreMatchChecklistTest {

  private final TestEnvironment env = new TestEnvironment();

  @BeforeEach
  void setUp() {
    env.install();
    env.enabled = false;
  }

  @AfterEach
  void tearDown() {
    env.uninstall();
  }

  @Test
  void pendingItemsAreListedInOrder() {
    boolean[] autoChosen = {false};
    PreMatchChecklist.addDefaults();
    PreMatchChecklist.add("Auto chosen", () -> autoChosen[0]);
    env.batteryVolts = 12.0;
    for (int i = 0; i < 10; i++) {
      env.cycle();
    }
    assertFalse(PreMatchChecklist.isReady());
    assertArrayEquals(
        new String[] {"Battery at 12.4 V or more", "Auto chosen"}, PreMatchChecklist.getPending());
    assertEquals(
        Level.WARNING,
        env.activeAlerts().get("Before the match: Battery at 12.4 V or more, Auto chosen"),
        env.activeAlerts().toString());

    env.batteryVolts = 12.8;
    autoChosen[0] = true;
    for (int i = 0; i < 10; i++) {
      env.cycle();
    }
    assertTrue(PreMatchChecklist.isReady(), String.join(", ", PreMatchChecklist.getPending()));
    assertTrue(env.activeAlerts().isEmpty(), env.activeAlerts().toString());
  }

  @Test
  void disconnectedDeviceKeepsTheRobotNotReady() {
    boolean[] connected = {false};
    HardwareHealthMonitor.register("Intake/Roller", () -> connected[0], () -> new String[0]);
    PreMatchChecklist.add("Every device connected", HardwareHealthMonitor::isAllConnected);
    env.cycle();
    env.cycle();
    assertFalse(PreMatchChecklist.isReady());
    connected[0] = true;
    env.cycle();
    env.cycle();
    assertTrue(PreMatchChecklist.isReady());
  }

  @Test
  void brokenCheckCountsAsPendingAndTheMatchSilencesTheAlert() {
    PreMatchChecklist.add(
        "Camera calibrated",
        () -> {
          throw new IllegalStateException("not yet");
        });
    env.cycle();
    assertArrayEquals(new String[] {"Camera calibrated"}, PreMatchChecklist.getPending());

    env.enabled = true;
    env.cycle();
    assertNull(env.activeAlerts().get("Before the match: Camera calibrated"), "not in a match");
  }

  @Test
  void predictedPoseFollowsTheFieldRelativeSpeeds() {
    Pose2d now = new Pose2d(2.0, 3.0, Rotation2d.kZero);
    Pose2d later = GeomUtil.predictPose(now, new ChassisSpeeds(1.0, -0.5, Math.PI), 0.5);
    assertEquals(2.5, later.getX(), 1e-9);
    assertEquals(2.75, later.getY(), 1e-9);
    assertEquals(90.0, later.getRotation().getDegrees(), 1e-9);
  }
}
