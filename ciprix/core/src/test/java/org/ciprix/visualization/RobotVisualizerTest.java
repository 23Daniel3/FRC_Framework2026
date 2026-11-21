package org.ciprix.visualization;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.nio.file.Files;
import java.nio.file.Path;
import org.ciprix.TestEnvironment;
import org.ciprix.visualization.MechanismPoses.Axis;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** The 3D view must put each CAD part where the mechanism really is. */
class RobotVisualizerTest {

  private final TestEnvironment env = new TestEnvironment();

  @BeforeEach
  void setUp() {
    env.install();
  }

  @AfterEach
  void tearDown() {
    env.uninstall();
  }

  private static void assertTranslation(Translation3d expected, Translation3d actual) {
    assertEquals(expected.getX(), actual.getX(), 1e-9, "x");
    assertEquals(expected.getY(), actual.getY(), 1e-9, "y");
    assertEquals(expected.getZ(), actual.getZ(), 1e-9, "z");
  }

  @Test
  void positivePitchLiftsTheFront() {
    Translation3d tip =
        new Translation3d(1, 0, 0).rotateBy(MechanismPoses.rotation(Axis.PITCH, Math.PI / 2));
    assertTranslation(new Translation3d(0, 0, 1), tip);
  }

  @Test
  void positiveYawTurnsCounterclockwise() {
    Translation3d tip =
        new Translation3d(1, 0, 0).rotateBy(MechanismPoses.rotation(Axis.YAW, Math.PI / 2));
    assertTranslation(new Translation3d(0, 1, 0), tip);
  }

  @Test
  void pivotTurnsAroundItsAxisPoint() {
    Translation3d axis = new Translation3d(0.30, 0.0, 0.18);
    Pose3d pose = MechanismPoses.pivot(axis, Axis.PITCH, () -> Degrees.of(90)).get();
    assertTranslation(axis, pose.getTranslation());
    // A point 0.2 m ahead of the axis on the part ends up 0.2 m above it.
    Pose3d tip = pose.plus(new Transform3d(new Translation3d(0.2, 0, 0), Rotation3d.kZero));
    assertTranslation(new Translation3d(0.30, 0.0, 0.38), tip.getTranslation());
  }

  @Test
  void sliderMovesAlongItsDirection() {
    Pose3d pose =
        MechanismPoses.slider(
                new Translation3d(0, 0, 0.1), new Translation3d(0, 0, 2), () -> Meters.of(0.5))
            .get();
    assertTranslation(new Translation3d(0, 0, 0.6), pose.getTranslation());
  }

  @Test
  void attachedPartFollowsItsParent() {
    var carriage =
        MechanismPoses.slider(
            new Translation3d(0, 0, 0.1), new Translation3d(0, 0, 1), () -> Meters.of(0.5));
    var arm =
        MechanismPoses.attach(
            carriage,
            MechanismPoses.pivot(new Translation3d(0.05, 0, 0.3), Axis.PITCH, () -> Degrees.of(0)));
    assertTranslation(new Translation3d(0.05, 0, 0.9), arm.get().getTranslation());
  }

  @Test
  void posesArePublishedByModelNumber() {
    RobotVisualizer.component(0, MechanismPoses.fixed(new Pose3d(1, 0, 0, Rotation3d.kZero)));
    RobotVisualizer.component(2, MechanismPoses.fixed(new Pose3d(0, 0, 2, Rotation3d.kZero)));
    env.cycle();

    Pose3d[] poses = RobotVisualizer.getPoses();
    assertEquals(3, poses.length);
    assertEquals(1.0, poses[0].getX(), 1e-9);
    assertEquals(Pose3d.kZero, poses[1], "model_1 was not registered");
    assertEquals(2.0, poses[2].getZ(), 1e-9);
  }

  @Test
  void brokenSupplierIsReportedOnceAndDrawnAtTheOrigin() {
    RobotVisualizer.component(
        0,
        () -> {
          throw new IllegalStateException("no inputs yet");
        });
    env.cycle();
    env.cycle();
    assertEquals(Pose3d.kZero, RobotVisualizer.getPoses()[0]);
    assertEquals(1, env.reports.size(), env.reports.toString());
  }

  @Test
  void configZeroesEachPartAtItsPivot(@TempDir Path folder) throws Exception {
    Translation3d axis = new Translation3d(0.30, 0.0, 0.18);
    RobotVisualizer.component(0, MechanismPoses.pivot(axis, Axis.PITCH, () -> Degrees.of(0)), axis);

    String json =
        RobotVisualizer.advantageScopeConfig("Test Robot", RobotVisualizer.ONSHAPE_ROTATIONS);
    assertTrue(json.contains("\"name\": \"Test Robot\""), json);
    assertTrue(json.contains("\"isFTC\": false"), json);
    assertTrue(json.contains("\"zeroedPosition\": [-0.3000, 0.0000, -0.1800]"), json);
    assertTrue(
        json.contains("\"components\": [\n    {\"zeroedRotations\": [{\"axis\": \"x\""), json);

    Path file = folder.resolve("Robot_Test/config.generated.json");
    assertTrue(RobotVisualizer.writeAdvantageScopeConfig(file, "Test Robot", "[]"));
    assertTrue(Files.readString(file).contains("\"rotations\": []"));
  }
}
