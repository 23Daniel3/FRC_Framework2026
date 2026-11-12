package org.ciprix.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import org.ciprix.TestEnvironment;
import org.ciprix.vision.PoseObservation.Type;
import org.ciprix.vision.VisionFilter.Context;
import org.ciprix.vision.VisionFilter.Rejection;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Only good camera poses may move the robot on the field, and each image only once. */
class VisionPipelineTest {

  private final TestEnvironment env = new TestEnvironment();

  @BeforeEach
  void setUp() {
    env.install();
  }

  @AfterEach
  void tearDown() {
    env.uninstall();
  }

  private static PoseObservation seen(
      double timestamp, double x, double y, int tags, double distance, Type type) {
    return new PoseObservation(
        timestamp, new Pose3d(x, y, 0.0, Rotation3d.kZero), 0.05, tags, distance, type);
  }

  @Test
  void goodPoseIsAccepted() {
    VisionFilter filter = new VisionFilter();
    assertEquals(Rejection.NONE, filter.check(seen(1.0, 3.0, 4.0, 2, 2.5, Type.MEGATAG_1)));
  }

  @Test
  void poseOutsideTheFieldIsRejected() {
    VisionFilter filter =
        new VisionFilter().field(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));
    assertEquals(
        Rejection.OUTSIDE_FIELD, filter.check(seen(1.0, 17.5, 4.0, 2, 2.0, Type.MEGATAG_2)));
    assertEquals(
        Rejection.OUTSIDE_FIELD, filter.check(seen(1.0, 3.0, -0.8, 2, 2.0, Type.MEGATAG_2)));
    assertEquals(
        Rejection.NONE,
        filter.check(seen(1.0, 16.8, 4.0, 2, 2.0, Type.MEGATAG_2)),
        "inside the margin");
  }

  @Test
  void flyingOrTiltedRobotIsRejected() {
    VisionFilter filter = new VisionFilter();
    PoseObservation flying =
        new PoseObservation(
            1.0, new Pose3d(3, 4, 1.2, Rotation3d.kZero), 0.0, 2, 2.0, Type.MEGATAG_1);
    assertEquals(Rejection.OFF_THE_FLOOR, filter.check(flying));
    PoseObservation tilted =
        new PoseObservation(
            1.0, new Pose3d(3, 4, 0, new Rotation3d(0, 0.6, 0)), 0.0, 2, 2.0, Type.MEGATAG_1);
    assertEquals(Rejection.TILTED, filter.check(tilted));
  }

  @Test
  void ambiguousSingleTagIsRejectedOnlyForFullSolutions() {
    VisionFilter filter = new VisionFilter().maxAmbiguity(0.2);
    PoseObservation megatag1 =
        new PoseObservation(
            1.0, new Pose3d(3, 4, 0, Rotation3d.kZero), 0.5, 1, 2.0, Type.MEGATAG_1);
    PoseObservation megatag2 =
        new PoseObservation(
            1.0, new Pose3d(3, 4, 0, Rotation3d.kZero), 0.5, 1, 2.0, Type.MEGATAG_2);
    assertEquals(Rejection.AMBIGUOUS, filter.check(megatag1));
    assertEquals(Rejection.NONE, filter.check(megatag2), "the gyro removes the ambiguity");
  }

  @Test
  void farTagsAndFastTurnsAreRejected() {
    VisionFilter filter = new VisionFilter().maxDistance(4.0);
    assertEquals(Rejection.TOO_FAR, filter.check(seen(1.0, 3, 4, 2, 4.5, Type.MEGATAG_1)));
    Context spinning = new Context(Math.toRadians(900), null, false);
    assertEquals(
        Rejection.TURNING_TOO_FAST,
        filter.check(seen(1.0, 3, 4, 2, 2.0, Type.MEGATAG_2), spinning));
    assertEquals(
        Rejection.NONE,
        filter.check(seen(1.0, 3, 4, 2, 2.0, Type.MEGATAG_1), spinning),
        "MegaTag 1 does not use the gyro");
  }

  @Test
  void singleTagJumpIsRejectedOnlyWhileTheEstimateIsTrusted() {
    VisionFilter filter = new VisionFilter().maxJump(1.0);
    Pose2d here = new Pose2d(3.0, 4.0, Rotation2d.kZero);
    PoseObservation flipped = seen(1.0, 5.0, 4.0, 1, 2.0, Type.MEGATAG_2);
    assertEquals(Rejection.JUMP, filter.check(flipped, new Context(0, here, true)));
    assertEquals(
        Rejection.NONE, filter.check(flipped, new Context(0, here, false)), "lost: find again");
    assertEquals(
        Rejection.NONE,
        filter.check(seen(1.0, 5.0, 4.0, 2, 2.0, Type.MEGATAG_2), new Context(0, here, true)),
        "two tags are trusted over the estimate");
  }

  @Test
  void standardDeviationsGrowWithDistanceAndIgnoreTheGyroHeading() {
    VisionStdDevs stdDevs = new VisionStdDevs();
    Matrix<N3, N1> near = stdDevs.compute(seen(1, 3, 4, 1, 1.0, Type.MEGATAG_1), 1.0, 0.0);
    Matrix<N3, N1> far = stdDevs.compute(seen(1, 3, 4, 1, 3.0, Type.MEGATAG_1), 1.0, 0.0);
    Matrix<N3, N1> twoTags = stdDevs.compute(seen(1, 3, 4, 2, 3.0, Type.MEGATAG_1), 1.0, 0.0);
    assertEquals(0.02, near.get(0, 0), 1e-9);
    assertEquals(0.18, far.get(0, 0), 1e-9, "9 times at three times the distance");
    assertEquals(0.09, twoTags.get(0, 0), 1e-9, "halved by a second tag");
    assertTrue(far.get(2, 0) < 1.0);

    Matrix<N3, N1> megatag2 = stdDevs.compute(seen(1, 3, 4, 1, 1.0, Type.MEGATAG_2), 1.0, 0.0);
    assertEquals(0.01, megatag2.get(0, 0), 1e-9);
    assertEquals(VisionStdDevs.IGNORED, megatag2.get(2, 0), "never corrects the gyro");

    Matrix<N3, N1> moving =
        new VisionStdDevs()
            .motionPenalty(0.5)
            .compute(seen(1, 3, 4, 1, 1.0, Type.MEGATAG_1), 2.0, 2.0);
    assertEquals(0.02 * 2.0 * 2.0, moving.get(0, 0), 1e-9, "camera factor and speed");
  }

  /** What the pose estimator received. */
  private record Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

  @Test
  void processorUsesEachImageOnceAndLogsTheRest() {
    List<Measurement> measurements = new ArrayList<>();
    VisionProcessor processor =
        new VisionProcessor(
            new VisionFilter(),
            new VisionStdDevs(),
            (pose, timestamp, stdDevs) ->
                measurements.add(new Measurement(pose, timestamp, stdDevs)));
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    inputs.poseObservations =
        new PoseObservation[] {
          seen(0.98, 3.0, 4.0, 2, 2.0, Type.MEGATAG_1),
          seen(0.98, 3.01, 4.0, 2, 2.0, Type.MEGATAG_2),
          seen(0.99, 30.0, 4.0, 2, 2.0, Type.MEGATAG_2)
        };
    assertEquals(2, processor.process("Front", inputs, 1.0, 0.0, 0.0, null));
    assertEquals(0.98, measurements.get(0).timestamp(), 1e-9);
    assertEquals(VisionStdDevs.IGNORED, measurements.get(1).stdDevs().get(2, 0));

    // The same images again (a camera read as "latest value"): nothing new reaches the estimator.
    assertEquals(0, processor.process("Front", inputs, 1.0, 0.0, 0.0, null));
    assertEquals(2, measurements.size());
    assertTrue(processor.hasRecentPose());

    // Another camera has its own images.
    assertEquals(2, processor.process("Back", inputs, 1.0, 0.0, 0.0, null));
  }

  @Test
  void estimateStopsBeingTrustedWhenVisionIsQuiet() {
    VisionProcessor processor =
        new VisionProcessor(new VisionFilter().maxJump(1.0), new VisionStdDevs(), null);
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    Pose2d estimate = new Pose2d(3.0, 4.0, Rotation2d.kZero);

    inputs.poseObservations = new PoseObservation[] {seen(1.0, 3.0, 4.0, 2, 2.0, Type.MEGATAG_2)};
    assertEquals(1, processor.process("Front", inputs, 1.0, 0.0, 0.0, estimate));
    inputs.poseObservations = new PoseObservation[] {seen(1.1, 6.0, 4.0, 1, 2.0, Type.MEGATAG_2)};
    assertEquals(0, processor.process("Front", inputs, 1.0, 0.0, 0.0, estimate), "a jump");

    env.now += VisionProcessor.TRUST_SECONDS + 0.1;
    inputs.poseObservations = new PoseObservation[] {seen(2.2, 6.0, 4.0, 1, 2.0, Type.MEGATAG_2)};
    assertEquals(1, processor.process("Front", inputs, 1.0, 0.0, 0.0, estimate), "relocalized");
  }

  @Test
  void limelightArrayIsParsed() {
    double[] botpose = {
      3.0,
      4.0,
      0.02,
      0.0,
      0.0,
      90.0, // pose
      25.0,
      1,
      0.0,
      2.4,
      0.3, // latency ms, tag count, span, average distance, area
      10,
      0.1,
      0.2,
      0.3,
      2.3,
      2.4,
      0.15 // one tag: id 10 with ambiguity 0.15
    };
    PoseObservation observation = VisionIOLimelight.parse(botpose, 2_000_000L, Type.MEGATAG_2);
    assertEquals(2.0 - 0.025, observation.timestamp(), 1e-9, "minus the latency");
    assertEquals(3.0, observation.pose().getX(), 1e-9);
    assertEquals(Math.PI / 2, observation.pose().getRotation().getZ(), 1e-9);
    assertEquals(1, observation.tagCount());
    assertEquals(2.4, observation.averageTagDistance(), 1e-9);
    assertEquals(0.15, observation.ambiguity(), 1e-9);
    assertEquals(List.of(10), VisionIOLimelight.tagIdsOf(botpose));

    double[] nothing = new double[11];
    assertNull(VisionIOLimelight.parse(nothing, 2_000_000L, Type.MEGATAG_1));
    assertNull(VisionIOLimelight.parse(new double[0], 2_000_000L, Type.MEGATAG_1));
  }
}
