package org.ciprix.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.ciprix.vision.PoseObservation.Type;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

/**
 * What a camera saw must come back the same from the log, so a replay accepts the same poses.
 * AdvantageKit touches the DriverStation when it logs records, so this test needs the HAL.
 */
class VisionInputsReplayTest {

  @Test
  void inputsSurviveTheLogForReplay() {
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    inputs.connected = true;
    inputs.latestTargetObservation =
        new VisionIO.TargetObservation(Rotation2d.fromDegrees(5), Rotation2d.fromDegrees(-2));
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              1.5, new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero), 0.05, 2, 2.0, Type.MEGATAG_2)
        };
    inputs.tagIds = new int[] {3, 4};

    LogTable table = new LogTable(0);
    inputs.toLog(table);
    VisionIO.VisionIOInputs replayed = new VisionIO.VisionIOInputs();
    replayed.fromLog(table);

    assertTrue(replayed.connected);
    assertEquals(5.0, replayed.latestTargetObservation.tx().getDegrees(), 1e-9);
    assertEquals(1, replayed.poseObservations.length);
    assertEquals(inputs.poseObservations[0].type(), replayed.poseObservations[0].type());
    assertEquals(3.0, replayed.poseObservations[0].pose().getX(), 1e-9);
    assertArrayEquals(new int[] {3, 4}, replayed.tagIds);
  }
}
