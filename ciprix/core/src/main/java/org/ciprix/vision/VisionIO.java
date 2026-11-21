package org.ciprix.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware layer of one AprilTag camera, in the AdvantageKit style: the camera fills {@link
 * VisionIOInputs} once per cycle, and everything after that (filters, standard deviations) works
 * the same on the robot, in simulation and when replaying a log.
 */
public interface VisionIO {

  /** Angles to the best target in the last image, for aiming without a pose (0 when none). */
  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** What one camera saw since the last cycle. */
  class VisionIOInputs implements LoggableInputs {
    /** True while the camera is sending images. */
    public boolean connected = false;

    /** Angles to the best target in the last image. */
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    /** Every pose solved since the last cycle, oldest first. */
    public PoseObservation[] poseObservations = new PoseObservation[0];

    /** Ids of the tags seen since the last cycle. */
    public int[] tagIds = new int[0];

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("LatestTargetObservation", latestTargetObservation);
      table.put("PoseObservations", poseObservations);
      table.put("TagIds", tagIds);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      latestTargetObservation = table.get("LatestTargetObservation", latestTargetObservation);
      poseObservations = table.get("PoseObservations", poseObservations);
      tagIds = table.get("TagIds", tagIds);
    }
  }

  /** Reads the camera. Called once per cycle. */
  default void updateInputs(VisionIOInputs inputs) {}
}
