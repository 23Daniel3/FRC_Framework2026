package org.ciprix.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * One robot pose seen by a camera, in field coordinates (origin at the blue alliance wall), at the
 * moment the image was taken.
 *
 * @param timestamp when the image was taken, in seconds on the robot clock, already without the
 *     camera latency
 * @param pose robot pose on the field
 * @param ambiguity how ambiguous a single-tag solution is, from 0 (sure) to 1 (two solutions
 *     equally good)
 * @param tagCount number of tags used in the solution
 * @param averageTagDistance average distance from the camera to the tags, in meters
 * @param type how the pose was solved
 */
public record PoseObservation(
    double timestamp,
    Pose3d pose,
    double ambiguity,
    int tagCount,
    double averageTagDistance,
    PoseObservation.Type type) {

  /** How a pose was solved. */
  public enum Type {
    /** Full pose from the tags alone (Limelight MegaTag 1). */
    MEGATAG_1,
    /** Position from the tags, heading from the robot gyro (Limelight MegaTag 2). */
    MEGATAG_2,
    /** PhotonVision (multi-tag on the coprocessor, or the lowest ambiguity tag). */
    PHOTONVISION
  }

  /** True when the heading is the gyro's own, so it must not be used to correct the gyro. */
  public boolean headingFromGyro() {
    return type == Type.MEGATAG_2;
  }

  /** The pose on the floor, as the drivetrain pose estimator takes it. */
  public Pose2d toPose2d() {
    return pose.toPose2d();
  }
}
