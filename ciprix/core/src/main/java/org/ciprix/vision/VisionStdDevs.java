package org.ciprix.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * How much the pose estimator should trust each camera pose: standard deviations (x, y, heading)
 * that grow with the square of the distance to the tags and shrink with the number of tags, like
 * the real error of a camera. The heading of a MegaTag 2 pose is the gyro's own, so it is ignored.
 *
 * <pre>{@code
 * VisionStdDevs stdDevs = new VisionStdDevs().linearBaseline(0.02).motionPenalty(0.5);
 * }</pre>
 */
public final class VisionStdDevs {

  /** A standard deviation so large that the pose estimator ignores that axis. */
  public static final double IGNORED = 1.0e6;

  /** Closer tags than this do not make a pose more trusted (avoids standard deviations near 0). */
  private static final double MIN_DISTANCE = 0.5;

  private double linearBaseline = 0.02;
  private double angularBaseline = 0.06;
  private double megatag2LinearFactor = 0.5;
  private double motionPenaltyPerMps = 0.0;

  /** Standard deviation of x and y with one tag at 1 m, in meters. Default 0.02 m. */
  public VisionStdDevs linearBaseline(double meters) {
    this.linearBaseline = Math.abs(meters);
    return this;
  }

  /** Standard deviation of the heading with one tag at 1 m, in radians. Default 0.06 rad. */
  public VisionStdDevs angularBaseline(double radians) {
    this.angularBaseline = Math.abs(radians);
    return this;
  }

  /** Multiplies x and y of MegaTag 2 poses, which are steadier. Default 0.5. */
  public VisionStdDevs megatag2Factor(double factor) {
    this.megatag2LinearFactor = Math.abs(factor);
    return this;
  }

  /**
   * Extra distrust while the robot moves: the standard deviations grow by this fraction per m/s of
   * {@code robotSpeed} (blurred images, latency). Default 0, off.
   */
  public VisionStdDevs motionPenalty(double perMetersPerSecond) {
    this.motionPenaltyPerMps = Math.abs(perMetersPerSecond);
    return this;
  }

  /**
   * Standard deviations for one observation.
   *
   * @param observation the camera pose
   * @param cameraFactor 1 for a normal camera, more for a camera trusted less
   * @param robotSpeed speed of the robot in m/s (used by {@link #motionPenalty})
   * @return x, y and heading standard deviations, as the pose estimator takes them
   */
  public Matrix<N3, N1> compute(
      PoseObservation observation, double cameraFactor, double robotSpeed) {
    double distance = Math.max(MIN_DISTANCE, observation.averageTagDistance());
    double factor =
        distance
            * distance
            / Math.max(1, observation.tagCount())
            * Math.max(0.0, cameraFactor)
            * (1.0 + motionPenaltyPerMps * Math.max(0.0, robotSpeed));
    double linear = linearBaseline * factor;
    double angular = angularBaseline * factor;
    if (observation.headingFromGyro()) {
      linear *= megatag2LinearFactor;
      angular = IGNORED;
    }
    return VecBuilder.fill(linear, linear, angular);
  }
}
