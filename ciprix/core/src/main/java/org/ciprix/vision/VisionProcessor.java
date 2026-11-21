package org.ciprix.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.ciprix.util.RobotClock;
import org.ciprix.vision.VisionFilter.Rejection;
import org.littletonrobotics.junction.Logger;

/**
 * The part of the vision that decides, with no hardware: it takes what each camera saw, throws away
 * the bad poses and the images already used, and sends the good ones to the pose estimator with
 * their standard deviations. {@link AprilTagVision} uses it; a team with its own vision subsystem
 * can use it directly.
 *
 * <p>Each image is judged once: an image that comes again (a camera read as "latest value" in every
 * cycle) is a {@link Rejection#DUPLICATE}, so it never counts twice in the pose estimator.
 */
public final class VisionProcessor {

  /** Receives the accepted poses, usually {@code drivetrain::addVisionMeasurement}. */
  @FunctionalInterface
  public interface MeasurementConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations);
  }

  /** How long after an accepted pose the estimate counts as trusted (see VisionFilter.maxJump). */
  public static final double TRUST_SECONDS = 1.0;

  private static final Pose3d[] NO_POSES = new Pose3d[0];

  private final MeasurementConsumer consumer;
  private VisionFilter filter;
  private VisionStdDevs stdDevs;
  private final Map<String, Double> lastTimestamps = new HashMap<>();
  private final Map<String, Integer> acceptedCounts = new HashMap<>();
  private final Map<String, Integer> rejectedCounts = new HashMap<>();
  private double lastAcceptedTime = Double.NEGATIVE_INFINITY;

  /**
   * @param filter rules that decide which poses are used
   * @param stdDevs how much each pose is trusted
   * @param consumer receives the accepted poses
   */
  public VisionProcessor(VisionFilter filter, VisionStdDevs stdDevs, MeasurementConsumer consumer) {
    this.filter = filter == null ? new VisionFilter() : filter;
    this.stdDevs = stdDevs == null ? new VisionStdDevs() : stdDevs;
    this.consumer = consumer == null ? (pose, timestamp, deviations) -> {} : consumer;
  }

  /** Replaces the rules. */
  public void setFilter(VisionFilter filter) {
    if (filter != null) {
      this.filter = filter;
    }
  }

  /** Replaces the standard deviations. */
  public void setStdDevs(VisionStdDevs stdDevs) {
    if (stdDevs != null) {
      this.stdDevs = stdDevs;
    }
  }

  /** The rules in use. */
  public VisionFilter getFilter() {
    return filter;
  }

  /**
   * Uses what one camera saw in this cycle: the good poses go to the pose estimator, oldest first.
   *
   * @param camera name of the camera, used in the log keys ({@code Vision/<camera>/...})
   * @param inputs what the camera saw
   * @param cameraFactor 1 for a normal camera, more for a camera trusted less
   * @param yawRateRadPerSec how fast the robot turns
   * @param robotSpeed speed of the robot in m/s, for {@link VisionStdDevs#motionPenalty}
   * @param estimatedPose where the pose estimator thinks the robot is (may be null)
   * @return how many poses were accepted
   */
  public int process(
      String camera,
      VisionIO.VisionIOInputs inputs,
      double cameraFactor,
      double yawRateRadPerSec,
      double robotSpeed,
      Pose2d estimatedPose) {
    double now = RobotClock.now();
    VisionFilter.Context context =
        new VisionFilter.Context(yawRateRadPerSec, estimatedPose, hasRecentPose(now));
    List<Pose3d> accepted = new ArrayList<>();
    List<Pose3d> rejected = new ArrayList<>();
    Rejection lastRejection = Rejection.NONE;

    for (PoseObservation observation : inputs.poseObservations) {
      if (observation == null) {
        continue;
      }
      String key = camera + "/" + observation.type();
      double last = lastTimestamps.getOrDefault(key, Double.NEGATIVE_INFINITY);
      Rejection rejection =
          observation.timestamp() <= last
              ? Rejection.DUPLICATE
              : filter.check(observation, context);
      if (rejection != Rejection.DUPLICATE) {
        lastTimestamps.put(key, observation.timestamp());
      }
      if (rejection == Rejection.NONE) {
        lastAcceptedTime = now;
        consumer.accept(
            observation.toPose2d(),
            observation.timestamp(),
            stdDevs.compute(observation, cameraFactor, robotSpeed));
        accepted.add(observation.pose());
      } else {
        lastRejection = rejection;
        if (observation.pose() != null) {
          rejected.add(observation.pose());
        }
      }
    }

    String prefix = "Vision/" + camera + "/";
    int acceptedTotal = acceptedCounts.merge(camera, accepted.size(), Integer::sum);
    int rejectedTotal = rejectedCounts.merge(camera, rejected.size(), Integer::sum);
    Logger.recordOutput(prefix + "AcceptedPoses", accepted.toArray(NO_POSES));
    Logger.recordOutput(prefix + "RejectedPoses", rejected.toArray(NO_POSES));
    Logger.recordOutput(prefix + "AcceptedCount", acceptedTotal);
    Logger.recordOutput(prefix + "RejectedCount", rejectedTotal);
    if (lastRejection != Rejection.NONE) {
      Logger.recordOutput(prefix + "LastRejection", lastRejection.name());
    }
    return accepted.size();
  }

  /** True when a pose was accepted in the last {@value #TRUST_SECONDS} s. */
  public boolean hasRecentPose() {
    return hasRecentPose(RobotClock.now());
  }

  private boolean hasRecentPose(double now) {
    return now - lastAcceptedTime < TRUST_SECONDS;
  }

  /** Time of the last accepted pose on the robot clock (negative infinity when none yet). */
  public double getLastAcceptedTime() {
    return lastAcceptedTime;
  }
}
