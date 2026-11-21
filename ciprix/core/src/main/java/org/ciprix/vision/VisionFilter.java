package org.ciprix.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

/**
 * Decides which camera poses are good enough to correct the odometry. Every rule has a default that
 * suits most robots, and each rejection has a reason, logged so a bad camera is easy to spot.
 *
 * <pre>{@code
 * VisionFilter filter = new VisionFilter()
 *     .field(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded))
 *     .maxDistance(4.0)
 *     .maxJump(1.0);
 * }</pre>
 */
public final class VisionFilter {

  /** Why an observation was not used. */
  public enum Rejection {
    /** Accepted. */
    NONE,
    /** No tag in the image. */
    NO_TAGS,
    /** That image was already used. */
    DUPLICATE,
    /** Fewer tags than {@link #minTags}. */
    TOO_FEW_TAGS,
    /** The robot would be outside the field. */
    OUTSIDE_FIELD,
    /** The robot would be above or below the floor. */
    OFF_THE_FLOOR,
    /** The robot would be tilted (only checked for full 3D solutions). */
    TILTED,
    /** The tags are too far away. */
    TOO_FAR,
    /** A single tag whose solution is ambiguous (only for full 3D solutions). */
    AMBIGUOUS,
    /** The robot turns too fast for MegaTag 2, whose heading would be late. */
    TURNING_TOO_FAST,
    /** A single tag far from where the robot surely is (see {@link #maxJump}). */
    JUMP
  }

  /**
   * What the rules need to know about the robot.
   *
   * @param yawRateRadPerSec how fast the robot turns
   * @param estimatedPose where the pose estimator thinks the robot is (may be null)
   * @param estimateTrusted true when vision corrected the estimate a moment ago, so a single tag
   *     that disagrees with it is more likely wrong than the estimate
   */
  public record Context(double yawRateRadPerSec, Pose2d estimatedPose, boolean estimateTrusted) {
    /** No information about the robot: only the rules on the observation itself run. */
    public static final Context NONE = new Context(0.0, null, false);
  }

  // Defaults: the 2026 field (welded), a robot on the floor, tags up to 6 m.
  private double fieldLength = 16.541;
  private double fieldWidth = 8.069;
  private double fieldMargin = 0.5;
  private double maxHeight = 0.75;
  private double maxTiltRadians = Math.toRadians(20.0);
  private double maxDistance = 6.0;
  private double maxAmbiguity = 0.3;
  private int minTags = 1;
  private double maxYawRateRadPerSec = Math.toRadians(720.0);
  private double maxJump = Double.POSITIVE_INFINITY;
  private AprilTagFieldLayout layout = null;

  /** Takes the field size from the AprilTag layout of the event (and keeps it for logging). */
  public VisionFilter field(AprilTagFieldLayout layout) {
    if (layout != null) {
      this.layout = layout;
      this.fieldLength = layout.getFieldLength();
      this.fieldWidth = layout.getFieldWidth();
    }
    return this;
  }

  /** How far outside the field lines a pose may be, in meters. Default 0.5 m. */
  public VisionFilter fieldMargin(double meters) {
    this.fieldMargin = Math.max(0.0, meters);
    return this;
  }

  /** Largest height of the robot above or below the floor, in meters. Default 0.75 m. */
  public VisionFilter maxHeight(double meters) {
    this.maxHeight = Math.abs(meters);
    return this;
  }

  /** Largest roll or pitch of a full 3D solution, in degrees. Default 20 degrees. */
  public VisionFilter maxTiltDegrees(double degrees) {
    this.maxTiltRadians = Math.toRadians(Math.abs(degrees));
    return this;
  }

  /** Largest average distance to the tags, in meters. Default 6 m. */
  public VisionFilter maxDistance(double meters) {
    this.maxDistance = Math.abs(meters);
    return this;
  }

  /** Largest ambiguity of a single-tag full 3D solution, from 0 to 1. Default 0.3. */
  public VisionFilter maxAmbiguity(double ambiguity) {
    this.maxAmbiguity = Math.abs(ambiguity);
    return this;
  }

  /** Fewest tags in an image. Default 1. */
  public VisionFilter minTags(int tags) {
    this.minTags = Math.max(1, tags);
    return this;
  }

  /** Fastest turn at which MegaTag 2 is still used, in degrees per second. Default 720. */
  public VisionFilter maxYawRateDegreesPerSecond(double degreesPerSecond) {
    this.maxYawRateRadPerSec = Math.toRadians(Math.abs(degreesPerSecond));
    return this;
  }

  /**
   * Largest distance between a single-tag pose and the current estimate, in meters, while the
   * estimate is trusted. It throws away the "flipped" solutions of a single tag. When no
   * measurement was accepted for a while the estimate is no longer trusted, so the robot can always
   * find itself again. Off by default.
   */
  public VisionFilter maxJump(double meters) {
    this.maxJump = Math.abs(meters);
    return this;
  }

  /** The AprilTag layout given to {@link #field}, if any. */
  public Optional<AprilTagFieldLayout> getLayout() {
    return Optional.ofNullable(layout);
  }

  /** Checks only the observation itself (no information about the robot). */
  public Rejection check(PoseObservation observation) {
    return check(observation, Context.NONE);
  }

  /** Checks one observation; {@link Rejection#NONE} means it can be used. */
  public Rejection check(PoseObservation observation, Context robot) {
    if (observation == null || observation.tagCount() < 1 || observation.pose() == null) {
      return Rejection.NO_TAGS;
    }
    if (observation.tagCount() < minTags) {
      return Rejection.TOO_FEW_TAGS;
    }
    Pose3d pose = observation.pose();
    if (pose.getX() < -fieldMargin
        || pose.getX() > fieldLength + fieldMargin
        || pose.getY() < -fieldMargin
        || pose.getY() > fieldWidth + fieldMargin) {
      return Rejection.OUTSIDE_FIELD;
    }
    if (Math.abs(pose.getZ()) > maxHeight) {
      return Rejection.OFF_THE_FLOOR;
    }
    boolean fullSolution = !observation.headingFromGyro();
    if (fullSolution
        && (Math.abs(pose.getRotation().getX()) > maxTiltRadians
            || Math.abs(pose.getRotation().getY()) > maxTiltRadians)) {
      return Rejection.TILTED;
    }
    if (observation.averageTagDistance() > maxDistance) {
      return Rejection.TOO_FAR;
    }
    if (fullSolution && observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
      return Rejection.AMBIGUOUS;
    }
    Context context = robot == null ? Context.NONE : robot;
    if (observation.headingFromGyro()
        && Math.abs(context.yawRateRadPerSec()) > maxYawRateRadPerSec) {
      return Rejection.TURNING_TOO_FAST;
    }
    if (observation.tagCount() == 1
        && context.estimateTrusted()
        && context.estimatedPose() != null
        && observation
                .toPose2d()
                .getTranslation()
                .getDistance(context.estimatedPose().getTranslation())
            > maxJump) {
      return Rejection.JUMP;
    }
    return Rejection.NONE;
  }
}
