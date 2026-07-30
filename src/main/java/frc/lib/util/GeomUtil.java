package frc.lib.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.game.FieldConstants;
import frc.lib.zones.Circle2d;
import frc.lib.zones.Polygon2d;
import java.util.List;
import java.util.function.Supplier;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
public class GeomUtil {

  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d rotationToTransform(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Creates a pure translated pose
   *
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d translationToPose(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure rotated pose
   *
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d rotationToPose(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }

  /**
   * Creates a 3d pose from a 2d pose
   *
   * @param rotation The pose2d to create the pose 3d
   * @return The resulting pose3d
   */
  public static Pose3d toPose3d(Pose2d pose) {
    return new Pose3d(pose);
  }

  /**
   * Multiplies a twist by a scaling factor
   *
   * @param twist The twist to multiply
   * @param factor The scaling factor for the twist components
   * @return The new twist
   */
  public static Twist2d multiplyTwist(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  /**
   * Converts a Pose3d to a Transform3d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform3d pose3dToTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose3d transform3dToPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Y). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation2d translation3dTo2dXY(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getY());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Z). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation2d translation3dTo2dXZ(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getZ());
  }

  public static Pose2d closestPose(Pose2d currentPose, List<Pose2d> poses) {
    return currentPose.nearest(poses);
  }

  /**
   * Calculates the angle required to align to a target pose.
   *
   * @param currentPose The current pose of the robot.
   * @param targetPose The target pose for the robot to align with.
   * @return The angle (in radians) required to align the robot with the target pose.
   */
  public static double thetaToTarget(Pose2d currentPose, Pose2d targetPose) {
    // Gets the X and Y coordinates of the current pose and target pose
    double deltaX = targetPose.getX() - currentPose.getX();
    double deltaY = targetPose.getY() - currentPose.getY();

    // Calculates the angle (in radians) required to align
    return Math.atan2(deltaY, deltaX);
  }

  public static double reverseInterpolate(
      Translation2d query, Translation2d start, Translation2d end) {
    Translation2d segment = end.minus(start);
    Translation2d queryToStart = query.minus(start);

    double segmentLengthSqr = segment.getX() * segment.getX() + segment.getY() * segment.getY();

    if (segmentLengthSqr == 0.0) { // start and end are the same point
      return 0.0;
    }

    return (queryToStart.getX() * segment.getX() + queryToStart.getY() * segment.getY())
        / segmentLengthSqr;
  }

  public static double distanceToLineSegment(
      Translation2d query, Translation2d start, Translation2d end) {
    double t = reverseInterpolate(query, start, end);
    if (t < 0.0) { // closest point is before start
      return query.getDistance(start);
    } else if (t > 1.0) { // closest point is after end
      return query.getDistance(end);
    } else { // closest point is within the segment
      Translation2d segment = end.minus(start);
      Translation2d closestPoint = start.plus(segment.times(t));
      return query.getDistance(closestPoint);
    }
  }

  public static double perpendicularDistanceToLine(
      Translation2d query, Translation2d start, Translation2d end) {
    double t = reverseInterpolate(query, start, end);
    Translation2d segment = end.minus(start);
    Translation2d closestPoint = start.plus(segment.times(t));
    return query.getDistance(closestPoint);
  }

  public static Trigger isWithinTolerance(
      Supplier<Pose2d> currentPoseSupplier, Pose2d targetPose, double toleranceMeters) {
    return new Trigger(
        () -> {
          Pose2d currentPose = currentPoseSupplier.get();
          double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
          return distance <= toleranceMeters;
        });
  }

  public static double flipX(double x) {
    return FieldConstants.fieldLength - x;
  }

  public static double flipY(double y) {
    return FieldConstants.fieldWidth - y;
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static Translation3d flip(Translation3d translation) {
    return new Translation3d(
        flipX(translation.getX()), flipY(translation.getY()), translation.getZ());
  }

  public static Rotation3d flip(Rotation3d rotation) {
    return rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI));
  }

  public static Pose3d flip(Pose3d pose) {
    return new Pose3d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /**
   * Flips a {@link Polygon2d} zone across the field center.
   *
   * <p>Uses {@link FieldConstants#fieldLength} and {@link FieldConstants#fieldWidth} as mirror
   * axes.
   *
   * @param polygon original zone
   * @return flipped zone
   */
  public static Polygon2d flipZone(Polygon2d polygon) {
    Translation2d[] original = polygon.getVertices();
    Translation2d[] flipped = new Translation2d[original.length];

    for (int i = 0; i < original.length; i++) {
      flipped[i] = flip(original[i]);
    }

    return new Polygon2d(flipped);
  }

  /**
   * Creates a field-relative flipped version of a {@link Circle2d}.
   *
   * <p>The circle is mirrored across the field's X and Y axes, using the field dimensions defined
   * in {@link frc.game.FieldConstants}. The radius remains unchanged.
   *
   * @param circle The original circle zone.
   * @return A new {@link Circle2d} mirrored across the field.
   */
  public static Circle2d flip(Circle2d circle) {
    Translation2d flippedCenter = flip(circle.getCenter());
    return new Circle2d(flippedCenter, circle.getRadius());
  }

  /**
   * Creates a field-relative flipped version of an axis-aligned rectangle zone.
   *
   * <p>The rectangle is mirrored across the field's X and Y axes using the two defining opposite
   * corners. The resulting rectangle remains axis-aligned and preserves its original dimensions.
   *
   * @param rect The original rectangle zone.
   * @return A new rectangle zone mirrored across the field.
   */
  public static frc.lib.zones.Rectangle2d flip(frc.lib.zones.Rectangle2d rect) {
    Translation2d flippedA = flip(rect.getCornerA());
    Translation2d flippedB = flip(rect.getCornerB());
    return new frc.lib.zones.Rectangle2d(flippedA, flippedB);
  }
}
