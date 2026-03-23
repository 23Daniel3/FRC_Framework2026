package frc.lib.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.game.FieldConstants;
import frc.lib.zones.Polygon2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Utility class for applying alliance-based field mirroring.
 *
 * <p>This class converts field-relative coordinates, rotations, and poses defined from the Blue
 * Alliance perspective into their Red Alliance equivalents when required. When on the Blue
 * Alliance, values are returned unchanged.
 *
 * <p>The coordinate system assumes the standard WPILib field reference frame, where the origin is
 * located at the Blue Alliance corner of the field.
 */
public class AllianceFlipUtil {

  /**
   * Unconditionally mirrors an X coordinate.
   *
   * @param x the field-relative X position (meters)
   * @return the mirrored X position
   */
  public static double flipX(double x) {
    return FieldConstants.fieldLength - x;
  }

  /**
   * Applies alliance-based mirroring to an X coordinate.
   *
   * @param x the field-relative X position (meters)
   * @param useAllianceColor if true, checks the current alliance color; if false, forces a flip
   * @return the mirrored X position based on logic
   */
  public static double applyX(double x, boolean useAllianceColor) {
    return (!useAllianceColor || shouldFlip()) ? flipX(x) : x;
  }

  /**
   * Applies alliance-based mirroring to an X coordinate.
   *
   * @param x the field-relative X position (meters)
   * @return the mirrored X position if on the Red Alliance; otherwise {@code x}
   */
  public static double applyX(double x) {
    return applyX(x, true);
  }

  /**
   * Unconditionally mirrors a Y coordinate.
   *
   * @param y the field-relative Y position (meters)
   * @return the mirrored Y position
   */
  public static double flipY(double y) {
    return FieldConstants.fieldWidth - y;
  }

  /**
   * Applies alliance-based mirroring to a Y coordinate.
   *
   * @param y the field-relative Y position (meters)
   * @param useAllianceColor if true, checks the current alliance color; if false, forces a flip
   * @return the mirrored Y position based on logic
   */
  public static double applyY(double y, boolean useAllianceColor) {
    return (!useAllianceColor || shouldFlip()) ? flipY(y) : y;
  }

  /**
   * Applies alliance-based mirroring to a Y coordinate.
   *
   * @param y the field-relative Y position (meters)
   * @return the mirrored Y position if on the Red Alliance; otherwise {@code y}
   */
  public static double applyY(double y) {
    return applyY(y, true);
  }

  /**
   * Unconditionally mirrors a 2D translation.
   *
   * @param translation the field-relative translation
   * @return the mirrored translation
   */
  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  /**
   * Applies alliance-based mirroring to a 2D translation.
   *
   * @param translation the field-relative translation
   * @param useAllianceColor if true, checks the current alliance color; if false, forces a flip
   * @return the mirrored translation based on logic
   */
  public static Translation2d apply(Translation2d translation, boolean useAllianceColor) {
    return (!useAllianceColor || shouldFlip()) ? flip(translation) : translation;
  }

  /**
   * Applies alliance-based mirroring to a 2D translation.
   *
   * @param translation the field-relative translation
   * @return the mirrored translation if on the Red Alliance; otherwise the original translation
   */
  public static Translation2d apply(Translation2d translation) {
    return apply(translation, true);
  }

  /**
   * Unconditionally mirrors a 2D rotation.
   *
   * @param rotation the field-relative rotation
   * @return the mirrored rotation (rotated by 180 degrees)
   */
  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  /**
   * Applies alliance-based mirroring to a 2D rotation.
   *
   * <p>When flipped, the rotation is rotated by 180 degrees to preserve the correct field-relative
   * orientation.
   *
   * @param rotation the field-relative rotation
   * @param useAllianceColor if true, checks the current alliance color; if false, forces a flip
   * @return the mirrored rotation based on logic
   */
  public static Rotation2d apply(Rotation2d rotation, boolean useAllianceColor) {
    return (!useAllianceColor || shouldFlip()) ? flip(rotation) : rotation;
  }

  /**
   * Applies alliance-based mirroring to a 2D rotation.
   *
   * <p>When flipped, the rotation is rotated by 180 degrees to preserve the correct field-relative
   * orientation.
   *
   * @param rotation the field-relative rotation
   * @return the mirrored rotation if on the Red Alliance; otherwise the original rotation
   */
  public static Rotation2d apply(Rotation2d rotation) {
    return apply(rotation, true);
  }

  /**
   * Unconditionally mirrors a 2D pose.
   *
   * @param pose the field-relative pose
   * @return the mirrored pose
   */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /**
   * Applies alliance-based mirroring to a 2D pose.
   *
   * @param pose the field-relative pose
   * @param useAllianceColor if true, checks the current alliance color; if false, forces a flip
   * @return the mirrored pose based on logic
   */
  public static Pose2d apply(Pose2d pose, boolean useAllianceColor) {
    return (!useAllianceColor || shouldFlip()) ? flip(pose) : pose;
  }

  /**
   * Applies alliance-based mirroring to a 2D pose.
   *
   * @param pose the field-relative pose
   * @return the mirrored pose if on the Red Alliance; otherwise the original pose
   */
  public static Pose2d apply(Pose2d pose) {
    return apply(pose, true);
  }

  /**
   * Unconditionally mirrors a 3D translation.
   *
   * @param translation the field-relative 3D translation
   * @return the mirrored translation
   */
  public static Translation3d flip(Translation3d translation) {
    return new Translation3d(
        flipX(translation.getX()), flipY(translation.getY()), translation.getZ());
  }

  /**
   * Applies alliance-based mirroring to a 3D translation.
   *
   * <p>The Z component is left unchanged.
   *
   * @param translation the field-relative 3D translation
   * @param useAllianceColor if true, checks the current alliance color; if false, forces a flip
   * @return the mirrored translation based on logic
   */
  public static Translation3d apply(Translation3d translation, boolean useAllianceColor) {
    return (!useAllianceColor || shouldFlip()) ? flip(translation) : translation;
  }

  /**
   * Applies alliance-based mirroring to a 3D translation.
   *
   * <p>The Z component is left unchanged.
   *
   * @param translation the field-relative 3D translation
   * @return the mirrored translation if on the Red Alliance; otherwise the original translation
   */
  public static Translation3d apply(Translation3d translation) {
    return apply(translation, true);
  }

  /**
   * Unconditionally mirrors a 3D rotation.
   *
   * @param rotation the field-relative 3D rotation
   * @return the mirrored rotation (rotated 180 degrees about Z)
   */
  public static Rotation3d flip(Rotation3d rotation) {
    return rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI));
  }

  /**
   * Applies alliance-based mirroring to a 3D rotation.
   *
   * <p>When flipped, a 180-degree rotation about the Z-axis is applied.
   *
   * @param rotation the field-relative 3D rotation
   * @param useAllianceColor if true, checks the current alliance color; if false, forces a flip
   * @return the mirrored rotation based on logic
   */
  public static Rotation3d apply(Rotation3d rotation, boolean useAllianceColor) {
    return (!useAllianceColor || shouldFlip()) ? flip(rotation) : rotation;
  }

  /**
   * Applies alliance-based mirroring to a 3D rotation.
   *
   * <p>When flipped, a 180-degree rotation about the Z-axis is applied.
   *
   * @param rotation the field-relative 3D rotation
   * @return the mirrored rotation if on the Red Alliance; otherwise the original rotation
   */
  public static Rotation3d apply(Rotation3d rotation) {
    return apply(rotation, true);
  }

  /**
   * Unconditionally mirrors a 3D pose.
   *
   * @param pose the field-relative 3D pose
   * @return the mirrored pose
   */
  public static Pose3d flip(Pose3d pose) {
    return new Pose3d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /**
   * Applies alliance-based mirroring to a 3D pose.
   *
   * @param pose the field-relative 3D pose
   * @param useAllianceColor if true, checks the current alliance color; if false, forces a flip
   * @return the mirrored pose based on logic
   */
  public static Pose3d apply(Pose3d pose, boolean useAllianceColor) {
    return (!useAllianceColor || shouldFlip()) ? flip(pose) : pose;
  }

  /**
   * Applies alliance-based mirroring to a 3D pose.
   *
   * @param pose the field-relative 3D pose
   * @return the mirrored pose if on the Red Alliance; otherwise the original pose
   */
  public static Pose3d apply(Pose3d pose) {
    return apply(pose, true);
  }

  /**
   * Determines whether field mirroring should be applied based on the current alliance.
   *
   * @return {@code true} if the robot is on the Red Alliance; {@code false} otherwise
   */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  public static ChassisSpeeds apply(
      Drivetrain drivetrain, double vx, double vy, double omega, boolean useAllianceColor) {
    Rotation2d heading =
        (!useAllianceColor || shouldFlip())
            ? drivetrain.getRotation().plus(Rotation2d.fromRadians(Math.PI))
            : drivetrain.getRotation();

    return ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, heading);
  }

  public static ChassisSpeeds apply(Drivetrain drivetrain, double vx, double vy, double omega) {
    return apply(drivetrain, vx, vy, omega, true);
  }

  /**
   * Returns a new {@link Polygon2d} whose vertices are mirrored across the field axes according to
   * the supplied flags and the current alliance (unless {@code useAllianceColor} is false, in which
   * case the flip is forced).
   *
   * <p>Behavior:
   *
   * <ul>
   *   <li>If {@code useAllianceColor} is true, the flip is applied only when the robot is on the
   *       Red Alliance (see {@link #shouldFlip()}).
   *   <li>{@code flipX} controls mirroring across the field X axis (i.e. {@code x -> fieldLength -
   *       x}).
   *   <li>{@code flipY} controls mirroring across the field Y axis (i.e. {@code y -> fieldWidth -
   *       y}).
   *   <li>If neither {@code flipX} nor {@code flipY} is true (or the flip is not active due to
   *       alliance), the original {@code polygon} is returned unchanged.
   * </ul>
   *
   * @param polygon the polygon to transform
   * @param flipX whether to mirror the polygon's X coordinates (mirror across field length)
   * @param flipY whether to mirror the polygon's Y coordinates (mirror across field width)
   * @param useAllianceColor if true, only apply the flip when on the Red Alliance; if false, force
   *     apply
   * @return a new {@link Polygon2d} with transformed vertices, or the original polygon if no
   *     transform is applied
   */
  public static Polygon2d apply(
      Polygon2d polygon, boolean flipX, boolean flipY, boolean useAllianceColor) {
    // Determine whether we should perform the flip (respect alliance unless explicitly forced).
    boolean doFlip = (!useAllianceColor) || shouldFlip();

    // If no flip requested or not active by alliance, return original polygon.
    if (!doFlip || (!flipX && !flipY)) {
      return polygon;
    }

    Translation2d[] src = polygon.getVertices();
    Translation2d[] dst = new Translation2d[src.length];

    for (int i = 0; i < src.length; i++) {
      double x = src[i].getX();
      double y = src[i].getY();

      if (flipX) {
        x = FieldConstants.fieldLength - x;
      }
      if (flipY) {
        y = FieldConstants.fieldWidth - y;
      }

      dst[i] = new Translation2d(x, y);
    }

    return new Polygon2d(dst);
  }
}
