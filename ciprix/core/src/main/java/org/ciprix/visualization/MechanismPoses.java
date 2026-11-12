package org.ciprix.visualization;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

/**
 * Poses of the moving parts of a robot, relative to the robot (origin at the center of the frame,
 * on the floor; X forward, Y left, Z up), as AdvantageScope expects for the articulated components
 * of a 3D model. Use them with {@link RobotVisualizer}:
 *
 * <pre>{@code
 * // An intake that pivots around an axis 30 cm ahead of the center and 18 cm high
 * RobotVisualizer.component(0, MechanismPoses.pivot(
 *     new Translation3d(0.30, 0.0, 0.18), Axis.PITCH, intake::getPivotAngle));
 *
 * // An elevator carriage and an arm mounted on it
 * Supplier<Pose3d> carriage = MechanismPoses.slider(
 *     new Translation3d(0.0, 0.0, 0.10), new Translation3d(0, 0, 1), elevator::getHeight);
 * RobotVisualizer.component(1, carriage);
 * RobotVisualizer.component(2, MechanismPoses.attach(carriage,
 *     MechanismPoses.pivot(new Translation3d(0.05, 0.0, 0.30), Axis.PITCH, arm::getAngle)));
 * }</pre>
 *
 * <p>For a pivot, the CAD model of the part must be exported so that its rotation axis passes
 * through the origin; {@link RobotVisualizer#advantageScopeConfig} writes the offsets that do it.
 */
public final class MechanismPoses {

  /** Axis a part turns around, with the sign chosen so that a positive angle "opens" it. */
  public enum Axis {
    /** Left-right axis of the robot: a positive angle lifts the front (arms, intakes, hoods). */
    PITCH,
    /** Vertical axis: a positive angle turns counterclockwise seen from above (turrets). */
    YAW,
    /** Front-back axis: a positive angle lifts the left side. */
    ROLL
  }

  private MechanismPoses() {}

  /** Rotation of {@code radians} around {@code axis}, with the sign convention of {@link Axis}. */
  public static Rotation3d rotation(Axis axis, double radians) {
    return switch (axis) {
      case PITCH -> new Rotation3d(0.0, -radians, 0.0);
      case YAW -> new Rotation3d(0.0, 0.0, radians);
      case ROLL -> new Rotation3d(radians, 0.0, 0.0);
    };
  }

  /**
   * A part that turns around an axis: arms, intakes, hoods, turrets, wrists.
   *
   * @param axisPoint a point of the rotation axis, relative to the robot (or to the parent, see
   *     {@link #attach})
   * @param axis which axis it turns around
   * @param angle angle of the part, zero where the CAD model was drawn
   */
  public static Supplier<Pose3d> pivot(Translation3d axisPoint, Axis axis, Supplier<Angle> angle) {
    return () -> new Pose3d(axisPoint, rotation(axis, angle.get().in(Radians)));
  }

  /**
   * A part that slides along a straight line: elevator stages and carriages, telescopes.
   *
   * @param start where the part is when the travel is zero (usually the origin of its model)
   * @param direction direction of the travel (any length, it is normalized)
   * @param travel how far the part moved from {@code start}
   */
  public static Supplier<Pose3d> slider(
      Translation3d start, Translation3d direction, Supplier<Distance> travel) {
    double norm = direction.getNorm();
    if (norm < 1e-9) {
      throw new IllegalArgumentException("direction must not be zero");
    }
    Translation3d unit = direction.div(norm);
    return () -> new Pose3d(start.plus(unit.times(travel.get().in(Meters))), Rotation3d.kZero);
  }

  /** A part that does not move, but is drawn separately (for example to change its color). */
  public static Supplier<Pose3d> fixed(Pose3d pose) {
    return () -> pose;
  }

  /**
   * A part mounted on another moving part: an arm on an elevator carriage, a roller on an intake.
   * The child pose is written relative to the parent as it is drawn in the CAD.
   */
  public static Supplier<Pose3d> attach(Supplier<Pose3d> parent, Supplier<Pose3d> child) {
    return () -> {
      Pose3d relative = child.get();
      return parent.get().plus(new Transform3d(relative.getTranslation(), relative.getRotation()));
    };
  }
}
