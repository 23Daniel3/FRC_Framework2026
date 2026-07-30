package frc.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.GeomUtil;
import frc.lib.zones.Polygon2d;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

public class FieldConstants {
  public static final double fieldLength = 16.54;
  public static final double fieldWidth = 8.07;

  public class Poses {

    public static final Pose2d RESET_POSE_BLUE = new Pose2d(3.6, 4.035, new Rotation2d());
    public static final Pose2d RESET_POSE_RED = AllianceFlipUtil.apply(RESET_POSE_BLUE, false);

    public static final Pose2d SHOOT_INTAKING_LEFT_BLUE = new Pose2d(3, 6.0525, new Rotation2d());
    public static final Pose2d SHOOT_INTAKING_RIGHT_BLUE = new Pose2d(3, 2.0175, new Rotation2d());

    public static final Pose2d SHOOT_INTAKING_LEFT_RED =
        AllianceFlipUtil.apply(SHOOT_INTAKING_LEFT_BLUE, false);
    public static final Pose2d SHOOT_INTAKING_RIGHT_RED =
        AllianceFlipUtil.apply(SHOOT_INTAKING_RIGHT_BLUE, false);

    public static final Pose2d START_AUTO_LEFT_BLUE =
        new Pose2d(4.30, 7.564, Rotation2d.fromDegrees(-85.6));
    public static final Pose2d START_AUTO_CENTER_BLUE = new Pose2d(3.65, 4.035, new Rotation2d());
    public static final Pose2d START_AUTO_RIGHT_BLUE =
        new Pose2d(4.30, 0.506, Rotation2d.fromDegrees(85.6));

    public static final Pose2d START_AUTO_MIDDLE_LEFT_BLUE =
        new Pose2d(3.67, 5.707, new Rotation2d());
    public static final Pose2d START_AUTO_MIDDLE_RIGHT_BLUE =
        new Pose2d(3.67, 2.3, new Rotation2d());

    public static final Pose2d START_AUTO_LEFT_RED =
        AllianceFlipUtil.apply(START_AUTO_LEFT_BLUE, false);
    public static final Pose2d START_AUTO_CENTER_RED =
        AllianceFlipUtil.apply(START_AUTO_CENTER_BLUE, false);
    public static final Pose2d START_AUTO_RIGHT_RED =
        AllianceFlipUtil.apply(START_AUTO_RIGHT_BLUE, false);

    public static final Pose2d START_AUTO_MIDDLE_LEFT_RED =
        AllianceFlipUtil.apply(START_AUTO_MIDDLE_LEFT_BLUE, false);
    public static final Pose2d START_AUTO_MIDDLE_RIGHT_RED =
        AllianceFlipUtil.apply(START_AUTO_MIDDLE_RIGHT_BLUE, false);

    public static final Pose2d SHOOT_LEFT_BLUE =
        new Pose2d(4.24, 7.348, Rotation2d.fromDegrees(-86));
    public static final Pose2d SHOOT_CENTER_BLUE = new Pose2d(2.0, 4.035, new Rotation2d());
    public static final Pose2d SHOOT_RIGHT_BLUE =
        new Pose2d(4.24, 0.722, Rotation2d.fromDegrees(86));

    public static final Pose2d SHOOT_LEFT_RED = AllianceFlipUtil.apply(SHOOT_LEFT_BLUE, false);
    public static final Pose2d SHOOT_CENTER_RED = AllianceFlipUtil.apply(SHOOT_CENTER_BLUE, false);
    public static final Pose2d SHOOT_RIGHT_RED = AllianceFlipUtil.apply(SHOOT_RIGHT_BLUE, false);

    public static final Pose2d COLLECT_NEUTRAL_ZONE_LEFT_BLUE_START =
        new Pose2d(7.1, 6.631, Rotation2d.fromDegrees(45));
    public static final Pose2d COLLECT_NEUTRAL_ZONE_RIGHT_BLUE_START =
        new Pose2d(7.1, 1.439, Rotation2d.fromDegrees(135));

    public static final Pose2d COLLECT_NEUTRAL_ZONE_LEFT_RED_START =
        AllianceFlipUtil.apply(COLLECT_NEUTRAL_ZONE_LEFT_BLUE_START, false);
    public static final Pose2d COLLECT_NEUTRAL_ZONE_RIGHT_RED_START =
        AllianceFlipUtil.apply(COLLECT_NEUTRAL_ZONE_RIGHT_BLUE_START, false);

    public static final Pose2d COLLECT_NEUTRAL_ZONE_LEFT_BLUE_FINAL =
        new Pose2d(7.729, 5.170, new Rotation2d());
    public static final Pose2d COLLECT_NEUTRAL_ZONE_RIGHT_BLUE_FINAL =
        new Pose2d(7.729, 2.9, Rotation2d.fromDegrees(180));

    public static final Pose2d COLLECT_DEPOT_BLUE_START =
        new Pose2d(2.172, 5.783, Rotation2d.fromDegrees(-35.682));
    public static final Pose2d COLLECT_DEPOT_RED_START =
        AllianceFlipUtil.apply(COLLECT_DEPOT_BLUE_START, false);

    public static final Pose2d COLLECT_DEPOT_BLUE_FINAL =
        new Pose2d(0.7, 6, Rotation2d.fromDegrees(-25));
    public static final Pose2d COLLECT_DEPOT_RED_FINAL =
        AllianceFlipUtil.apply(COLLECT_DEPOT_BLUE_FINAL, false);

    public static final Pose2d COLLECT_DEPOT_BLUE_STARTING_SHOOT =
        new Pose2d(0.7, 7, Rotation2d.fromDegrees(-25));
    public static final Pose2d COLLECT_DEPOT_RED_STARTING_SHOOT =
        AllianceFlipUtil.apply(COLLECT_DEPOT_BLUE_STARTING_SHOOT, false);

    public static final Pose2d HUB_CENTER_BLUE = new Pose2d(4.625, 4.035, new Rotation2d());
    public static final Pose2d HUB_CENTER_RED = AllianceFlipUtil.apply(HUB_CENTER_BLUE, false);

    public static final Pose2d CENTER_TRENCH_LEFT_BLUE =
        GeomUtil.translationToPose(Zones.TRENCH_LEFT_BLUE.getCenter());
    public static final Pose2d CENTER_TRENCH_RIGHT_BLUE =
        GeomUtil.translationToPose(Zones.TRENCH_RIGHT_BLUE.getCenter());
    public static final Pose2d CENTER_TRENCH_LEFT_RED =
        GeomUtil.translationToPose(Zones.TRENCH_LEFT_RED.getCenter());
    public static final Pose2d CENTER_TRENCH_RIGHT_RED =
        GeomUtil.translationToPose(Zones.TRENCH_RIGHT_RED.getCenter());
  }

  public class Zones {
    public static final Polygon2d TRENCH_LEFT_BLUE =
        new Polygon2d(
            new Translation2d(3.650, 8.07),
            new Translation2d(3.650, 6.789),
            new Translation2d(5.550, 6.789),
            new Translation2d(5.550, 8.07));

    public static final Polygon2d TRENCH_RIGHT_BLUE =
        AllianceFlipUtil.apply(TRENCH_LEFT_BLUE, false, true, false);

    public static final Polygon2d TRENCH_LEFT_RED =
        AllianceFlipUtil.apply(TRENCH_LEFT_BLUE, true, true, false);

    public static final Polygon2d TRENCH_RIGHT_RED =
        AllianceFlipUtil.apply(TRENCH_RIGHT_BLUE, true, true, false);

    public static final Polygon2d BUMP_LEFT_BLUE =
        new Polygon2d(
            new Translation2d(3.95, 6.4),
            new Translation2d(3.95, 4.7),
            new Translation2d(5.25, 4.7),
            new Translation2d(5.25, 6.4));

    public static final Polygon2d BUMP_RIGHT_BLUE =
        AllianceFlipUtil.apply(BUMP_LEFT_BLUE, false, true, false);

    public static final Polygon2d BUMP_LEFT_RED =
        AllianceFlipUtil.apply(BUMP_LEFT_BLUE, true, true, false);

    public static final Polygon2d BUMP_RIGHT_RED =
        AllianceFlipUtil.apply(BUMP_RIGHT_BLUE, true, true, false);

    public static final Polygon2d ALLIANCE_BLUE_ZONE =
        new Polygon2d(
            new Translation2d(0, 8.07),
            new Translation2d(0, 0),
            new Translation2d(4.624, 0),
            new Translation2d(4.624, 8.07));

    public static final Polygon2d ALLIANCE_RED_ZONE =
        AllianceFlipUtil.apply(ALLIANCE_BLUE_ZONE, true, false, false);

    public static final Polygon2d NEUTRAL_ZONE =
        new Polygon2d(
            new Translation2d(4.624, 8.07),
            new Translation2d(4.624, 0),
            new Translation2d(11.916, 0),
            new Translation2d(11.916, 8.07));

    public static final Polygon2d SHOOT_ZONE_BLUE =
        new Polygon2d(
            new Translation2d(0, 8.07),
            new Translation2d(0, 0),
            new Translation2d(3.6, 1.6),
            new Translation2d(3.6, 6.47));

    public static final Polygon2d SHOOT_ZONE_RED =
        AllianceFlipUtil.apply(SHOOT_ZONE_BLUE, true, true, false);

    public static final Polygon2d NEUTRAL_LEFT_ZONE_BLUE =
        new Polygon2d(
            new Translation2d(4.624, 8.07),
            new Translation2d(4.624, 4.035),
            new Translation2d(11.916, 4.035),
            new Translation2d(11.916, 8.07));

    public static final Polygon2d NEUTRAL_RIGHT_ZONE_BLUE =
        new Polygon2d(
            new Translation2d(4.624, 4.035),
            new Translation2d(4.624, 0),
            new Translation2d(11.916, 0),
            new Translation2d(11.916, 4.035));

    public static final Polygon2d NEUTRAL_MID_ZONE =
        new Polygon2d(
            new Translation2d(4.624, 5.07),
            new Translation2d(4.624, 3),
            new Translation2d(11.916, 3),
            new Translation2d(11.916, 5.07));

    public static final Polygon2d NEUTRAL_LEFT_ZONE_RED = NEUTRAL_RIGHT_ZONE_BLUE;

    public static final Polygon2d NEUTRAL_RIGHT_ZONE_RED = NEUTRAL_LEFT_ZONE_BLUE;

    public static boolean isAtBump(Translation2d robot) {
      return BUMP_LEFT_BLUE.contains(robot)
          || BUMP_LEFT_RED.contains(robot)
          || BUMP_RIGHT_BLUE.contains(robot)
          || BUMP_RIGHT_RED.contains(robot);
    }

    public static boolean isAtTrench(Translation2d robot) {
      return TRENCH_LEFT_BLUE.contains(robot)
          || TRENCH_LEFT_RED.contains(robot)
          || TRENCH_RIGHT_BLUE.contains(robot)
          || TRENCH_RIGHT_RED.contains(robot);
    }

    public static DrivetrainConstants.Zones getGeneralZone(Translation2d robot) {
      if (ALLIANCE_BLUE_ZONE.contains(robot)) {
        return DrivetrainConstants.Zones.ALLIANCE_BLUE_ZONE;
      } else if (ALLIANCE_RED_ZONE.contains(robot)) {
        return DrivetrainConstants.Zones.ALLIANCE_RED_ZONE;
      } else if (NEUTRAL_ZONE.contains(robot)) {
        return DrivetrainConstants.Zones.NEUTRAL_ZONE;
      }
      return DrivetrainConstants.Zones.NOT_ZONE;
    }
  }
}
