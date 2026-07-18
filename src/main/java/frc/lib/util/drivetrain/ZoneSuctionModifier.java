package frc.lib.util.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.AllianceSelector;
import frc.lib.zones.Polygon2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ZoneSuctionModifier implements TranslationModifier {
  private final Polygon2d zone;
  private final double strength;

  private Double forwardDirectionX;

  public ZoneSuctionModifier(Polygon2d zone, double strength) {
    this.zone = zone;
    this.strength = strength;
  }

  @Override
  public Translation2d apply(Drivetrain drivetrain, Translation2d input) {
    Translation2d robotLoc = drivetrain.getPose().getTranslation();

    if (!zone.contains(robotLoc)) {
      forwardDirectionX = null;
      return input;
    }

    boolean isBlue =
        AllianceSelector.getInstance().getResolvedAlliance()
            == Alliance.Blue;
    double allianceSign = isBlue ? 1.0 : -1.0;

    Translation2d center = zone.getCenter();

    if (forwardDirectionX == null) {
      double deltaX = center.getX() - robotLoc.getX();
      forwardDirectionX = Math.signum(deltaX) * allianceSign;
    }

    double yError = (center.getY() - robotLoc.getY()) * allianceSign;

    double moveY = yError * 2.5;
    if (Math.abs(yError) < 0.03) {
      moveY = 0.0;
    }

    double xSpeedScale = Math.abs(yError) > 0.20 ? 0.6 : 1.0;
    double moveX = forwardDirectionX * strength * xSpeedScale;

    moveY = Math.max(-strength, Math.min(strength, moveY));

    Translation2d pullVec = new Translation2d(moveX, moveY);

    if (pullVec.getNorm() > strength) {
      pullVec = pullVec.div(pullVec.getNorm()).times(strength);
    }

    return pullVec;
  }
}
