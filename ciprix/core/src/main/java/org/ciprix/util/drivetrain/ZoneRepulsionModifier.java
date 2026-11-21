package org.ciprix.util.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import org.ciprix.zones.Polygon2d;

public class ZoneRepulsionModifier implements TranslationModifier {
  private final Polygon2d zone;
  private final double strength;

  public ZoneRepulsionModifier(Polygon2d zone, double strength) {
    this.zone = zone;
    this.strength = strength;
  }

  @Override
  public Translation2d apply(DriveContext drivetrain, Translation2d input) {
    Translation2d robotLoc = drivetrain.getPose().getTranslation();
    if (!zone.contains(robotLoc)) return input;

    Translation2d center = zone.getCenter();
    Translation2d repulsion = robotLoc.minus(center);
    if (repulsion.getNorm() > 1e-6) repulsion = repulsion.div(repulsion.getNorm()).times(strength);

    Translation2d finalVec = input.plus(repulsion);
    if (finalVec.getNorm() > 1.0) finalVec = finalVec.div(finalVec.getNorm());

    return finalVec;
  }
}
