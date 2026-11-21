package org.ciprix.util.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import org.ciprix.calculus.ThrottleMap;

public class ThrottleMapModifier implements TranslationModifier {
  private final ThrottleMap map =
      new ThrottleMap(
          new double[] {0.0, 0.3, 0.6, 0.8, 0.9, 1.0},
          new double[] {0.0, 0.15, 0.3, 0.5, 0.75, 1.0});

  @Override
  public Translation2d apply(DriveContext drivetrain, Translation2d input) {
    return new Translation2d(
        map.applyThrottleAbs(input.getX()), map.applyThrottleAbs(input.getY()));
  }
}
