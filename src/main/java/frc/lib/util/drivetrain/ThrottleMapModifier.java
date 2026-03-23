package frc.lib.util.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.calculus.ThrottleMap;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ThrottleMapModifier implements TranslationModifier {
  private final ThrottleMap map =
      new ThrottleMap(
          new double[] {0.0, 0.3, 0.6, 0.8, 0.9, 1.0},
          new double[] {0.0, 0.15, 0.3, 0.5, 0.75, 1.0});

  @Override
  public Translation2d apply(Drivetrain drivetrain, Translation2d input) {
    return new Translation2d(
        map.applyThrottleAbs(input.getX()), map.applyThrottleAbs(input.getY()));
  }
}
