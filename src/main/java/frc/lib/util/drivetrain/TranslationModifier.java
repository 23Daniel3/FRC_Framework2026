package frc.lib.util.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

@FunctionalInterface
public interface TranslationModifier {
  Translation2d apply(Drivetrain drivetrain, Translation2d input);
}
