package org.ciprix.util.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;

@FunctionalInterface
public interface TranslationModifier {
  Translation2d apply(DriveContext drivetrain, Translation2d input);
}
