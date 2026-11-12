package org.ciprix.util.drivetrain;

@FunctionalInterface
public interface RotationStrategy {
  double calculate(DriveContext drivetrain);

  default void reset(DriveContext drivetrain) {}
}
