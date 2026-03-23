package frc.lib.util.drivetrain;

import frc.robot.subsystems.drivetrain.Drivetrain;

@FunctionalInterface
public interface RotationStrategy {
  double calculate(Drivetrain drivetrain);

  default void reset(Drivetrain drivetrain) {}
}
