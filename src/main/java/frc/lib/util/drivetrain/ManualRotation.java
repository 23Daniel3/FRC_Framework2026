package frc.lib.util.drivetrain;

import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class ManualRotation implements RotationStrategy {
  private final DoubleSupplier stickSupplier;
  private final DoubleSupplier maxSpeed;

  public ManualRotation(DoubleSupplier stickSupplier, DoubleSupplier maxSpeed) {
    this.stickSupplier = stickSupplier;
    this.maxSpeed = maxSpeed;
  }

  @Override
  public double calculate(Drivetrain drivetrain) {
    double val = stickSupplier.getAsDouble();
    return Math.copySign(val * val, val) * maxSpeed.getAsDouble();
  }
}
