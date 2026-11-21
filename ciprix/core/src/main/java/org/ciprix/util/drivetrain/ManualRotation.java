package org.ciprix.util.drivetrain;

import java.util.function.DoubleSupplier;

public class ManualRotation implements RotationStrategy {
  private final DoubleSupplier stickSupplier;
  private final DoubleSupplier maxSpeed;

  public ManualRotation(DoubleSupplier stickSupplier, DoubleSupplier maxSpeed) {
    this.stickSupplier = stickSupplier;
    this.maxSpeed = maxSpeed;
  }

  @Override
  public double calculate(DriveContext drivetrain) {
    double val = stickSupplier.getAsDouble();
    return Math.copySign(val * val, val) * maxSpeed.getAsDouble();
  }
}
