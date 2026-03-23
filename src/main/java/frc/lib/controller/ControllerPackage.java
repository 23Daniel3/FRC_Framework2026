package frc.lib.controller;

import java.util.function.Supplier;

public class ControllerPackage {
  private Supplier<Double> leftX;
  private Supplier<Double> leftY;
  private Supplier<Double> rightX;

  public ControllerPackage() {}

  public void setValues(Supplier<Double> leftX, Supplier<Double> leftY, Supplier<Double> rightX) {
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
  }

  public double getLeftX() {
    return leftX.get();
  }

  public double getLeftY() {
    return leftY.get();
  }

  public double getRightX() {
    return rightX.get();
  }
}
