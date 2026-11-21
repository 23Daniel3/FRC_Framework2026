package org.ciprix.util.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import org.ciprix.CiprixSettings;
import org.ciprix.util.AllianceFlipUtil;

public class HeadingStickRotation implements RotationStrategy {
  private final PIDRotation pidStrategy;
  private final DoubleSupplier xStick, yStick;
  private Rotation2d lastTarget = null;

  public HeadingStickRotation(DoubleSupplier xStick, DoubleSupplier yStick) {
    this.xStick = xStick;
    this.yStick = yStick;
    this.pidStrategy = new PIDRotation(() -> lastTarget);
  }

  @Override
  public void reset(DriveContext drivetrain) {
    lastTarget = drivetrain.getRotation();
    pidStrategy.reset(drivetrain);
  }

  @Override
  public double calculate(DriveContext drivetrain) {
    double x = xStick.getAsDouble();
    double y = yStick.getAsDouble();

    if (Math.hypot(x, y) > CiprixSettings.getControllerDeadband()) {
      lastTarget = new Rotation2d(Math.atan2(x, y));
      lastTarget = AllianceFlipUtil.apply(lastTarget, true);
    }

    if (lastTarget == null) lastTarget = drivetrain.getRotation();

    return pidStrategy.calculate(drivetrain);
  }
}
