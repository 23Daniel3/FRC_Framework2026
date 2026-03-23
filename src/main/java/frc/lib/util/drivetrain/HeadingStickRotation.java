package frc.lib.util.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

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
  public void reset(Drivetrain drivetrain) {
    lastTarget = drivetrain.getRotation();
    pidStrategy.reset(drivetrain);
  }

  @Override
  public double calculate(Drivetrain drivetrain) {
    double x = xStick.getAsDouble();
    double y = yStick.getAsDouble();

    if (Math.hypot(x, y) > Constants.CONTROLLER_DEADBAND) {
      lastTarget = new Rotation2d(Math.atan2(x, y));
      lastTarget = AllianceFlipUtil.apply(lastTarget, true);
    }

    if (lastTarget == null) lastTarget = drivetrain.getRotation();

    return pidStrategy.calculate(drivetrain);
  }
}
