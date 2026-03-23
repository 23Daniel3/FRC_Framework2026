package frc.lib.util.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.Supplier;

public class TargetAssistModifier implements TranslationModifier {
  private final Supplier<Translation2d> targetSupplier;
  private final double weight; // 0.0 to 1.0

  public TargetAssistModifier(Supplier<Translation2d> targetSupplier, double weight) {
    this.targetSupplier = targetSupplier;
    this.weight = weight;
  }

  @Override
  public Translation2d apply(Drivetrain drivetrain, Translation2d joystick) {
    Translation2d current = drivetrain.getPose().getTranslation();
    Translation2d target = targetSupplier.get();
    Translation2d vectorToTarget = target.minus(current);

    if (vectorToTarget.getNorm() > 0.01)
      vectorToTarget = vectorToTarget.div(vectorToTarget.getNorm());
    else vectorToTarget = new Translation2d();

    Translation2d blended = joystick.times(1.0 - weight).plus(vectorToTarget.times(weight));

    if (blended.getNorm() > 1.0) blended = blended.div(blended.getNorm());
    return blended;
  }
}
