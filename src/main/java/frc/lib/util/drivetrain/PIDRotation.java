package frc.lib.util.drivetrain;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logger.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PIDRotation implements RotationStrategy {
  private final PIDController controller;
  private final Supplier<Rotation2d> targetSupplier;
  private final DoubleSupplier manualControl;

  // Tunable gains via NetworkTables/AdvantageKit
  private final LoggedTunableNumber kp =
      new LoggedTunableNumber("Drive/RotationPID/kP", DrivetrainConstants.ANGLE_KP);
  private final LoggedTunableNumber ki =
      new LoggedTunableNumber("Drive/RotationPID/kI", DrivetrainConstants.ANGLE_KI);
  private final LoggedTunableNumber kd =
      new LoggedTunableNumber("Drive/RotationPID/kD", DrivetrainConstants.ANGLE_KD);

  public PIDRotation(Supplier<Rotation2d> targetSupplier) {
    this(targetSupplier, null);
  }

  public PIDRotation(Supplier<Rotation2d> targetSupplier, DoubleSupplier manualControl) {
    this.targetSupplier = targetSupplier;
    this.manualControl = manualControl;

    // Initialize standard PID controller
    this.controller = new PIDController(kp.get(), ki.get(), kd.get());

    // Essential for rotation: enables continuous input so -179° and 179° are treated as adjacent
    this.controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void reset(Drivetrain drivetrain) {
    controller.reset();
  }

  @Override
  public double calculate(Drivetrain drivetrain) {
    // Update gains if they have been changed on the Dashboard
    updatePIDConstants();

    if (manualControl != null) {
      double stick = manualControl.getAsDouble();
      if (Math.abs(stick) > Constants.CONTROLLER_DEADBAND) {
        // Manual control with exponential curve for sensitivity
        return Math.copySign(stick * stick, stick)
            * drivetrain.getMaxAngularSpeed().in(RadiansPerSecond);
      }
    }

    // Simple PID calculation (Target vs Current)
    return controller.calculate(
        drivetrain.getRotation().getRadians(), targetSupplier.get().getRadians());
  }

  /** Checks if tunable numbers have changed and updates the controller. */
  private void updatePIDConstants() {
    if (kp.hasChanged(this.hashCode())
        || ki.hasChanged(this.hashCode())
        || kd.hasChanged(this.hashCode())) {
      controller.setPID(kp.get(), ki.get(), kd.get());
    }
  }
}
