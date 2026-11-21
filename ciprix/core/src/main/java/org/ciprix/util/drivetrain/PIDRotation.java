package org.ciprix.util.drivetrain;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ciprix.CiprixSettings;
import org.ciprix.logger.LoggedTunableNumber;

/**
 * Rotation strategy that holds a target heading with a PID controller, optionally giving control
 * back to the driver while the rotation stick is outside the deadband.
 *
 * <p>The gains are shared by every instance and tunable under {@code Drive/RotationPID}. Call
 * {@link #setDefaultGains(double, double, double)} once, before creating the first instance, to use
 * the gains of your drivetrain.
 */
public class PIDRotation implements RotationStrategy {
  // Tunable gains via NetworkTables/AdvantageKit, shared by every instance.
  private static final LoggedTunableNumber kp = new LoggedTunableNumber("Drive/RotationPID/kP");
  private static final LoggedTunableNumber ki = new LoggedTunableNumber("Drive/RotationPID/kI");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("Drive/RotationPID/kD");

  private static double defaultKp = 10.0;
  private static double defaultKi = 0.0;
  private static double defaultKd = 0.0;

  private final PIDController controller;
  private final Supplier<Rotation2d> targetSupplier;
  private final DoubleSupplier manualControl;

  /**
   * Sets the default gains of the heading controller. Only takes effect when called before the
   * first {@code PIDRotation} is created.
   */
  public static void setDefaultGains(double kP, double kI, double kD) {
    defaultKp = kP;
    defaultKi = kI;
    defaultKd = kD;
  }

  public PIDRotation(Supplier<Rotation2d> targetSupplier) {
    this(targetSupplier, null);
  }

  public PIDRotation(Supplier<Rotation2d> targetSupplier, DoubleSupplier manualControl) {
    this.targetSupplier = targetSupplier;
    this.manualControl = manualControl;

    kp.initDefault(defaultKp);
    ki.initDefault(defaultKi);
    kd.initDefault(defaultKd);

    // Initialize standard PID controller
    this.controller = new PIDController(kp.get(), ki.get(), kd.get());

    // Essential for rotation: enables continuous input so -179° and 179° are treated as adjacent
    this.controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void reset(DriveContext drivetrain) {
    controller.reset();
  }

  @Override
  public double calculate(DriveContext drivetrain) {
    // Update gains if they have been changed on the Dashboard
    updatePIDConstants();

    if (manualControl != null) {
      double stick = manualControl.getAsDouble();
      if (Math.abs(stick) > CiprixSettings.getControllerDeadband()) {
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
    int id = this.hashCode();
    // Evaluate the three tunables (no short-circuit) so each one records its latest value.
    boolean changed = kp.hasChanged(id) | ki.hasChanged(id) | kd.hasChanged(id);
    if (changed) {
      controller.setPID(kp.get(), ki.get(), kd.get());
    }
  }
}
