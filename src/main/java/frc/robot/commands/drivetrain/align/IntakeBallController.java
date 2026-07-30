package frc.robot.commands.drivetrain.align;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.calculus.TunableControls.ControlConstants;
import frc.lib.calculus.TunableControls.TunableControlConstants;
import frc.lib.calculus.TunableControls.TunablePIDController;
import frc.lib.logger.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.commands.CommandConstants.IntakeBallGeneralConstants;
import frc.robot.commands.CommandConstants.IntakeBallHConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.VisionCamera;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeBallController extends Command {

  private final Drivetrain drivetrain;
  private final Vision vision;
  private DoubleSupplier controller;

  private static final double K_DAMPING = 1.4;

  private static final ControlConstants kRotationConstants =
      new ControlConstants()
          .withPID(IntakeBallHConstants.k_P, IntakeBallHConstants.k_I, IntakeBallHConstants.k_D)
          .withTolerance(IntakeBallGeneralConstants.H_TOLERANCE);

  private final TunableControlConstants rotationTunables;
  private final TunablePIDController rotationController;
  private final double rotationSetpoint = IntakeBallGeneralConstants.H_SETPOINT;

  private final LoggedTunableNumber kDamping =
      new LoggedTunableNumber("IntakeBallController/kDamping", K_DAMPING);

  public IntakeBallController(Drivetrain drivetrain, Vision vision, DoubleSupplier controller) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.controller = controller;
    addRequirements(drivetrain);

    this.rotationTunables =
        new TunableControlConstants("IntakeBallController/Rotation", kRotationConstants);
    this.rotationController = new TunablePIDController(rotationTunables);
  }

  @Override
  public void initialize() {
    rotationController.reset();
  }

  @Override
  public void execute() {
    if (Constants.tuningMode) {
      rotationController.updateParams();
      // Extra logs for tuning
      Logger.recordOutput("IntakeBallController/Setpoint", rotationSetpoint);
      Logger.recordOutput("IntakeBallController/CurrentTX", vision.getTx(VisionCamera.FRONT));
    }

    double tx = vision.getTx(VisionCamera.FRONT);
    double pidOutput = rotationController.calculate(tx, rotationSetpoint);

    // PREDICTIVE COMPENSATION: Subtract current angular velocity to damp motion
    double currentAngularVel = drivetrain.getRobotVelocity().omegaRadiansPerSecond;
    double omega = pidOutput - (currentAngularVel * kDamping.get());

    if (controller.getAsDouble() == 0.0) {
      controller = () -> 0.3;
    }
    drivetrain.driveRobotRelative(
        0, -controller.getAsDouble() * drivetrain.getMaxSpeed().in(MetersPerSecond), omega);

    Logger.recordOutput("IntakeBallController/Err/H", rotationController.getPositionError());
    Logger.recordOutput("IntakeBallController/V/OmegaRaw", pidOutput);
    Logger.recordOutput("IntakeBallController/V/OmegaCompensated", omega);
    Logger.recordOutput("IntakeBallController/AtGoal", rotationController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
