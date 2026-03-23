package frc.robot.commands.drivetrain.align;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.calculus.TunableControls.ControlConstants;
import frc.lib.calculus.TunableControls.TunableControlConstants;
import frc.lib.calculus.TunableControls.TunablePIDController;
import frc.robot.Constants;
import frc.robot.commands.CommandConstants.IntakeBallGeneralConstants;
import frc.robot.commands.CommandConstants.IntakeBallHConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.VisionCamera;
import org.littletonrobotics.junction.Logger;

public class IntakeBall extends Command {

  private final Drivetrain drivetrain;
  private final Vision vision;

  // Ajuste este valor: quanto maior, mais ele "freia" a oscilação
  private static final double K_DAMPING = 1.4;

  private static final ControlConstants kRotationConstants =
      new ControlConstants()
          .withPID(IntakeBallHConstants.k_P, IntakeBallHConstants.k_I, IntakeBallHConstants.k_D)
          .withTolerance(IntakeBallGeneralConstants.H_TOLERANCE);

  private final TunableControlConstants rotationTunables;
  private final TunablePIDController rotationController;
  private final double rotationSetpoint = IntakeBallGeneralConstants.H_SETPOINT;
  private static final double FORWARD_VELOCITY_MPS = 1.0;

  public IntakeBall(Drivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    addRequirements(drivetrain);

    this.rotationTunables = new TunableControlConstants("IntakeBall/Rotation", kRotationConstants);
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
      Logger.recordOutput("IntakeBall/Setpoint", rotationSetpoint);
      Logger.recordOutput("IntakeBall/CurrentTX", vision.getTx(VisionCamera.FRONT));
    }

    double tx = vision.getTx(VisionCamera.FRONT);
    double pidOutput = rotationController.calculate(tx, rotationSetpoint);

    // COMPENSAÇÃO PREDITIVA: Usa a velocidade real do drivetrain para "prever" o overshoot
    double currentAngularVel = drivetrain.getRobotVelocity().omegaRadiansPerSecond;
    double omega = pidOutput - (currentAngularVel * K_DAMPING);

    drivetrain.driveRobotRelative(0, -FORWARD_VELOCITY_MPS, omega);

    Logger.recordOutput("IntakeBall/Err/H", rotationController.getPositionError());
    Logger.recordOutput("IntakeBall/V/OmegaRaw", pidOutput);
    Logger.recordOutput("IntakeBall/V/OmegaCompensated", omega);
    Logger.recordOutput("IntakeBall/AtGoal", rotationController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
