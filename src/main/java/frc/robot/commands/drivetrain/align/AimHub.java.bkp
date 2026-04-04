package frc.robot.commands.drivetrain.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.superstructure.SuperStructure;
import org.littletonrobotics.junction.Logger;

public class AimHub extends Command {
  private final Drivetrain drivetrain;
  private final SuperStructure superStructure;
  private final PIDController thetaController;

  public AimHub(Drivetrain drivetrain, SuperStructure superStructure) {
    this.drivetrain = drivetrain;
    this.superStructure = superStructure;

    this.thetaController =
        new PIDController(
            DrivetrainConstants.ANGLE_KP,
            DrivetrainConstants.ANGLE_KI,
            DrivetrainConstants.ANGLE_KD);

    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.thetaController.setTolerance(Math.toRadians(1), Math.toRadians(1.0));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    thetaController.reset();
  }

  @Override
  public void execute() {
    Rotation2d currentRotation = drivetrain.getPose().getRotation();
    Rotation2d targetHeading = superStructure.getPredictiveAimAngle();

    double omega =
        thetaController.calculate(currentRotation.getRadians(), targetHeading.getRadians());

    drivetrain.driveFieldRelative(new ChassisSpeeds(0, 0, omega));

    Logger.recordOutput("Commands/Drivetrain/AimOnly/TargetHeadingDeg", targetHeading.getDegrees());
    Logger.recordOutput(
        "Commands/Drivetrain/AimOnly/RotationErrorDeg", Math.toDegrees(thetaController.getError()));
  }

  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
