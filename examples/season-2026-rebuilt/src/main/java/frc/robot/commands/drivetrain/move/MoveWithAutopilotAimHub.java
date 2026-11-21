package frc.robot.commands.drivetrain.move;

import static edu.wpi.first.units.Units.*;

import com.therekrab.autopilot.*;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.superstructure.SuperStructure;
import org.littletonrobotics.junction.Logger;

public class MoveWithAutopilotAimHub extends Command {
  private final Drivetrain drivetrain;
  private final SuperStructure superStructure;
  private final Pose2d targetPose;

  private boolean translationConverged = false;

  private final PIDController thetaController;

  private final APConstraints apConstraints =
      new APConstraints()
          .withAcceleration(DrivetrainConstants.MAX_ACCELERATION)
          .withJerk(DrivetrainConstants.MAX_JERK);

  private final APProfile apProfile =
      new APProfile(apConstraints)
          .withErrorXY(Centimeters.of(2))
          .withErrorTheta(Degrees.of(1))
          .withBeelineRadius(Meters.of(0.35));

  private final Autopilot autopilot = new Autopilot(apProfile);

  public MoveWithAutopilotAimHub(
      Drivetrain drivetrain, SuperStructure superStructure, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.superStructure = superStructure;
    this.targetPose = targetPose;

    thetaController =
        new PIDController(
            DrivetrainConstants.ANGLE_KP,
            DrivetrainConstants.ANGLE_KI,
            DrivetrainConstants.ANGLE_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.toRadians(1), Math.toRadians(1.0));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    thetaController.reset();

    Logger.recordOutput("Commands/Drivetrain/AutopilotAim/Target/X", targetPose.getX());
    Logger.recordOutput("Commands/Drivetrain/AutopilotAim/Target/Y", targetPose.getY());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    ChassisSpeeds robotRelativeSpeeds = drivetrain.getState().Speeds;

    double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double distanceTolerance = apProfile.getErrorXY().in(Meters);
    translationConverged = distanceToTarget < distanceTolerance;

    APTarget target = new APTarget(targetPose);
    double vx, vy;

    if (translationConverged) {
      vx = 0;
      vy = 0;
    } else {
      APResult output = autopilot.calculate(currentPose, robotRelativeSpeeds, target);
      vx = output.vx().in(MetersPerSecond);
      vy = output.vy().in(MetersPerSecond);
    }

    Rotation2d targetHeading = superStructure.getActiveShotParameters().aimAngle();

    double omega =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetHeading.getRadians());

    drivetrain.driveFieldRelative(new ChassisSpeeds(vx, vy, omega));

    Logger.recordOutput(
        "Commands/Drivetrain/AutopilotAim/IsTranslationConverged", translationConverged);
    Logger.recordOutput("Commands/Drivetrain/AutopilotAim/DistanceError", distanceToTarget);
    Logger.recordOutput(
        "Commands/Drivetrain/AutopilotAim/TargetHeadingDeg", targetHeading.getDegrees());
  }

  @Override
  public boolean isFinished() {
    boolean translationFinished = translationConverged;

    boolean rotationFinished = thetaController.atSetpoint();

    Logger.recordOutput("Commands/Drivetrain/AutopilotAim/Fin/Trans", translationFinished);
    Logger.recordOutput("Commands/Drivetrain/AutopilotAim/Fin/Rot", rotationFinished);

    return translationFinished && rotationFinished;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  public boolean isAligned() {
    return superStructure.isAtSetpointAngle();
  }
}
