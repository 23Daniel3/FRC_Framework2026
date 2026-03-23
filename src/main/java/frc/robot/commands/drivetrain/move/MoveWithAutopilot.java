package frc.robot.commands.drivetrain.move;

import static edu.wpi.first.units.Units.*;

import com.therekrab.autopilot.*;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logger.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import org.littletonrobotics.junction.Logger;

public class MoveWithAutopilot extends Command {
  private final Drivetrain drivetrain;
  private final Pose2d targetPose;
  private final Rotation2d entryAngle;
  private boolean withEntryAngle;

  private final LoggedTunableNumber kHP =
      new LoggedTunableNumber(
          "/Tuning/MoveWithAutopilot/H/KP", DrivetrainConstants.PROFILED_PID_ANGLE_KP);
  private final LoggedTunableNumber kHI =
      new LoggedTunableNumber(
          "/Tuning/MoveWithAutopilot/H/KI", DrivetrainConstants.PROFILED_PID_ANGLE_KI);
  private final LoggedTunableNumber kHD =
      new LoggedTunableNumber(
          "/Tuning/MoveWithAutopilot/H/KD", DrivetrainConstants.PROFILED_PID_ANGLE_KD);

  private final ProfiledPIDController thetaController;

  private final APConstraints apConstraints =
      new APConstraints()
          .withAcceleration(DrivetrainConstants.MAX_ACCELERATION)
          .withJerk(DrivetrainConstants.MAX_JERK);

  private final APProfile apProfile =
      new APProfile(apConstraints)
          .withErrorXY(Centimeters.of(1))
          .withErrorTheta(Degrees.of(1))
          .withBeelineRadius(Meters.of(0.35));

  private final Autopilot autopilot = new Autopilot(apProfile);

  public MoveWithAutopilot(Drivetrain drivetrain, Pose2d targetPose, Rotation2d entryAngle) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    this.entryAngle = entryAngle;

    thetaController =
        new ProfiledPIDController(
            kHP.get(),
            kHI.get(),
            kHD.get(),
            new TrapezoidProfile.Constraints(
                DrivetrainConstants.MAX_ANGULAR_SPEED,
                DrivetrainConstants.MAX_ANGULAR_ACCELERATION));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.toRadians(1), Math.toRadians(1.0));

    withEntryAngle = true;

    addRequirements(drivetrain);
  }

  public MoveWithAutopilot(Drivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    this.entryAngle = new Rotation2d();

    withEntryAngle = false;

    thetaController =
        new ProfiledPIDController(
            kHP.get(),
            kHI.get(),
            kHD.get(),
            new TrapezoidProfile.Constraints(
                DrivetrainConstants.MAX_ANGULAR_SPEED,
                DrivetrainConstants.MAX_ANGULAR_ACCELERATION));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.toRadians(1), Math.toRadians(1.0));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d odo = drivetrain.getPose();
    thetaController.reset(
        odo.getRotation().getRadians(), drivetrain.getState().Speeds.omegaRadiansPerSecond);

    Logger.recordOutput("Commands/Drivetrain/Autopilot/Target/X", targetPose.getX());
    Logger.recordOutput("Commands/Drivetrain/Autopilot/Target/Y", targetPose.getY());
    Logger.recordOutput(
        "Commands/Drivetrain/Autopilot/Target/H", targetPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    ChassisSpeeds robotRelativeSpeeds = drivetrain.getState().Speeds;

    if (Constants.tuningMode) {
      thetaController.setPID(kHP.get(), kHI.get(), kHD.get());
    }

    APTarget target = new APTarget(targetPose);

    if (withEntryAngle) {
      target = target.withEntryAngle(entryAngle);
    }
    APResult output = autopilot.calculate(currentPose, robotRelativeSpeeds, target);

    double vx = output.vx().in(MetersPerSecond);
    double vy = output.vy().in(MetersPerSecond);

    Rotation2d targetHeading = output.targetAngle();
    double omega =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetHeading.getRadians());

    drivetrain.driveFieldRelative(new ChassisSpeeds(vx, vy, omega));

    Logger.recordOutput("Commands/Drivetrain/Autopilot/Current/X", currentPose.getX());
    Logger.recordOutput("Commands/Drivetrain/Autopilot/Current/Y", currentPose.getY());
    Logger.recordOutput("Commands/Drivetrain/Autopilot/Output/Vx", vx);
    Logger.recordOutput("Commands/Drivetrain/Autopilot/Output/Vy", vy);
    Logger.recordOutput("Commands/Drivetrain/Autopilot/Output/Omega", omega);
    Logger.recordOutput(
        "Commands/Drivetrain/Autopilot/Output/TargetHeading", targetHeading.getDegrees());

    Logger.recordOutput(
        "Commands/Drivetrain/Autopilot/Error/X", targetPose.getX() - currentPose.getX());
    Logger.recordOutput(
        "Commands/Drivetrain/Autopilot/Error/Y", targetPose.getY() - currentPose.getY());
  }

  @Override
  public boolean isFinished() {
    boolean translationFinished =
        autopilot.atTarget(
            drivetrain.getPose(), new APTarget(targetPose).withEntryAngle(entryAngle));
    boolean rotationFinished = thetaController.atGoal();

    Logger.recordOutput("Commands/Drivetrain/Autopilot/translationFinished", translationFinished);
    Logger.recordOutput("Commands/Drivetrain/Autopilot/rotationFinished", rotationFinished);
    return translationFinished && rotationFinished;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
