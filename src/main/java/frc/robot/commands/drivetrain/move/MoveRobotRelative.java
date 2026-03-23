package frc.robot.commands.drivetrain.move;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ChassisSpeedsLimiter;
import frc.robot.commands.CommandConstants.MoveHConstants;
import frc.robot.commands.CommandConstants.MoveXConstants;
import frc.robot.commands.CommandConstants.MoveYConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class MoveRobotRelative extends Command {
  private final Drivetrain drivetrain;
  private final HolonomicDriveController controller;

  private Pose2d targetPose;
  private final double deltaX;
  private final double deltaY;
  private final Double targetHeadingDegrees;

  private final LoggedNetworkNumber kXP =
      new LoggedNetworkNumber("/Tuning/X/KP", MoveXConstants.k_P);
  private final LoggedNetworkNumber kXI =
      new LoggedNetworkNumber("/Tuning/X/KI", MoveXConstants.k_I);
  private final LoggedNetworkNumber kXD =
      new LoggedNetworkNumber("/Tuning/X/KD", MoveXConstants.k_D);

  private final LoggedNetworkNumber kYP =
      new LoggedNetworkNumber("/Tuning/Y/KP", MoveYConstants.k_P);
  private final LoggedNetworkNumber kYI =
      new LoggedNetworkNumber("/Tuning/Y/KI", MoveYConstants.k_I);
  private final LoggedNetworkNumber kYD =
      new LoggedNetworkNumber("/Tuning/Y/KD", MoveYConstants.k_D);

  private final LoggedNetworkNumber kHP =
      new LoggedNetworkNumber("/Tuning/H/KP", MoveHConstants.k_P);
  private final LoggedNetworkNumber kHI =
      new LoggedNetworkNumber("/Tuning/H/KI", MoveHConstants.k_I);
  private final LoggedNetworkNumber kHD =
      new LoggedNetworkNumber("/Tuning/H/KD", MoveHConstants.k_D);

  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private final ChassisSpeedsLimiter ramp = new ChassisSpeedsLimiter();

  private Pose2d currentPose;
  private Pose2d inicialPose;

  public MoveRobotRelative(
      Drivetrain drivetrain, double deltaX, double deltaY, Double targetHeadingDegrees) {

    this.drivetrain = drivetrain;
    this.deltaX = deltaX;
    this.deltaY = deltaY;
    this.targetHeadingDegrees = targetHeadingDegrees;

    xController = new PIDController(kXP.get(), kXI.get(), kXD.get());
    yController = new PIDController(kYP.get(), kYI.get(), kYD.get());

    thetaController =
        new ProfiledPIDController(
            kHP.get(),
            kHI.get(),
            kHD.get(),
            new TrapezoidProfile.Constraints(Math.toRadians(360), Math.toRadians(720)));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(xController, yController, thetaController);

    controller.setTolerance(new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(2.0)));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("/MoveXYHeading/X/Setpoint", deltaX);
    Logger.recordOutput("/MoveXYHeading/Y/Setpoint", deltaY);
    Logger.recordOutput("/MoveXYHeading/H/Setpoint", targetHeadingDegrees);

    xController.reset();
    yController.reset();
    thetaController.reset(drivetrain.getRotation().getRadians());

    xController.setPID(kXP.get(), kXI.get(), kXD.get());
    yController.setPID(kYP.get(), kYI.get(), kYD.get());
    thetaController.setPID(kHP.get(), kHI.get(), kHD.get());

    currentPose = drivetrain.getPose();
    inicialPose = currentPose;

    Translation2d robotRelativeDelta = new Translation2d(deltaX, deltaY);

    Translation2d fieldRelativeDelta = robotRelativeDelta.rotateBy(currentPose.getRotation());

    Translation2d newTranslation = currentPose.getTranslation().plus(fieldRelativeDelta);

    Rotation2d newRotation =
        (targetHeadingDegrees != null)
            ? currentPose.getRotation().plus(Rotation2d.fromDegrees(targetHeadingDegrees))
            : currentPose.getRotation();

    targetPose = new Pose2d(newTranslation, newRotation);
  }

  @Override
  public void execute() {
    xController.setPID(kXP.get(), kXI.get(), kXD.get());
    yController.setPID(kYP.get(), kYI.get(), kYD.get());
    thetaController.setPID(kHP.get(), kHI.get(), kHD.get());

    currentPose = drivetrain.getPose();

    ChassisSpeeds targetSpeeds =
        controller.calculate(currentPose, targetPose, 0.0, targetPose.getRotation());

    ChassisSpeeds rampSpeed = ramp.calculate(targetSpeeds);

    drivetrain.driveRobotRelative(rampSpeed);

    logValues(targetSpeeds, rampSpeed);
  }

  @Override
  public boolean isFinished() {
    return controller.atReference();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  private void logValues(ChassisSpeeds targetSpeeds, ChassisSpeeds rampProfileSpeed) {

    Logger.recordOutput("/MoveXYHeading/X/CurrentPoseValue", currentPose.getX());
    Logger.recordOutput("/MoveXYHeading/Y/CurrentPoseValue", currentPose.getY());
    Logger.recordOutput(
        "/MoveXYHeading/H/CurrentPoseValue", currentPose.getRotation().getDegrees());

    Logger.recordOutput(
        "/MoveXYHeading/X/RelativePoseValue", currentPose.getX() - inicialPose.getX());
    Logger.recordOutput(
        "/MoveXYHeading/Y/RelativePoseValue", currentPose.getY() - inicialPose.getY());
    Logger.recordOutput(
        "/MoveXYHeading/H/RelativePoseValue",
        currentPose.getRotation().getDegrees() - inicialPose.getRotation().getDegrees());

    Logger.recordOutput("/MoveXYHeading/X/Error", targetPose.getX() - currentPose.getX());
    Logger.recordOutput("/MoveXYHeading/Y/Error", targetPose.getY() - currentPose.getY());
    Logger.recordOutput(
        "/MoveXYHeading/H/Error",
        targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());

    Logger.recordOutput("/MoveXYHeading/X/Vx Setpoint", targetSpeeds.vxMetersPerSecond);
    Logger.recordOutput("/MoveXYHeading/Y/Vy Setpoint", targetSpeeds.vyMetersPerSecond);
    Logger.recordOutput("/MoveXYHeading/H/VOmega Setpoint", targetSpeeds.omegaRadiansPerSecond);

    Logger.recordOutput("/MoveXYHeading/X/Vx Setpoint ramp", rampProfileSpeed.vxMetersPerSecond);
    Logger.recordOutput("/MoveXYHeading/Y/Vy Setpoint ramp", rampProfileSpeed.vyMetersPerSecond);
    Logger.recordOutput(
        "/MoveXYHeading/H/VOmega Setpoint ramp", rampProfileSpeed.omegaRadiansPerSecond);

    Logger.recordOutput(
        "/MoveXYHeading/DistanceToTarget",
        currentPose.getTranslation().getDistance(targetPose.getTranslation()));

    Logger.recordOutput("/MoveXYHeading/AtSetpoint", controller.atReference());
  }
}
