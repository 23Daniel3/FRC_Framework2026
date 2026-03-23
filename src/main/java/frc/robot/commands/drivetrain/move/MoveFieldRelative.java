package frc.robot.commands.drivetrain.move;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.MoveHConstants;
import frc.robot.commands.CommandConstants.MoveXConstants;
import frc.robot.commands.CommandConstants.MoveYConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class MoveFieldRelative extends Command {
  private final Drivetrain drivetrain;
  private final Pose2d targetPose;
  private final PathConstraints constraints;

  // Tunáveis em tempo real
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

  // Controllers profilados
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  public MoveFieldRelative(Drivetrain drivetrain, Pose2d targetPose, PathConstraints constraints) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    this.constraints = constraints;

    // Extrai scalars das measures
    double maxTransVel = constraints.maxVelocity().abs(MetersPerSecond);
    double maxTransAccel = constraints.maxAcceleration().abs(MetersPerSecondPerSecond);
    double maxAngVel = constraints.maxAngularVelocity().abs(RadiansPerSecond);
    double maxAngAccel = constraints.maxAngularAcceleration().abs(RadiansPerSecondPerSecond);

    // Controllers com perfil trapezoidal
    xController =
        new ProfiledPIDController(
            kXP.get(),
            kXI.get(),
            kXD.get(),
            new TrapezoidProfile.Constraints(maxTransVel, maxTransAccel));
    yController =
        new ProfiledPIDController(
            kYP.get(),
            kYI.get(),
            kYD.get(),
            new TrapezoidProfile.Constraints(maxTransVel, maxTransAccel));
    thetaController =
        new ProfiledPIDController(
            kHP.get(),
            kHI.get(),
            kHD.get(),
            new TrapezoidProfile.Constraints(maxAngVel, maxAngAccel));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Tolerâncias
    xController.setTolerance(0.01, 0.0);
    yController.setTolerance(0.01, 0.0);
    thetaController.setTolerance(Math.toRadians(1), Math.toRadians(1.0));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d odo = drivetrain.getPose();
    xController.reset(odo.getX(), 0.0);
    yController.reset(odo.getY(), 0.0);
    thetaController.reset(odo.getRotation().getRadians(), 0.0);

    // Log Inicial de Constraints e Setpoints
    Logger.recordOutput(
        "/GoToFieldPose/TransMaxVel", constraints.maxVelocity().abs(MetersPerSecond));
    Logger.recordOutput(
        "/GoToFieldPose/TransMaxAccel",
        constraints.maxAcceleration().abs(MetersPerSecondPerSecond));
    Logger.recordOutput(
        "/GoToFieldPose/RotMaxVel", constraints.maxAngularVelocity().abs(RadiansPerSecond));
    Logger.recordOutput(
        "/GoToFieldPose/RotMaxAccel",
        constraints.maxAngularAcceleration().abs(RadiansPerSecondPerSecond));

    Logger.recordOutput("/GoToFieldPose/Setpoint/X", targetPose.getX());
    Logger.recordOutput("/GoToFieldPose/Setpoint/Y", targetPose.getY());
    Logger.recordOutput("/GoToFieldPose/Setpoint/H", targetPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    Pose2d cur = drivetrain.getPose();

    // Atualiza ganhos dinamicamente
    xController.setPID(kXP.get(), kXI.get(), kXD.get());
    yController.setPID(kYP.get(), kYI.get(), kYD.get());
    thetaController.setPID(kHP.get(), kHI.get(), kHD.get());

    // Saídas perfiladas
    double vx = xController.calculate(cur.getX(), targetPose.getX());
    double vy = yController.calculate(cur.getY(), targetPose.getY());
    double omega =
        thetaController.calculate(
            cur.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, drivetrain.getRotation());

    drivetrain.driveFieldRelative(speeds);

    // Logging completo para tuning
    Logger.recordOutput("/GoToFieldPose/Cur/X", cur.getX());
    Logger.recordOutput("/GoToFieldPose/Cur/Y", cur.getY());
    Logger.recordOutput("/GoToFieldPose/Cur/H", cur.getRotation().getDegrees());
    Logger.recordOutput("/GoToFieldPose/Err/X", targetPose.getX() - cur.getX());
    Logger.recordOutput("/GoToFieldPose/Err/Y", targetPose.getY() - cur.getY());
    Logger.recordOutput(
        "/GoToFieldPose/Err/H", targetPose.getRotation().minus(cur.getRotation()).getDegrees());
    Logger.recordOutput("/GoToFieldPose/Vx", vx);
    Logger.recordOutput("/GoToFieldPose/Vy", vy);
    Logger.recordOutput("/GoToFieldPose/Omega", Math.toDegrees(omega));
    Logger.recordOutput(
        "/GoToFieldPose/AtGoal",
        xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
