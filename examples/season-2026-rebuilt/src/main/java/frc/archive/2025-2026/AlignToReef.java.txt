package frc.robot.commands.drivetrain.align;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.calculus.TunableControls.ControlConstants;
import frc.lib.calculus.TunableControls.TunableControlConstants;
import frc.lib.calculus.TunableControls.TunablePIDController;
import frc.lib.logger.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.commands.CommandConstants.AlignToReefGeneralConstants;
import frc.robot.commands.CommandConstants.AlignToReefHConstants;
import frc.robot.commands.CommandConstants.AlignToReefXConstants;
import frc.robot.commands.CommandConstants.AlignToReefYConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class AlignToReef extends Command {

  private static final TunableControlConstants xConstants =
      new TunableControlConstants(
          "AlignToReef/X_Ctrl",
          new ControlConstants()
              .withPID(
                  AlignToReefXConstants.k_P, AlignToReefXConstants.k_I, AlignToReefXConstants.k_D)
              .withTolerance(AlignToReefGeneralConstants.X_TOLERANCE));

  private static final TunableControlConstants yConstants =
      new TunableControlConstants(
          "AlignToReef/Y_Ctrl",
          new ControlConstants()
              .withPID(
                  AlignToReefYConstants.k_P, AlignToReefYConstants.k_I, AlignToReefYConstants.k_D)
              .withTolerance(AlignToReefGeneralConstants.Y_TOLERANCE));

  private static final TunableControlConstants hConstants =
      new TunableControlConstants(
          "AlignToReef/H_Ctrl",
          new ControlConstants()
              .withPID(
                  AlignToReefHConstants.k_P, AlignToReefHConstants.k_I, AlignToReefHConstants.k_D)
              .withTolerance(AlignToReefGeneralConstants.H_TOLERANCE)
              .withContinuous(-Math.PI, Math.PI));

  private static final LoggedTunableNumber spX =
      new LoggedTunableNumber("AlignToReef/Setpoints/X", 1.0);
  private static final LoggedTunableNumber spY_Right =
      new LoggedTunableNumber("AlignToReef/Setpoints/Y_Right", -0.5);
  private static final LoggedTunableNumber spY_Left =
      new LoggedTunableNumber("AlignToReef/Setpoints/Y_Left", 0.5);
  private static final LoggedTunableNumber spH =
      new LoggedTunableNumber("AlignToReef/Setpoints/H", 0.0);

  private final Drivetrain drivetrain;
  private final boolean isRightScore;

  private final TunablePIDController xController = new TunablePIDController(xConstants);
  private final TunablePIDController yController = new TunablePIDController(yConstants);
  private final TunablePIDController hController = new TunablePIDController(hConstants);

  private final LoggedNetworkBoolean useX = new LoggedNetworkBoolean("AlignToReef/Use/X", true);
  private final LoggedNetworkBoolean useY = new LoggedNetworkBoolean("AlignToReef/Use/Y", true);
  private final LoggedNetworkBoolean useH = new LoggedNetworkBoolean("AlignToReef/Use/H", true);

  private final Timer timer = new Timer();

  private enum AlignPhase {
    START,
    BACKING,
    FORWARD_ALIGN,
    DONE
  }

  private AlignPhase phase = AlignPhase.START;

  public AlignToReef(boolean isRightScore, Drivetrain drivetrain) {
    this.isRightScore = isRightScore;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    hController.reset();

    xController.updateParams();
    yController.updateParams();
    hController.updateParams();

    phase = AlignPhase.START;
    timer.restart();
  }

  @Override
  public void execute() {
    boolean seeTag = LimelightHelpers.getTV("limelight-left");
    Logger.recordOutput("/AlignToReef/SeeTag", seeTag);
    if (!seeTag) {
      cancel();
      return;
    }

    if (Constants.tuningMode) {
      xController.updateParams();
      yController.updateParams();
      hController.updateParams();
    }

    double[] pose = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
    double forwardDistance = pose[0];
    double lateralDistance = pose[1];
    double headingDistance = pose[5];

    double vx = 0, vy = 0, omega;

    omega = useH.get() ? -hController.calculate(headingDistance, spH.get()) : 0.0;

    switch (phase) {
      case START:
        if (Math.abs(forwardDistance - spX.get()) < 0.3) {
          phase = AlignPhase.BACKING;
        } else {
          phase = AlignPhase.FORWARD_ALIGN;
        }
        break;

      case BACKING:
        vx = -0.7;
        vy =
            useY.get()
                ? -yController.calculate(
                    lateralDistance, isRightScore ? spY_Right.get() : spY_Left.get())
                : 0.0;

        if (Math.abs(forwardDistance - spX.get()) >= 0.3) {
          phase = AlignPhase.FORWARD_ALIGN;
        }
        break;

      case FORWARD_ALIGN:
        vx = useX.get() ? xController.calculate(forwardDistance, spX.get()) : 0.0;
        vy =
            useY.get()
                ? -yController.calculate(
                    lateralDistance, isRightScore ? spY_Right.get() : spY_Left.get())
                : 0.0;

        if (xController.atSetpoint() && yController.atSetpoint() && hController.atSetpoint()) {
          phase = AlignPhase.DONE;
        }
        break;

      case DONE:
        vx = 0;
        vy = 0;
        omega = 0;
        break;
    }

    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);

    drivetrain.driveRobotRelative(speeds);

    Logger.recordOutput("/AlignToReef/Phase", phase);
    Logger.recordOutput("/AlignToReef/Current/X", forwardDistance);
    Logger.recordOutput("/AlignToReef/Current/Y", lateralDistance);
    Logger.recordOutput("/AlignToReef/Current/H", headingDistance);

    Logger.recordOutput("/AlignToReef/Err/X", xController.getPositionError());
    Logger.recordOutput("/AlignToReef/Err/Y", yController.getPositionError());
    Logger.recordOutput("/AlignToReef/Err/H", hController.getPositionError());

    Logger.recordOutput("/AlignToReef/V/Vx", vx);
    Logger.recordOutput("/AlignToReef/V/Vy", vy);
    Logger.recordOutput("/AlignToReef/V/Omega", omega);
    Logger.recordOutput("/AlignToReef/AtGoal/X", xController.atSetpoint());
    Logger.recordOutput("/AlignToReef/AtGoal/Y", yController.atSetpoint());
    Logger.recordOutput("/AlignToReef/AtGoal/H", hController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return phase == AlignPhase.DONE;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
