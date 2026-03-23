package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.game.AllianceManager;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.DoubleSupplier;

public class JoystickDriveShooting extends Command {
  private final Drivetrain drivetrain;
  private final SuperStructure superStructure;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final AllianceManager allianceManager = AllianceManager.getInstance();

  private final PIDController rotationController =
      new PIDController(
          DrivetrainConstants.ANGLE_KP, DrivetrainConstants.ANGLE_KI, DrivetrainConstants.ANGLE_KD);

  public JoystickDriveShooting(
      Drivetrain drivetrain,
      SuperStructure superStructure,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    this.drivetrain = drivetrain;
    this.superStructure = superStructure;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    rotationController.reset();
  }

  @Override
  public void execute() {
    double xController = xSupplier.getAsDouble();
    double yController = ySupplier.getAsDouble();

    if (allianceManager.myAlliance() == Alliance.Red) {
      xController = -xController;
      yController = -yController;
    }

    Rotation2d targetAngle = superStructure.calculateDynamicTargetAngle();

    double angularSpeed =
        rotationController.calculate(
            drivetrain.getRotation().getRadians(), targetAngle.getRadians());

    drivetrain.driveFieldRelative(
        xController * drivetrain.getMaxSpeed().in(MetersPerSecond),
        yController * drivetrain.getMaxSpeed().in(MetersPerSecond),
        angularSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
