package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.drivetrain.DualZoneSuctionBumpModifier;
import frc.lib.util.drivetrain.HeadingStickRotation;
import frc.lib.util.drivetrain.ManualRotation;
import frc.lib.util.drivetrain.PIDRotation;
import frc.lib.util.drivetrain.RotationStrategy;
import frc.lib.util.drivetrain.TargetAssistModifier;
import frc.lib.util.drivetrain.ThrottleMapModifier;
import frc.lib.util.drivetrain.TranslationModifier;
import frc.lib.util.drivetrain.ZoneRepulsionModifier;
import frc.lib.util.drivetrain.ZoneSuctionModifier;
import frc.lib.zones.Polygon2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DrivetrainCommands {

  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private static final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private static Command driveCore(
      Drivetrain drivetrain,
      DoubleSupplier xStick,
      DoubleSupplier yStick,
      RotationStrategy rotationStrategy,
      TranslationModifier... modifiers) {

    return Commands.run(
            () -> {
              Translation2d translation =
                  new Translation2d(xStick.getAsDouble(), yStick.getAsDouble());

              for (TranslationModifier mod : modifiers) {
                translation = mod.apply(drivetrain, translation);
              }

              double omega = rotationStrategy.calculate(drivetrain);

              ChassisSpeeds speeds =
                  AllianceFlipUtil.apply(
                      drivetrain,
                      translation.getX() * drivetrain.getMaxSpeed().in(MetersPerSecond),
                      translation.getY() * drivetrain.getMaxSpeed().in(MetersPerSecond),
                      omega,
                      true);

              speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getRotation());

              drivetrain.driveFieldRelative(speeds);
            },
            drivetrain)
        .beforeStarting(() -> rotationStrategy.reset(drivetrain));
  }

  public static Command joystickDrive(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return driveCore(
        drivetrain,
        x,
        y,
        new ManualRotation(omega, () -> drivetrain.getMaxAngularSpeed().in(RadiansPerSecond)));
  }

  public static Command joystickDriveThrottleMap(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return driveCore(
        drivetrain,
        x,
        y,
        new ManualRotation(omega, () -> drivetrain.getMaxAngularSpeed().in(RadiansPerSecond)),
        new ThrottleMapModifier());
  }

  public static Command joystickDriveAtAngle(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> angle) {
    return driveCore(drivetrain, x, y, new PIDRotation(angle));
  }

  public static Command joystickDriveHeading(
      Drivetrain drivetrain,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rotX,
      DoubleSupplier rotY) {
    return driveCore(drivetrain, x, y, new HeadingStickRotation(rotX, rotY));
  }

  public static Command joystickDriveAimAtPoint(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, double targetX, double targetY) {
    return driveCore(
        drivetrain,
        x,
        y,
        new PIDRotation(
            () -> {
              Translation2d robot = drivetrain.getPose().getTranslation();
              return new Rotation2d(Math.atan2(targetY - robot.getY(), targetX - robot.getX()));
            }));
  }

  public static Command joystickDriveAimAtBall(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier tx) {
    return driveCore(
        drivetrain,
        x,
        y,
        new PIDRotation(
            () -> {
              Rotation2d currentRotation = drivetrain.getPose().getRotation();
              return new Rotation2d(
                  Degrees.of(currentRotation.getDegrees() - tx.getAsDouble() - 90));
            }));
  }

  public static Command joystickDriveAimHub(
      Drivetrain drivetrain, SuperStructure superStructure, DoubleSupplier x, DoubleSupplier y) {

    return driveCore(
        drivetrain, x, y, new PIDRotation(() -> superStructure.getPredictiveAimAngle()));
  }

  public static Command joystickDriveTrench(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y) {
    return driveCore(
        drivetrain,
        () -> x.getAsDouble(),
        () -> y.getAsDouble(),
        new PIDRotation(
            () -> {
              double currentDegrees = drivetrain.getPose().getRotation().getDegrees();

              double[] targets = {45.0, -45.0, 135.0, -135.0};

              double closestAngle = targets[0];
              double minDistance =
                  Math.abs(MathUtil.inputModulus(targets[0] - currentDegrees, -180, 180));

              // 3. Loop para encontrar o ângulo com a menor distância circular
              for (double target : targets) {
                double distance =
                    Math.abs(MathUtil.inputModulus(target - currentDegrees, -180, 180));
                if (distance < minDistance) {
                  minDistance = distance;
                  closestAngle = target;
                }
              }

              return Rotation2d.fromDegrees(closestAngle);
            }));
  }

  public static Command joystickDriveTowardsPoint(
      Drivetrain drivetrain,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      double targetX,
      double targetY) {
    return driveCore(
        drivetrain,
        x,
        y,
        new ManualRotation(omega, () -> drivetrain.getMaxAngularSpeed().in(RadiansPerSecond)),
        new TargetAssistModifier(() -> new Translation2d(targetX, targetY), 0.3) // 30% assist
        );
  }

  public static Command joystickDriveWithZoneRepulsion(
      Drivetrain drivetrain,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      Polygon2d zone,
      double strength) {
    return driveCore(
        drivetrain,
        x,
        y,
        new ManualRotation(omega, () -> drivetrain.getMaxAngularSpeed().in(RadiansPerSecond)),
        new ZoneRepulsionModifier(zone, strength));
  }

  public static Command joystickDriveWithZoneSuction(
      Drivetrain drivetrain,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      Polygon2d zone,
      double strength) {
    return driveCore(
        drivetrain,
        x,
        y,
        new ManualRotation(omega, () -> drivetrain.getMaxAngularSpeed().in(RadiansPerSecond)),
        new ZoneSuctionModifier(zone, strength));
  }

  public static Command joystickDriveWithZoneBumpSuction(
      Drivetrain drivetrain,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      Polygon2d zone1,
      Polygon2d zone2,
      double strength) {
    return driveCore(
        drivetrain,
        x,
        y,
        new ManualRotation(omega, () -> drivetrain.getMaxAngularSpeed().in(RadiansPerSecond)),
        new DualZoneSuctionBumpModifier(zone1, zone2, strength));
  }

  public static Command forwardStraight(Drivetrain drivetrain, double velocityX) {
    return drivetrain.applyRequest(() -> forwardStraight.withVelocityX(velocityX).withVelocityY(0));
  }

  public static Command brakeSwerveModules(Drivetrain drivetrain) {
    return drivetrain.applyRequest(() -> brake);
  }

  public static Command pointModules(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y) {
    return drivetrain.applyRequest(
        () -> point.withModuleDirection(new Rotation2d(x.getAsDouble(), y.getAsDouble())));
  }

  public static Command drivetrainResetRotation(Drivetrain drivetrain) {
    return drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
  }

  public static Command sysIdDynamicForward(Drivetrain drivetrain) {
    return drivetrain.sysIdDynamic(Direction.kForward);
  }

  public static Command sysIdDynamicReverse(Drivetrain drivetrain) {
    return drivetrain.sysIdDynamic(Direction.kReverse);
  }

  public static Command sysIdQuasistaticForward(Drivetrain drivetrain) {
    return drivetrain.sysIdQuasistatic(Direction.kForward);
  }

  public static Command sysIdQuasistaticReverse(Drivetrain drivetrain) {
    return drivetrain.sysIdQuasistatic(Direction.kReverse);
  }

  public static Command driveToPose(Drivetrain drivetrain, Supplier<Pose2d> targetPose) {
    PathConstraints constraints =
        new PathConstraints(
            drivetrain.getMaxSpeed().in(MetersPerSecond),
            drivetrain.getMaxAcceleration().in(MetersPerSecondPerSecond),
            drivetrain.getMaxAngularSpeed().in(RadiansPerSecond),
            drivetrain.getMaxAngularAcceleration().in(RadiansPerSecondPerSecond));

    return AutoBuilder.pathfindToPose(targetPose.get(), constraints, MetersPerSecond.of(0))
        .withName("Commands/Drivetrain/DriveToPose");
  }
}
