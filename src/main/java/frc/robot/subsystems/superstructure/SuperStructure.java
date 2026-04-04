package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.AllianceManager;
import frc.game.FieldConstants.Poses;
import frc.lib.calculus.LinearInterpolation.Point;
import frc.lib.calculus.LoggedTunableMap;
import frc.lib.calculus.ShotOnTheMoveCalculator;
import frc.lib.calculus.ShotParameters;
import frc.lib.controller.NaturalXboxController;
import frc.lib.interfaces.fsm.StateMachine;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.robot.Constants.RobotRequest;
import frc.robot.Constants.RobotState;
import frc.robot.commands.LedCommands;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorRequest;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRequest;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {

  private final Conveyor conveyor;
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Led led;
  private final Shooter shooter;
  private final PowerDistribution pd = new PowerDistribution();

  private RobotRequest robotRequest = RobotRequest.IDLE;
  private ShooterRequest shooterRequest = ShooterRequest.STOP;
  private IntakeRequest intakeRequest = IntakeRequest.IN;
  private ConveyorRequest conveyorRequest = ConveyorRequest.STOP;

  private final ShotOnTheMoveCalculator hubCalculator;
  private final ShotOnTheMoveCalculator feedCalculator;

  private final LoggedTunableNumber aimScalar = new LoggedTunableNumber("SOTM/AimScalar", -0.8);
  private final LoggedTunableNumber rpmScalar = new LoggedTunableNumber("SOTM/RPMScalar", 0.13);
  private final LoggedTunableNumber shooterEfficiency =
      new LoggedTunableNumber("SOTM/ShooterEfficiency", 1.2);

  private final LoggedTunableMap flywheelMap =
      new LoggedTunableMap(
          "FlywheelCalibrate/Flywheel",
          true,
          new Point(SuperStructureConstants.X_1, SuperStructureConstants.Y_1),
          new Point(SuperStructureConstants.X_2, SuperStructureConstants.Y_2),
          new Point(SuperStructureConstants.X_3, SuperStructureConstants.Y_3),
          new Point(SuperStructureConstants.X_4, SuperStructureConstants.Y_4),
          new Point(SuperStructureConstants.X_5, SuperStructureConstants.Y_5),
          new Point(SuperStructureConstants.X_6, SuperStructureConstants.Y_6),
          new Point(SuperStructureConstants.X_7, SuperStructureConstants.Y_7),
          new Point(SuperStructureConstants.X_8, SuperStructureConstants.Y_8),
          new Point(SuperStructureConstants.X_9, SuperStructureConstants.Y_9));

  private final AllianceManager allianceManager = AllianceManager.getInstance();
  private final StateMachine<RobotState> generalFsm;

  public SuperStructure(
      Conveyor conveyor,
      Drivetrain drivetrain,
      Intake intake,
      Led led,
      Shooter shooter,
      Vision vision,
      NaturalXboxController driverControl,
      NaturalXboxController operatorControl) {

    this.conveyor = conveyor;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.shooter = shooter;
    this.led = led;

    ShotOnTheMoveCalculator.Config shotConfig = buildShotConfig(1.0);
    ShotOnTheMoveCalculator.Config feedConfig = buildShotConfig(0.05);

    this.hubCalculator =
        new ShotOnTheMoveCalculator(
            "SOTM/Hub",
            this::resolveHubTarget,
            flywheelMap,
            aimScalar,
            rpmScalar,
            shooterEfficiency,
            shotConfig);

    this.feedCalculator =
        new ShotOnTheMoveCalculator(
            "SOTM/Feed",
            this::resolveFeedTarget,
            flywheelMap,
            aimScalar,
            rpmScalar,
            shooterEfficiency,
            feedConfig);

    generalFsm =
        new StateMachine<>(
            "Subsystems/SuperStructure/RobotState", RobotState.class, RobotState.IDLE);

    configureGeneralFSM();

    setName("Subsystems/SuperStructure");
    ConstantsLogger.logConstants(SuperStructureConstants.class, getName());
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());

    allianceManager.showAllianceMessageOnDashboard();

    flywheelMap.calculate();

    Pose2d pose = drivetrain.getPose();
    ChassisSpeeds speeds = drivetrain.getRobotVelocity();
    hubCalculator.calculate(pose, speeds);
    feedCalculator.calculate(pose, speeds);

    generalFsm.update();

    log();

    PeriodicTimer.stop(getName());
  }

  public void setRequest(RobotRequest request) {
    this.robotRequest = request;
  }

  public ShooterRequest getShooterRequest() {
    return shooterRequest;
  }

  public IntakeRequest getIntakeRequest() {
    return intakeRequest;
  }

  public ConveyorRequest getConveyorRequest() {
    return conveyorRequest;
  }

  public ShotParameters getActiveShotParameters() {
    return isInAllianceZone() ? hubCalculator.getLastResult() : feedCalculator.getLastResult();
  }

  public boolean isAtSetpointAngle() {
    Rotation2d current = drivetrain.getPose().getRotation();
    Rotation2d target = getActiveShotParameters().aimAngle();
    double errorDeg = Math.abs(current.minus(target).getDegrees());
    double tolerance =
        isInAllianceZone()
            ? SuperStructureConstants.MAX_ERROR_ANGLE_DEG_SHOOT
            : SuperStructureConstants.MAX_ERROR_ANGLE_DEG_NEUTRAL;
    return errorDeg <= tolerance;
  }

  private void configureGeneralFSM() {
    generalFsm
        .state(RobotState.IDLE)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.STOP;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
              schedule(LedCommands.breathe(led, Color.kViolet));
            });

    generalFsm
        .state(RobotState.IDLEING)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.STOP;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
              schedule(LedCommands.chase(led, Color.kCyan));
            })
        .transitionTo(
            RobotState.IDLE,
            () ->
                intake.atGoal() && shooter.atGoal() && conveyor.atGoal() && !drivetrain.IsMoving());

    generalFsm
        .state(RobotState.COLLECTING)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.COLLECT;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
              schedule(LedCommands.chase(led, Color.kCyan));
            });

    generalFsm
        .state(RobotState.GOING_COLLECT)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.COLLECT;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
              schedule(LedCommands.chase(led, Color.kCyan));
            });

    generalFsm
        .state(RobotState.GOING_SHOOT)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.SHOOT;
              intakeRequest = IntakeRequest.OUT;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
              schedule(LedCommands.chase(led, Color.kCyan));
            })
        .transitionTo(RobotState.SHOOTING, () -> shooter.readyToShoot() && isAtSetpointAngle());

    generalFsm
        .state(RobotState.SHOOTING)
        .onEnter(
            () -> {
              conveyorRequest = ConveyorRequest.RUN;
              schedule(LedCommands.rainbowContinuous(led, 8));
            })
        .transitionTo(RobotState.SHOOTING_RECOVERY, () -> !shooter.readyToShoot())
        .transitionTo(RobotState.GOING_SHOOT, () -> !isAtSetpointAngle());

    generalFsm
        .state(RobotState.SHOOTING_RECOVERY)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.SHOOT;
              intakeRequest = IntakeRequest.OUT;
              conveyorRequest = ConveyorRequest.RUN_SLOW;
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
              schedule(LedCommands.breathe(led, Color.kYellow));
            })
        .transitionTo(RobotState.SHOOTING, () -> shooter.readyToShoot());

    generalFsm
        .state(RobotState.GOING_COLLECT_SHOOT)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.SHOOT;
              intakeRequest = IntakeRequest.COLLECT;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
              schedule(LedCommands.chase(led, Color.kCyan));
            })
        .transitionTo(
            RobotState.COLLECT_SHOOTING, () -> shooter.readyToShoot() && isAtSetpointAngle());

    generalFsm
        .state(RobotState.COLLECT_SHOOTING)
        .onEnter(
            () -> {
              conveyorRequest = ConveyorRequest.RUN;
              schedule(LedCommands.rainbowContinuous(led, 8));
            })
        .transitionTo(RobotState.COLLECT_SHOOTING_RECOVERY, () -> !shooter.readyToShoot())
        .transitionTo(RobotState.GOING_SHOOT, () -> !isAtSetpointAngle());

    generalFsm
        .state(RobotState.COLLECT_SHOOTING_RECOVERY)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.SHOOT;
              intakeRequest = IntakeRequest.COLLECT;
              conveyorRequest = ConveyorRequest.RUN_SLOW;
              schedule(LedCommands.breathe(led, Color.kYellow));
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(RobotState.COLLECT_SHOOTING, () -> shooter.readyToShoot());

    generalFsm
        .state(RobotState.CLOSING)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.IN;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
              schedule(LedCommands.chase(led, Color.kCyan));
            })
        .transitionTo(
            RobotState.CLOSED, () -> intake.atGoal() && conveyor.atGoal() && shooter.atGoal());

    generalFsm
        .state(RobotState.CLOSED)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.IN;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
              schedule(LedCommands.breathe(led, Color.kViolet));
            });

    generalFsm.addGlobalTransition(
        RobotState.GOING_SHOOT,
        () ->
            robotRequest == RobotRequest.SHOOT
                && generalFsm.getCurrentState() != RobotState.SHOOTING
                && generalFsm.getCurrentState() != RobotState.SHOOTING_RECOVERY);

    generalFsm.addGlobalTransition(
        RobotState.GOING_COLLECT,
        () ->
            robotRequest == RobotRequest.COLLECT
                && generalFsm.getCurrentState() != RobotState.COLLECTING);

    generalFsm.addGlobalTransition(
        RobotState.GOING_COLLECT_SHOOT,
        () ->
            robotRequest == RobotRequest.COLLECT_SHOOT
                && generalFsm.getCurrentState() != RobotState.COLLECT_SHOOTING
                && generalFsm.getCurrentState() != RobotState.COLLECT_SHOOTING_RECOVERY);

    generalFsm.addGlobalTransition(
        RobotState.CLOSING,
        () ->
            robotRequest == RobotRequest.CLOSE
                && generalFsm.getCurrentState() != RobotState.CLOSED);

    generalFsm.addGlobalTransition(
        RobotState.IDLEING,
        () -> robotRequest == RobotRequest.IDLE && generalFsm.getCurrentState() != RobotState.IDLE);
  }

  private void schedule(Command command) {
    CommandScheduler.getInstance().schedule(command);
  }

  private Translation2d resolveHubTarget(Pose2d robotPose) {
    return allianceManager.isBlue()
        ? Poses.HUB_CENTER_BLUE.getTranslation()
        : Poses.HUB_CENTER_RED.getTranslation();
  }

  private Translation2d resolveFeedTarget(Pose2d robotPose) {
    Alliance alliance = allianceManager.myAlliance();

    Pose2d left =
        alliance == Alliance.Blue ? Poses.SHOOT_INTAKING_LEFT_BLUE : Poses.SHOOT_INTAKING_LEFT_RED;
    Pose2d right =
        alliance == Alliance.Blue
            ? Poses.SHOOT_INTAKING_RIGHT_BLUE
            : Poses.SHOOT_INTAKING_RIGHT_RED;

    double dLeft = robotPose.getTranslation().getDistance(left.getTranslation());
    double dRight = robotPose.getTranslation().getDistance(right.getTranslation());

    return (dLeft < dRight ? left : right).getTranslation();
  }

  private boolean isInAllianceZone() {
    return allianceManager.isInAllianceZone(drivetrain.getCurrentGeneralZone());
  }

  private static ShotOnTheMoveCalculator.Config buildShotConfig(double rpmSmootherAlpha) {
    double avgDiameter =
        (SuperStructureConstants.DIAMETER_WHEEL_UP_METERS
                + SuperStructureConstants.DIAMETER_WHEEL_DOWN_METERS)
            / 2.0;

    return new ShotOnTheMoveCalculator.Config(
        avgDiameter,
        SuperStructureConstants.BALL_EXITING_ANGLE_DEG,
        SuperStructureConstants.SHOOTER_OFFSET_METERS,
        rpmSmootherAlpha);
  }

  private double distanceFromRobotToHub() {
    return drivetrain
        .getPose()
        .getTranslation()
        .getDistance(
            allianceManager.isBlue()
                ? Poses.HUB_CENTER_BLUE.getTranslation()
                : Poses.HUB_CENTER_RED.getTranslation());
  }

  private void log() {
    if (getCurrentCommand() != null) {
      Logger.recordOutput("SuperStructure/Command", getCurrentCommand().getName());
    }

    ShotParameters active = getActiveShotParameters();
    Logger.recordOutput("SuperStructure/GeneralFSM/State", generalFsm.getCurrentState());
    Logger.recordOutput("SuperStructure/GeneralFSM/TimeInState", generalFsm.getTimeInState());
    Logger.recordOutput("SuperStructure/Shooting/DistanceToHub", distanceFromRobotToHub());
    Logger.recordOutput("SuperStructure/Shooting/ActiveAimAngle", active.aimAngle().getDegrees());
    Logger.recordOutput("SuperStructure/Shooting/ActiveRPM", RPM.of(active.rpm()));
    Logger.recordOutput("SuperStructure/Flags/DrivetrainAligned", isAtSetpointAngle());
    Logger.recordOutput("SuperStructure/Flags/InAllianceZone", isInAllianceZone());

    Logger.recordOutput("Subsystems/PDH/totalCurrent", pd.getTotalCurrent());
    Logger.recordOutput("Subsystems/PDH/voltage", pd.getVoltage());
    Logger.recordOutput("Subsystems/PDH/totalEnergy", pd.getTotalEnergy());
    Logger.recordOutput("Subsystems/PDH/totalPower", pd.getTotalPower());
  }
}
