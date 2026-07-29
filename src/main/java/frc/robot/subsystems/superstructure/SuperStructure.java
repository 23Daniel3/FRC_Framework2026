package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.power.RobotPowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.AllianceManager;
import frc.game.FieldConstants.Poses;
import frc.lib.calculus.LinearInterpolation.Point;
import frc.lib.calculus.LoggedTunableMap;
import frc.lib.calculus.ShotOnTheMoveCalculator;
import frc.lib.calculus.ShotParameters;
import frc.lib.interfaces.fsm.StateMachine;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.robot.Constants.RobotRequest;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorRequest;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRequest;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {

  private final Conveyor conveyor;
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Shooter shooter;

  private RobotRequest robotRequest = RobotRequest.IDLE;

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

  public SuperStructure(Conveyor conveyor, Drivetrain drivetrain, Intake intake, Shooter shooter) {

    this.conveyor = conveyor;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.shooter = shooter;

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

    // Alimenta o shooter continuamente com o RPM calculado pelo SOTM. Sem isto o
    // flywheel roda eternamente no setpoint inicial (0 RPM) — o ciclo completo e:
    // calculadores → setVelocity aqui → estados do Shooter aplicam no onUpdate.
    shooter.setVelocity(RPM.of(getActiveShotParameters().rpm()));

    generalFsm.update();

    log();

    PeriodicTimer.stop(getName());
  }

  public void setRequest(RobotRequest request) {
    this.robotRequest = request;
  }

  /** Estado atual da FSM geral (consumido pelo observador de LEDs e por telemetria). */
  public RobotState getRobotState() {
    return generalFsm.getCurrentState();
  }

  public ConveyorRequest getConveyorRequest() {
    return conveyor.getRequest();
  }

  public IntakeRequest getIntakeRequest() {
    return intake.getRequest();
  }

  public ShooterRequest getShooterRequest() {
    return shooter.getRequest();
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
              shooter.setRequest(ShooterRequest.STOP);
              intake.setRequest(IntakeRequest.STOP);
              conveyor.setRequest(ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    generalFsm
        .state(RobotState.IDLING)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.STOP);
              intake.setRequest(IntakeRequest.STOP);
              conveyor.setRequest(ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            })
        .transitionTo(
            RobotState.IDLE,
            () ->
                intake.atGoal() && shooter.atGoal() && conveyor.atGoal() && !drivetrain.isMoving());

    generalFsm
        .state(RobotState.COLLECTING)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.STOP);
              intake.setRequest(IntakeRequest.COLLECT);
              conveyor.setRequest(ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    generalFsm
        .state(RobotState.GOING_COLLECT)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.STOP);
              intake.setRequest(IntakeRequest.COLLECT);
              conveyor.setRequest(ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            })
        .transitionTo(RobotState.COLLECTING, () -> intake.atGoal());

    generalFsm
        .state(RobotState.GOING_SHOOT)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.SHOOT);
              intake.setRequest(IntakeRequest.OUT);
              conveyor.setRequest(ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(RobotState.SHOOTING, () -> shooter.readyToShoot() && isAtSetpointAngle());

    generalFsm
        .state(RobotState.SHOOTING)
        .onEnter(
            () -> {
              conveyor.setRequest(ConveyorRequest.RUN);
            })
        .transitionTo(RobotState.SHOOTING_RECOVERY, () -> !shooter.readyToShoot())
        .transitionTo(RobotState.GOING_SHOOT, () -> !isAtSetpointAngle());

    generalFsm
        .state(RobotState.SHOOTING_RECOVERY)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.SHOOT);
              intake.setRequest(IntakeRequest.OUT);
              conveyor.setRequest(ConveyorRequest.RUN_SLOW);
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(RobotState.SHOOTING, () -> shooter.readyToShoot());

    generalFsm
        .state(RobotState.GOING_COLLECT_SHOOT)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.SHOOT);
              intake.setRequest(IntakeRequest.COLLECT);
              conveyor.setRequest(ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(
            RobotState.COLLECT_SHOOTING, () -> shooter.readyToShoot() && isAtSetpointAngle());

    generalFsm
        .state(RobotState.COLLECT_SHOOTING)
        .onEnter(
            () -> {
              conveyor.setRequest(ConveyorRequest.RUN);
            })
        .transitionTo(RobotState.COLLECT_SHOOTING_RECOVERY, () -> !shooter.readyToShoot())
        // Perder o alinhamento em modo COLLECT_SHOOT volta para GOING_COLLECT_SHOOT
        // (nao GOING_SHOOT), para o intake continuar coletando durante o realinhamento.
        .transitionTo(RobotState.GOING_COLLECT_SHOOT, () -> !isAtSetpointAngle());

    generalFsm
        .state(RobotState.COLLECT_SHOOTING_RECOVERY)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.SHOOT);
              intake.setRequest(IntakeRequest.COLLECT);
              conveyor.setRequest(ConveyorRequest.RUN_SLOW);
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(RobotState.COLLECT_SHOOTING, () -> shooter.readyToShoot());

    generalFsm
        .state(RobotState.CLOSING)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.STOP);
              intake.setRequest(IntakeRequest.IN);
              conveyor.setRequest(ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            })
        .transitionTo(
            RobotState.CLOSED, () -> intake.atGoal() && conveyor.atGoal() && shooter.atGoal());

    generalFsm
        .state(RobotState.CLOSED)
        .onEnter(
            () -> {
              shooter.setRequest(ShooterRequest.STOP);
              intake.setRequest(IntakeRequest.IN);
              conveyor.setRequest(ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    // Mapeamento request → (estado de entrada, estados que ja satisfazem o request).
    // O estado de entrada e excluido automaticamente pelo framework.
    generalFsm.addRequestTransition(
        () -> robotRequest == RobotRequest.SHOOT,
        RobotState.GOING_SHOOT,
        RobotState.SHOOTING,
        RobotState.SHOOTING_RECOVERY);

    generalFsm.addRequestTransition(
        () -> robotRequest == RobotRequest.COLLECT,
        RobotState.GOING_COLLECT,
        RobotState.COLLECTING);

    generalFsm.addRequestTransition(
        () -> robotRequest == RobotRequest.COLLECT_SHOOT,
        RobotState.GOING_COLLECT_SHOOT,
        RobotState.COLLECT_SHOOTING,
        RobotState.COLLECT_SHOOTING_RECOVERY);

    generalFsm.addRequestTransition(
        () -> robotRequest == RobotRequest.CLOSE, RobotState.CLOSING, RobotState.CLOSED);

    generalFsm.addRequestTransition(
        () -> robotRequest == RobotRequest.IDLE, RobotState.IDLING, RobotState.IDLE);

    generalFsm.validateComplete();
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
    Logger.recordOutput("SuperStructure/GeneralFSM/Request", robotRequest);
    Logger.recordOutput("SuperStructure/GeneralFSM/TimeInState", generalFsm.getTimeInState());
    Logger.recordOutput("SuperStructure/Shooting/DistanceToHub", distanceFromRobotToHub());
    Logger.recordOutput("SuperStructure/Shooting/ActiveAimAngle", active.aimAngle().getDegrees());
    Logger.recordOutput("SuperStructure/Shooting/ActiveRPM", RPM.of(active.rpm()));
    Logger.recordOutput("SuperStructure/Flags/DrivetrainAligned", isAtSetpointAngle());
    Logger.recordOutput("SuperStructure/Flags/InAllianceZone", isInAllianceZone());

    RobotPowerDistribution.getInstance().log();
  }
}
