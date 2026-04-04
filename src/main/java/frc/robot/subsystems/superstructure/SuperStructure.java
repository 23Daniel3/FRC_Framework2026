package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.AllianceManager;
import frc.game.FieldConstants.Poses;
import frc.lib.calculus.ExponentialMovingAverage;
import frc.lib.calculus.LinearInterpolation.Point;
import frc.lib.calculus.LoggedTunableMap;
import frc.lib.controller.NaturalXboxController;
import frc.lib.interfaces.fsm.StateMachine;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.AllianceSelector;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.robot.Constants.RobotRequest;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorRequest;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Zones;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRequest;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {

  private final Conveyor conveyor;
  // private final Display display;
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Led led;
  private final Shooter shooter;
  private final PowerDistribution pd = new PowerDistribution();

  private ShooterRequest shooterRequest = ShooterRequest.STOP;
  private IntakeRequest intakeRequest = IntakeRequest.IN;
  private ConveyorRequest conveyorRequest = ConveyorRequest.STOP;

  private boolean hasElement = false;
  private boolean isConveyorReversing = false;

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

  private final LoggedTunableNumber aimPredictionScalar =
      new LoggedTunableNumber("SOTM/AimScalar", -0.8);
  private final LoggedTunableNumber rpmPredictionScalar =
      new LoggedTunableNumber("SOTM/RPMScalar", 0.13);

  private final LoggedTunableNumber shooterEfficiency =
      new LoggedTunableNumber("SOTM/ShooterEfficiency", 1.2);

  private Rotation2d predictiveAimAngle = new Rotation2d();
  private double predictiveRPM = 0.0;
  private ExponentialMovingAverage smooth = new ExponentialMovingAverage(0.05);

  private AllianceManager allianceManager = AllianceManager.getInstance();

  private RobotRequest robotRequest = RobotRequest.IDLE;

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

    generalFsm =
        new StateMachine<RobotState>(
            "Subsystems/SuperStructure/RobotState", RobotState.class, RobotState.IDLE);

    configureGeneralFSM();

    setName("Subsystems/SuperStructure");
    ConstantsLogger.logConstants(SuperStructureConstants.class, getName());
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
            });

    generalFsm
        .state(RobotState.IDLEING)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.STOP;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            })
        .transitionTo(RobotState.IDLE, () -> intake.atGoal() && shooter.atGoal() && conveyor.atGoal() && !drivetrain.IsMoving());

    generalFsm
        .state(RobotState.COLLECTING)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.COLLECT;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    generalFsm
        .state(RobotState.GOING_COLLECT)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.COLLECT;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    generalFsm
        .state(RobotState.SHOOTING)
        .onEnter(() -> {
          conveyorRequest = ConveyorRequest.RUN;
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
          })
      .transitionTo(RobotState.SHOOTING, () -> shooter.readyToShoot());

    generalFsm
        .state(RobotState.GOING_SHOOT)
        .onEnter(
            () -> {
              shooterRequest = ShooterRequest.SHOOT;
              intakeRequest = IntakeRequest.OUT;
              conveyorRequest = ConveyorRequest.STOP;
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(RobotState.SHOOTING, () -> shooter.readyToShoot() && isAtSetpointAngle());

    generalFsm
      .state(RobotState.COLLECT_SHOOTING)
      .onEnter(
        () -> {
          conveyorRequest = ConveyorRequest.RUN;
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
            drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
          })
      .transitionTo(RobotState.COLLECT_SHOOTING, () -> shooter.readyToShoot());

    generalFsm
      .state(RobotState.GOING_COLLECT_SHOOT)
      .onEnter(
          () -> {
              shooterRequest = ShooterRequest.SHOOT;
              intakeRequest = IntakeRequest.COLLECT;
              conveyorRequest = ConveyorRequest.STOP; 
            drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
          })
        .transitionTo(RobotState.COLLECT_SHOOTING, () -> shooter.readyToShoot() && isAtSetpointAngle());


    generalFsm
      .state(RobotState.CLOSED)
      .onEnter(
          () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.IN;
              conveyorRequest = ConveyorRequest.STOP;
            drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
          });

    generalFsm
      .state(RobotState.CLOSING)
      .onEnter(
          () -> {
              shooterRequest = ShooterRequest.STOP;
              intakeRequest = IntakeRequest.IN;
              conveyorRequest = ConveyorRequest.STOP;
            drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
          })
      .transitionTo(RobotState.CLOSED, () -> intake.atGoal() && conveyor.atGoal() && shooter.atGoal());

    generalFsm.addGlobalTransition(RobotState.GOING_SHOOT, () -> robotRequest == RobotRequest.SHOOT && generalFsm.getCurrentState() != RobotState.SHOOTING && generalFsm.getCurrentState() != RobotState.SHOOTING_RECOVERY);
    generalFsm.addGlobalTransition(RobotState.GOING_COLLECT, () -> robotRequest == RobotRequest.COLLECT && generalFsm.getCurrentState() != RobotState.COLLECTING);
    generalFsm.addGlobalTransition(RobotState.GOING_COLLECT_SHOOT, () -> robotRequest == RobotRequest.COLLECT_SHOOT && generalFsm.getCurrentState() != RobotState.COLLECT_SHOOTING && generalFsm.getCurrentState() != RobotState.COLLECT_SHOOTING_RECOVERY);
    generalFsm.addGlobalTransition(RobotState.CLOSING, () -> robotRequest == RobotRequest.CLOSE && generalFsm.getCurrentState() != RobotState.CLOSED);
    generalFsm.addGlobalTransition(RobotState.IDLEING, () -> robotRequest == RobotRequest.IDLE && generalFsm.getCurrentState() != RobotState.IDLE);
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());

    allianceManager.showAllianceMessageOnDashboard();

    flywheelMap.calculate();

    generalFsm.update();

    updateShooterPredictions();

    log();

    PeriodicTimer.stop(getName());
  }

  public void setRequest(RobotRequest request) {
    this.robotRequest = request;
  }

  private void updateShooterPredictions() {
    Pose2d robotPose = drivetrain.getPose();
    ChassisSpeeds robotSpeeds = drivetrain.getRobotVelocity();

    Translation2d target =
        allianceManager.isBlue()
            ? Poses.HUB_CENTER_BLUE.getTranslation()
            : Poses.HUB_CENTER_RED.getTranslation();

    Translation2d robotToTarget = target.minus(robotPose.getTranslation());
    double staticDistance = robotToTarget.getNorm();

    if (staticDistance < 0.1) staticDistance = 0.1;

    double baseRPM = flywheelMap.applyThrottle(staticDistance);
    double avgDiameter =
        (SuperStructureConstants.DIAMETER_WHEEL_UP_METERS
                + SuperStructureConstants.DIAMETER_WHEEL_DOWN_METERS)
            / 2.0;

    double vExitTotal = (baseRPM * Math.PI * avgDiameter / 60.0) * shooterEfficiency.get();
    double vExitHorizontal =
        vExitTotal * Math.cos(Math.toRadians(SuperStructureConstants.BALL_EXITING_ANGLE_DEG));

    if (vExitHorizontal < 1.0) vExitHorizontal = 10.0; // Fallback para evitar divisão por zero
    double timeOfFlight = staticDistance / vExitHorizontal;

    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d unitToTarget = robotToTarget.div(staticDistance);

    double radialVelocity =
        robotVelocity.getX() * unitToTarget.getX() + robotVelocity.getY() * unitToTarget.getY();

    double tangentialVelocity =
        robotVelocity.getY() * unitToTarget.getX() - robotVelocity.getX() * unitToTarget.getY();

    double lateralDisplacement = tangentialVelocity * timeOfFlight * aimPredictionScalar.get();

    Rotation2d angleToTarget = robotToTarget.getAngle();
    double compensationAngle = Math.atan2(lateralDisplacement, staticDistance);

    double offsetRatio = SuperStructureConstants.SHOOTER_OFFSET_METERS / staticDistance;
    offsetRatio = Math.max(-0.99, Math.min(0.99, offsetRatio));
    double offsetCorrection = Math.asin(offsetRatio);

    this.predictiveAimAngle =
        angleToTarget.plus(new Rotation2d(compensationAngle - offsetCorrection));

    double radialRpmEquivalent =
        (radialVelocity * 60.0) / (Math.PI * avgDiameter * shooterEfficiency.get());

    double neededRPM = baseRPM - (radialRpmEquivalent * rpmPredictionScalar.get());

    this.predictiveRPM = Math.max(0, neededRPM);

    Logger.recordOutput("SuperStructure/Prediction/RadialVelocity", radialVelocity);
    Logger.recordOutput("SuperStructure/Prediction/TangentialVelocity", tangentialVelocity);
    Logger.recordOutput("SuperStructure/Prediction/TimeOfFlight", timeOfFlight);
    Logger.recordOutput(
        "SuperStructure/Prediction/CalculatedAngle", predictiveAimAngle.getDegrees());
  }

  public boolean isAtSetpointAngle() {
    Rotation2d currentRotation = drivetrain.getPose().getRotation();
    Rotation2d setpointRotation = calculateDynamicTargetAngle();

    double errorDegrees = Math.abs(currentRotation.minus(setpointRotation).getDegrees());

    if (allianceManager.isInAllianceZone(drivetrain.getCurrentGeneralZone())) {
      return errorDegrees <= SuperStructureConstants.MAX_ERROR_ANGLE_DEG_SHOOT;
    } else {
      return errorDegrees <= SuperStructureConstants.MAX_ERROR_ANGLE_DEG_NEUTRAL;
    }
  }

  private double idealFlywheelRPM() {
    if (drivetrain.getCurrentGeneralZone() == allianceManager.myAllianceZone()) {
      return this.predictiveRPM;
    } else {
      Alliance alliance = allianceManager.myAlliance();
      Pose2d leftPose =
          (alliance == Alliance.Blue)
              ? Poses.SHOOT_INTAKING_LEFT_BLUE
              : Poses.SHOOT_INTAKING_LEFT_RED;

      Pose2d rightPose =
          (alliance == Alliance.Blue)
              ? Poses.SHOOT_INTAKING_RIGHT_BLUE
              : Poses.SHOOT_INTAKING_RIGHT_RED;

      Translation2d robotTranslation = drivetrain.getPose().getTranslation();

      double distance =
          Math.min(
              robotTranslation.getDistance(leftPose.getTranslation()),
              robotTranslation.getDistance(rightPose.getTranslation()));

      Logger.recordOutput(
          "SuperStructure/Shooting/DistanceToShootPoseWhenInNeutralZones", distance);

      double throttle = flywheelMap.applyThrottle(distance);
      return smooth.calculate(throttle);
    }
  }

  public Rotation2d getPredictiveAimAngle() {
    return this.predictiveAimAngle;
  }

  public Rotation2d calculateDynamicTargetAngle() {
    Alliance myAlliance =
        DriverStation.getAlliance().orElse(AllianceSelector.getInstance().getAlliance());

    Zones currentZone = drivetrain.getCurrentGeneralZone();
    Pose2d robotPose = drivetrain.getPose();

    boolean inScoringZone =
        (myAlliance == Alliance.Blue && currentZone == Zones.ALLIANCE_BLUE_ZONE)
            || (myAlliance == Alliance.Red && currentZone == Zones.ALLIANCE_RED_ZONE);

    if (inScoringZone) {
      return getPredictiveAimAngle();
    }

    if (currentZone == Zones.NEUTRAL_ZONE) {
      return getNearestIntakeRotation(robotPose, myAlliance);
    }

    return getPredictiveAimAngle();
  }

  public Rotation2d getNearestIntakeRotation(Pose2d robotPose, Alliance alliance) {
    Pose2d leftPose =
        (alliance == Alliance.Blue)
            ? Poses.SHOOT_INTAKING_LEFT_BLUE
            : Poses.SHOOT_INTAKING_LEFT_RED;

    Pose2d rightPose =
        (alliance == Alliance.Blue)
            ? Poses.SHOOT_INTAKING_RIGHT_BLUE
            : Poses.SHOOT_INTAKING_RIGHT_RED;

    double distToLeft = robotPose.getTranslation().getDistance(leftPose.getTranslation());
    double distToRight = robotPose.getTranslation().getDistance(rightPose.getTranslation());

    Pose2d targetPose = (distToLeft < distToRight) ? leftPose : rightPose;

    return new Rotation2d(
        Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX()));
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

  public ShooterRequest getShooterRequest() { return shooterRequest; }
  public IntakeRequest getIntakeRequest() { return intakeRequest; }
  public ConveyorRequest getConveyorRequest() { return conveyorRequest; }

  public void log() {
    if (getCurrentCommand() != null) {
      Logger.recordOutput("SuperStructure/Command", getCurrentCommand().getName());
    }

    Logger.recordOutput("SuperStructure/GeneralFSM/State", generalFsm.getCurrentState());
    Logger.recordOutput("SuperStructure/GeneralFSM/TimeInState", generalFsm.getTimeInState());

    Logger.recordOutput("SuperStructure/Shooting/DistanceToHub", distanceFromRobotToHub());
    Logger.recordOutput(
        "SuperStructure/Shooting/IdealFlywheelRPMSmooth", RPM.of(idealFlywheelRPM()));
    Logger.recordOutput("SuperStructure/Flags/DrivetrainAligned", isAtSetpointAngle());
    Logger.recordOutput("SuperStructure/Flags/hasElement", hasElement);
    Logger.recordOutput("SuperStructure/Flags/isConveyorReversing", isConveyorReversing);

    Logger.recordOutput("Subsystems/PDH/totalCurrent", pd.getTotalCurrent());
    Logger.recordOutput("Subsystems/PDH/voltage", pd.getVoltage());
    Logger.recordOutput("Subsystems/PDH/totalEnergy", pd.getTotalEnergy());
    Logger.recordOutput("Subsystems/PDH/totalPower", pd.getTotalPower());
  }
}
