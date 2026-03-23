package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import frc.lib.util.SetpointTracker;
import frc.lib.util.security.LockedMotorDetector;
import frc.robot.Constants.GeneralIntention;
import frc.robot.commands.LedCommands;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorIntention;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Zones;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelIntention;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeIntention;
import frc.robot.subsystems.intake.IntakeConstants.RollerIntention;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.kicker.KickerConstants.KickerIntention;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {

  private final Conveyor conveyor;
  // private final Display display;
  private final Drivetrain drivetrain;
  private final Flywheel flywheel;
  private final Intake intake;
  private final Kicker kicker;
  private final Led led;
  private final PowerDistribution pd = new PowerDistribution();
  // private final Vision vision;

  // private final NaturalXboxController driverControl;
  // private final NaturalXboxController operatorControl;

  // private final LoggedTunableNumber intakePower =
  //     new LoggedTunableNumber("INTAKE_POWER_PIT_TEST", 0.5);

  private final Timer conveyorReverseTimer = new Timer();
  private final Timer sensorTimer = new Timer();

  private final LockedMotorDetector lockedConveyor =
      new LockedMotorDetector(
          SuperStructureConstants.MAX_TIME_LOCKED, SuperStructureConstants.CONVEYOR_MIN_VELOCITY);
  private boolean shouldBeMovingConveyor = false;

  private final LoggedTunableNumber flywheelTolerance =
      new LoggedTunableNumber(
          "Superstructure/FlywheelTolerance", FlywheelConstants.SHOOT_TOLERANCE);
  private final LoggedTunableNumber kickerTolerance =
      new LoggedTunableNumber("Superstructure/KickerTolerance", KickerConstants.VELOCITY_TOLERANCE);

  private boolean hasElement = false;
  private boolean shouldConsiderate = true;
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

  private ConveyorIntention conveyorIntention = ConveyorIntention.STOP;
  private FlywheelIntention flywheelIntention = FlywheelIntention.STOP;
  private IntakeIntention intakeIntention = IntakeIntention.IN;
  private RollerIntention rollerIntention = RollerIntention.STOP;
  private KickerIntention kickerIntention = KickerIntention.STOP;

  private ConveyorIntention pilotConveyorIntention = ConveyorIntention.NON_INTENTION;
  private FlywheelIntention pilotFlywheelIntention = FlywheelIntention.NON_INTENTION;
  private IntakeIntention pilotIntakeIntention = IntakeIntention.NON_INTENTION;
  private RollerIntention pilotRollerIntention = RollerIntention.NON_INTENTION;
  private KickerIntention pilotKickerIntention = KickerIntention.NON_INTENTION;

  private ConveyorIntention possibleConveyorIntention = ConveyorIntention.STOP;
  private FlywheelIntention possibleFlywheelIntention = FlywheelIntention.STOP;
  private IntakeIntention possibleIntakeIntention = IntakeIntention.IN;
  private RollerIntention possibleRollerIntention = RollerIntention.STOP;
  private KickerIntention possibleKickerIntention = KickerIntention.STOP;

  private GeneralIntention generalIntention = GeneralIntention.IDLE;

  private enum ShootState {
    IDLE,
    WAITING_COLLECT,
    RUN_FLYWHEEL,
    RUN_KICKER,
    SHOOT,
    RECOVERY_SHOOT
  }

  private boolean forceShoot = false;

  private final StateMachine<ShootState> shootFsm;
  private final StateMachine<GeneralIntention> generalFsm;

  public SuperStructure(
      Conveyor conveyor,
      // Display display,
      Drivetrain drivetrain,
      Flywheel flywheel,
      Intake intake,
      Kicker kicker,
      Led led,
      Vision vision,
      NaturalXboxController driverControl,
      NaturalXboxController operatorControl) {

    this.conveyor = conveyor;
    // this.display = display;
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;
    this.intake = intake;
    this.kicker = kicker;
    this.led = led;

    // this.driverControl = driverControl;
    // this.operatorControl = operatorControl;

    shootFsm =
        new StateMachine<ShootState>(
            "Subsystems/SuperStructure/Shoot", ShootState.class, ShootState.IDLE);

    generalFsm =
        new StateMachine<GeneralIntention>(
            "Subsystems/SuperStructure/General", GeneralIntention.class, GeneralIntention.IDLE);

    configureShootFSM();
    configureGeneralFSM();

    // zoneTriggers = new ZoneTriggers(drive);

    setName("Subsystems/SuperStructure");
    ConstantsLogger.logConstants(SuperStructureConstants.class, getName());
    shootFsm.forceState(ShootState.IDLE);
  }

  private void configureShootFSM() {
    shootFsm
        .state(ShootState.IDLE)
        .onEnter(
            () -> {
              if (!allianceManager.isInsideShootingWindow()) {
                flywheelIntention = FlywheelIntention.STOP;
              }
              kickerIntention = KickerIntention.STOP;
              conveyorIntention = ConveyorIntention.STOP;
              runCommand(LedCommands.loading(led, Color.kBlue));
            })
        .onExit(
            () -> {
              shouldConsiderate = true;
            })
        .transitionTo(ShootState.RUN_FLYWHEEL, () -> generalIntention == GeneralIntention.SHOOT);

    shootFsm
        .state(ShootState.WAITING_COLLECT)
        .onEnter(
            () -> {
              flywheelIntention = FlywheelIntention.STOP;
              conveyorIntention = ConveyorIntention.STOP;
              kickerIntention = KickerIntention.STOP;
              shouldConsiderate = true;
              runCommand(LedCommands.breathe(led, Color.kCyan));
            })
        .transitionTo(ShootState.RUN_FLYWHEEL, () -> generalIntention == GeneralIntention.SHOOT);

    shootFsm
        .state(ShootState.RUN_FLYWHEEL)
        .onEnter(
            () -> {
              rollerIntention = RollerIntention.INTAKE;
              flywheelIntention = FlywheelIntention.SHOOT;
              kickerIntention = KickerIntention.STOP;
              conveyorIntention = ConveyorIntention.REVERSE;
              shouldConsiderate = true;
              runCommand(LedCommands.loading(led, Color.kBlue));
              conveyorReverseTimer.restart();
            })
        .transitionTo(ShootState.RUN_KICKER, () -> isFlywheelAlmostAtRPMToShoot());

    shootFsm
        .state(ShootState.RUN_KICKER)
        .onEnter(
            () -> {
              rollerIntention = RollerIntention.INTAKE;
              flywheelIntention = FlywheelIntention.SHOOT;
              kickerIntention = KickerIntention.SHOOT;
              conveyorIntention = ConveyorIntention.STOP;
              shouldConsiderate = true;
              runCommand(LedCommands.loading(led, Color.kBlue));
            })
        .transitionTo(ShootState.SHOOT, () -> readyToShoot());

    shootFsm
        .state(ShootState.SHOOT)
        .onEnter(
            () -> {
              rollerIntention = RollerIntention.INTAKE;
              flywheelIntention = FlywheelIntention.SHOOT;
              kickerIntention = KickerIntention.SHOOT;
              conveyorIntention = ConveyorIntention.RUN;
              shouldConsiderate = true;
              runCommand(LedCommands.rainbowContinuous(led, 8));
              if (sensorTimer.hasElapsed(SuperStructureConstants.SENSOR_DELAY_SECONDS)) {
                shouldConsiderate = false;
              }
            })
        .onExit(
            () -> {
              rollerIntention = RollerIntention.STOP;
            })
        .transitionTo(ShootState.RECOVERY_SHOOT, () -> !readyToShoot());

    shootFsm
        .state(ShootState.RECOVERY_SHOOT)
        .onEnter(
            () -> {
              rollerIntention = RollerIntention.INTAKE;
              flywheelIntention = FlywheelIntention.SHOOT;
              kickerIntention = KickerIntention.SHOOT;
              conveyorIntention = ConveyorIntention.RUN_SLOW;
              shouldConsiderate = true;
              runCommand(LedCommands.breathe(led, Color.kYellow));
              if (sensorTimer.hasElapsed(SuperStructureConstants.SENSOR_DELAY_SECONDS)) {
                shouldConsiderate = false;
              }
            })
        .onExit(
            () -> {
              rollerIntention = RollerIntention.STOP;
            })
        .transitionTo(ShootState.SHOOT, () -> readyToShoot());
  }

  private void configureGeneralFSM() {
    generalFsm
        .state(GeneralIntention.IDLE)
        .onEnter(
            () -> {
              shootFsm.forceState(ShootState.IDLE);
              rollerIntention = RollerIntention.STOP;
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    generalFsm
        .state(GeneralIntention.SHOOT)
        .onEnter(
            () -> {
              if (kicker.hasElement() || isConveyorReversing) {
                sensorTimer.restart();
              }
              shootFsm.forceState(ShootState.RUN_FLYWHEEL);
              intakeIntention = IntakeIntention.OUT;
              rollerIntention = RollerIntention.STOP;
              intakeIntention = IntakeIntention.MIDDLE;
              drivetrain.setMaxSpeed(MetersPerSecond.of(1));
            });

    generalFsm
        .state(GeneralIntention.COLLECT)
        .onEnter(
            () -> {
              intakeIntention = IntakeIntention.OUT;
              rollerIntention = RollerIntention.INTAKE;
              shootFsm.forceState(ShootState.WAITING_COLLECT);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    generalFsm
        .state(GeneralIntention.COLLECT_SHOOTING)
        .onEnter(
            () -> {
              shootFsm.forceState(ShootState.RUN_FLYWHEEL);
              intakeIntention = IntakeIntention.OUT;
              rollerIntention = RollerIntention.INTAKE;
              drivetrain.setMaxSpeed(MetersPerSecond.of(1));
            });

    generalFsm
        .state(GeneralIntention.CLOSED)
        .onEnter(
            () -> {
              intakeIntention = IntakeIntention.IN;
              rollerIntention = RollerIntention.STOP;
              shootFsm.forceState(ShootState.IDLE);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });
  }

  private void runCommand(Command command) {
    CommandScheduler.getInstance().schedule(command);
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());

    lockedConveyor.update(
        shouldBeMovingConveyor, conveyor.getMotorInputs().velocity.in(RadiansPerSecond));

    allianceManager.showAllianceMessageOnDashboard();

    if (allianceManager.isInsideShootingWindow()) {
      flywheelIntention = FlywheelIntention.SHOOT;
    }

    if (lockedConveyor.getAsBoolean()) {
      conveyorReverseTimer.restart();
    }

    if (conveyorReverseTimer.hasElapsed(1.5)) {
      conveyorReverseTimer.stop();
      conveyorReverseTimer.reset();
    }

    hasElement =
        (kicker.hasElement()
            || (sensorTimer.get() < SuperStructureConstants.SENSOR_DELAY_SECONDS
                && sensorTimer.get() > 0)
            || shouldConsiderate);

    flywheelMap.calculate();

    generalFsm.update();
    shootFsm.update();

    updateShooterPredictions();

    var velocity = drivetrain.getRobotVelocity();
    double vx = velocity.vxMetersPerSecond;
    double vy = velocity.vyMetersPerSecond;

    double finalVelocity = Math.hypot(vx, vy);

    Logger.recordOutput("Subsystems/Drivetrain/RobotTotalVelocity", finalVelocity);

    // if (finalVelocity >= SuperStructureConstants.MAX_VELOCITY_TO_CLOSE_INTAKE
    //     && RobotModeTriggers.autonomous().negate().getAsBoolean()) {
    //   intakeIntention = IntakeIntention.IN;
    // }

    log();

    runConveyor();
    runFlywheel();
    runIntake();
    runRoller();
    runKicker();
    PeriodicTimer.stop(getName());
  }

  public boolean hasElement() {
    return hasElement;
  }

  private void runConveyor() {

    possibleConveyorIntention =
        pilotConveyorIntention == ConveyorIntention.NON_INTENTION
            ? conveyorIntention
            : pilotConveyorIntention;

    switch (possibleConveyorIntention) {
      case STOP:
        conveyor.stop();
        shouldBeMovingConveyor = false;
        break;

      case RUN:
        shouldBeMovingConveyor = true;
        if (lockedConveyor.getAsBoolean()
            && !conveyorReverseTimer.hasElapsed(SuperStructureConstants.MAX_TIME_LOCKED)) {
          conveyor.runPercentOutput(ConveyorConstants.REVERSE_POWER);
          isConveyorReversing = true;
        } else {
          conveyorReverseTimer.reset();

          if (RobotModeTriggers.autonomous().getAsBoolean()
              && forceShoot
              && drivetrain.isNeutralMidZone()) {
            conveyor.stop();
          } else {
            conveyor.runPercentOutput(ConveyorConstants.POWER);
          }
          isConveyorReversing = false;
        }
        break;

      case REVERSE:
        shouldBeMovingConveyor = true;
        conveyor.runPercentOutput(ConveyorConstants.REVERSE_POWER);
        break;

      case SLOW_REVERSE:
        shouldBeMovingConveyor = true;
        conveyor.runPercentOutput(ConveyorConstants.SLOW_REVERSE_POWER);
        break;

      case WIGGLE:
        shouldBeMovingConveyor = false;
        wiggleConveyor();
        break;
      case RUN_SLOW:
        shouldBeMovingConveyor = true;
        if (lockedConveyor.getAsBoolean()
            && !conveyorReverseTimer.hasElapsed(SuperStructureConstants.MAX_TIME_LOCKED)) {
          conveyor.runPercentOutput(ConveyorConstants.REVERSE_POWER);
          isConveyorReversing = true;
        } else {
          conveyorReverseTimer.reset();

          if (RobotModeTriggers.autonomous().getAsBoolean()
              && forceShoot
              && drivetrain.isNeutralMidZone()) {
            conveyor.stop();
          } else {
            conveyor.runPercentOutput(ConveyorConstants.SLOW_POWER);
          }
          isConveyorReversing = false;
        }
        break;
      case NON_INTENTION:
        conveyor.stop();
        break;
    }
  }

  private void wiggleConveyor() {
    double currentTime = Timer.getFPGATimestamp();
    double phase = currentTime % SuperStructureConstants.WIGGLE_PERIOD;
    boolean forward = phase < (SuperStructureConstants.WIGGLE_PERIOD / 2.0);

    double power =
        forward ? SuperStructureConstants.WIGGLE_POWER : -SuperStructureConstants.WIGGLE_POWER;
    conveyor.runPercentOutput(power);
  }

  private void runIntake() {

    possibleIntakeIntention =
        pilotIntakeIntention == IntakeIntention.NON_INTENTION
            ? intakeIntention
            : pilotIntakeIntention;

    switch (possibleIntakeIntention) {
      case IN:
        intake.runPosition(Rotations.of(IntakeConstants.INTAKE_IN_POSITION));
        break;

      case OUT:
        intake.runPosition(Rotations.of(IntakeConstants.INTAKE_OUT_POSITION));
        break;

      case MIDDLE:
        intake.runPosition(Rotations.of(IntakeConstants.INTAKE_MIDDLE_POSITION));
        break;

      default:
        intake.stopIntakeMotor();
        break;
    }
  }

  private void runFlywheel() {

    possibleFlywheelIntention =
        pilotFlywheelIntention == FlywheelIntention.NON_INTENTION
            ? flywheelIntention
            : pilotFlywheelIntention;

    switch (possibleFlywheelIntention) {
      case STOP:
        flywheel.stop();
        break;

      case SHOOT:
        flywheelShootByDistance();
        break;

      case REVERSE:
        flywheel.runPercentOutput(FlywheelConstants.REVERSE_POWER);
        break;

      case IDLE_SPIN:
        flywheel.runVelocity(RPM.of(FlywheelConstants.IDLE_SPIN_VELOCITY));
        break;

      default:
        flywheel.stop();
        break;
    }
  }

  private void flywheelShootByDistance() {
    flywheel.runVelocity(RPM.of(idealFlywheelRPM()));
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

  public boolean isFlywheelAtRPMToShoot() {
    return SetpointTracker.atSetpoint(
        idealFlywheelRPM(), flywheelTolerance.get(), flywheel.getLeaderInputs().velocity.in(RPM));
  }

  public boolean isFlywheelAlmostAtRPMToShoot() {
    return SetpointTracker.atSetpoint(
        idealFlywheelRPM(),
        FlywheelConstants.START_KICKER_TOLERANCE,
        flywheel.getLeaderInputs().velocity.in(RPM));
  }

  public boolean isKickerAtRPMToShoot() {
    return SetpointTracker.atSetpoint(
        idealFlywheelRPM(), kickerTolerance.get(), kicker.getMotorInputs().velocity.in(RPM));
  }

  public boolean readyToShoot() {
    return (isFlywheelAtRPMToShoot() && isKickerAtRPMToShoot() && isAtSetpointAngle())
        || (RobotModeTriggers.autonomous().getAsBoolean() && forceShoot);
  }

  private void runKicker() {
    possibleKickerIntention =
        pilotKickerIntention == KickerIntention.NON_INTENTION
            ? kickerIntention
            : pilotKickerIntention;

    switch (possibleKickerIntention) {
      case STOP:
        kicker.stop();
        break;

      case SHOOT:
        kicker.runVelocity(RPM.of(idealFlywheelRPM()));
        break;

      case IDLE_SPIN:
        kicker.runVelocity(RPM.of(FlywheelConstants.IDLE_SPIN_VELOCITY));
        break;

      case REVERSE:
        kicker.runPercentOutput(KickerConstants.REVERSE_POWER);
        break;

      case SLOW_REVERSE:
        kicker.runPercentOutput(KickerConstants.SLOW_REVERSE_POWER);
        break;
      default:
        kicker.stop();
        break;
    }
  }

  private void runRoller() {

    possibleRollerIntention =
        pilotRollerIntention == RollerIntention.NON_INTENTION
            ? rollerIntention
            : pilotRollerIntention;

    switch (possibleRollerIntention) {
      case INTAKE:
        if (intake.getIntakeMotorInputs().position.in(Rotations) >= 8) {
          // intake.runVelocity(RPM.of(IntakeConstants.INTAKE_MAX_VELOCITY));
          intake.runPercentOutputRollerMotor(IntakeConstants.INTAKE_POWER);
          // intake.runPercentOutputRollerMotor(intakePower.get());
        } else {
          intake.stopRollerMotor();
        }
        break;

      case OUTAKE:
        if (intake.getIntakeMotorInputs().position.in(Rotations) >= 8) {
          intake.runPercentOutputRollerMotor(IntakeConstants.INTAKE_REVERSE_POWER);
        } else {
          intake.stopRollerMotor();
        }
        break;

      case STOP:
        intake.stopRollerMotor();
        break;

      default:
        intake.stopRollerMotor();
        break;
    }
  }

  public void setConveyorIntention(ConveyorIntention conveyorIntention) {
    this.pilotConveyorIntention = conveyorIntention;
  }

  public void setIntakeIntention(IntakeIntention intakeIntention) {
    this.pilotIntakeIntention = intakeIntention;
  }

  public void setRollerIntention(RollerIntention rollerIntention) {
    this.pilotRollerIntention = rollerIntention;
  }

  public void setKickerIntention(KickerIntention kickerIntention) {
    this.pilotKickerIntention = kickerIntention;
  }

  public void setFlywheelIntention(FlywheelIntention flywheelIntention) {
    this.pilotFlywheelIntention = flywheelIntention;
  }

  public void setGeneralIntention(GeneralIntention generalIntention) {
    this.generalIntention = generalIntention;
    this.generalFsm.forceState(generalIntention);
  }

  public void setForceShoot(boolean forceShoot) {
    this.forceShoot = forceShoot;
  }

  public void log() {
    if (getCurrentCommand() != null) {
      Logger.recordOutput("SuperStructure/Command", getCurrentCommand().getName());
    }

    Logger.recordOutput("SuperStructure/ShootFSM/State", shootFsm.getCurrentState());
    Logger.recordOutput("SuperStructure/ShootFSM/TimeInState", shootFsm.getTimeInState());

    Logger.recordOutput("SuperStructure/GeneralFSM/State", generalFsm.getCurrentState());
    Logger.recordOutput("SuperStructure/GeneralFSM/TimeInState", generalFsm.getTimeInState());

    Logger.recordOutput("SuperStructure/Intention/General", generalIntention);

    Logger.recordOutput("SuperStructure/Intention/System/Conveyor", conveyorIntention);
    Logger.recordOutput("SuperStructure/Intention/System/Flywheel", flywheelIntention);
    Logger.recordOutput("SuperStructure/Intention/System/Intake", intakeIntention);
    Logger.recordOutput("SuperStructure/Intention/System/Roller", rollerIntention);
    Logger.recordOutput("SuperStructure/Intention/System/Kicker", kickerIntention);

    Logger.recordOutput("SuperStructure/Intention/Possible/Conveyor", possibleConveyorIntention);
    Logger.recordOutput("SuperStructure/Intention/Possible/Flywheel", possibleFlywheelIntention);
    Logger.recordOutput("SuperStructure/Intention/Possible/Intake", possibleIntakeIntention);
    Logger.recordOutput("SuperStructure/Intention/Possible/Roller", possibleRollerIntention);
    Logger.recordOutput("SuperStructure/Intention/Possible/Kicker", possibleKickerIntention);

    Logger.recordOutput("SuperStructure/Intention/Pilot/Conveyor", pilotConveyorIntention);
    Logger.recordOutput("SuperStructure/Intention/Pilot/Flywheel", pilotFlywheelIntention);
    Logger.recordOutput("SuperStructure/Intention/Pilot/Intake", pilotIntakeIntention);
    Logger.recordOutput("SuperStructure/Intention/Pilot/Roller", pilotRollerIntention);
    Logger.recordOutput("SuperStructure/Intention/Pilot/Kicker", pilotKickerIntention);

    Logger.recordOutput("SuperStructure/Shooting/DistanceToHub", distanceFromRobotToHub());
    Logger.recordOutput(
        "SuperStructure/Shooting/IdealFlywheelRPMSmooth", RPM.of(idealFlywheelRPM()));
    Logger.recordOutput("SuperStructure/Shooting/ReadyToShoot", readyToShoot());

    Logger.recordOutput("SuperStructure/Flags/FlywheelAtSetpoint", isFlywheelAtRPMToShoot());
    Logger.recordOutput(
        "SuperStructure/Flags/FlywheelAlmostAtSetpoint", isFlywheelAlmostAtRPMToShoot());
    Logger.recordOutput("SuperStructure/Flags/KickerAtSetpoint", isKickerAtRPMToShoot());
    Logger.recordOutput("SuperStructure/Flags/DrivetrainAligned", isAtSetpointAngle());
    Logger.recordOutput("SuperStructure/Flags/hasElement", hasElement);
    Logger.recordOutput("SuperStructure/Flags/isConveyorReversing", isConveyorReversing);

    Logger.recordOutput(
        "SuperStructure/Feedback/FlywheelRPM", flywheel.getLeaderInputs().velocity.in(RPM));
    Logger.recordOutput(
        "SuperStructure/Feedback/KickerRPM", kicker.getMotorInputs().velocity.in(RPM));

    Logger.recordOutput("Subsystems/PDH/totalCurrent", pd.getTotalCurrent());
    Logger.recordOutput("Subsystems/PDH/voltage", pd.getVoltage());
    Logger.recordOutput("Subsystems/PDH/totalEnergy", pd.getTotalEnergy());
    Logger.recordOutput("Subsystems/PDH/totalPower", pd.getTotalPower());
  }
}
