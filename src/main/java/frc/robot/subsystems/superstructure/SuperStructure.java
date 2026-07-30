package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.AllianceManager;
import frc.game.FieldConstants;
import frc.lib.calculus.ShotParameters;
import frc.lib.interfaces.fsm.StateMachine;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.robot.Constants.RobotRequest;
import frc.robot.Constants.RobotState;
import frc.robot.power.RobotPowerDistribution;
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

  private final ShooterTargetCalculator shooterCalculator;

  private RobotRequest robotRequest = RobotRequest.IDLE;

  private final AllianceManager allianceManager = AllianceManager.getInstance();
  private final StateMachine<RobotState> generalFsm;

  public SuperStructure(Conveyor conveyor, Drivetrain drivetrain, Intake intake, Shooter shooter) {

    this.conveyor = conveyor;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.shooter = shooter;

    this.shooterCalculator = new ShooterTargetCalculator(allianceManager);

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

    Pose2d pose = drivetrain.getPose();
    ChassisSpeeds speeds = drivetrain.getRobotVelocity();
    shooterCalculator.update(pose, speeds);

    // Alimenta o shooter continuamente com o RPM calculado pelo SOTM.
    shooter.setVelocity(RPM.of(getActiveShotParameters().rpm()));

    generalFsm.update();

    log();

    PeriodicTimer.stop(getName());
  }

  private IntakeRequest intakeRequest = IntakeRequest.STOP;
  private ShooterRequest shooterRequest = ShooterRequest.STOP;
  private ConveyorRequest conveyorRequest = ConveyorRequest.STOP;

  public void setRequest(RobotRequest request) {
    this.robotRequest = request;
  }

  /** Estado atual da FSM geral (consumido pelo observador de LEDs e por telemetria). */
  public RobotState getRobotState() {
    return generalFsm.getCurrentState();
  }

  public ConveyorRequest getConveyorRequest() {
    return conveyorRequest;
  }

  public IntakeRequest getIntakeRequest() {
    return intakeRequest;
  }

  public ShooterRequest getShooterRequest() {
    return shooterRequest;
  }

  public ShotParameters getActiveShotParameters() {
    return shooterCalculator.getActiveShotParameters(isInAllianceZone());
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

  private void setSubsystemRequests(
      ShooterRequest shooterReq, IntakeRequest intakeReq, ConveyorRequest conveyorReq) {
    this.shooterRequest = shooterReq;
    this.intakeRequest = intakeReq;
    this.conveyorRequest = conveyorReq;
  }

  private void configureGeneralFSM() {
    generalFsm
        .state(RobotState.IDLE)
        .onEnter(
            () -> {
              setSubsystemRequests(ShooterRequest.STOP, IntakeRequest.STOP, ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    generalFsm
        .state(RobotState.IDLING)
        .onEnter(
            () -> {
              setSubsystemRequests(ShooterRequest.STOP, IntakeRequest.STOP, ConveyorRequest.STOP);
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
              setSubsystemRequests(
                  ShooterRequest.STOP, IntakeRequest.COLLECT, ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            });

    generalFsm
        .state(RobotState.GOING_COLLECT)
        .onEnter(
            () -> {
              setSubsystemRequests(
                  ShooterRequest.STOP, IntakeRequest.COLLECT, ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            })
        .transitionTo(RobotState.COLLECTING, () -> intake.atGoal());

    generalFsm
        .state(RobotState.GOING_SHOOT)
        .onEnter(
            () -> {
              setSubsystemRequests(ShooterRequest.SHOOT, IntakeRequest.OUT, ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(RobotState.SHOOTING, () -> shooter.readyToShoot() && isAtSetpointAngle());

    generalFsm
        .state(RobotState.SHOOTING)
        .onEnter(
            () -> {
              this.conveyorRequest = ConveyorRequest.RUN;
            })
        .transitionTo(RobotState.SHOOTING_RECOVERY, () -> !shooter.readyToShoot())
        .transitionTo(RobotState.GOING_SHOOT, () -> !isAtSetpointAngle());

    generalFsm
        .state(RobotState.SHOOTING_RECOVERY)
        .onEnter(
            () -> {
              setSubsystemRequests(
                  ShooterRequest.SHOOT, IntakeRequest.OUT, ConveyorRequest.RUN_SLOW);
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(RobotState.SHOOTING, () -> shooter.readyToShoot());

    generalFsm
        .state(RobotState.GOING_COLLECT_SHOOT)
        .onEnter(
            () -> {
              setSubsystemRequests(
                  ShooterRequest.SHOOT, IntakeRequest.COLLECT, ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(
            RobotState.COLLECT_SHOOTING, () -> shooter.readyToShoot() && isAtSetpointAngle());

    generalFsm
        .state(RobotState.COLLECT_SHOOTING)
        .onEnter(
            () -> {
              this.conveyorRequest = ConveyorRequest.RUN;
            })
        .transitionTo(RobotState.COLLECT_SHOOTING_RECOVERY, () -> !shooter.readyToShoot())
        // Perder o alinhamento em modo COLLECT_SHOOT volta para GOING_COLLECT_SHOOT
        // (nao GOING_SHOOT), para o intake continuar coletando durante o realinhamento.
        .transitionTo(RobotState.GOING_COLLECT_SHOOT, () -> !isAtSetpointAngle());

    generalFsm
        .state(RobotState.COLLECT_SHOOTING_RECOVERY)
        .onEnter(
            () -> {
              setSubsystemRequests(
                  ShooterRequest.SHOOT, IntakeRequest.COLLECT, ConveyorRequest.RUN_SLOW);
              drivetrain.setMaxSpeed(SuperStructureConstants.MAX_VELOCITY_TO_SHOOT);
            })
        .transitionTo(RobotState.COLLECT_SHOOTING, () -> shooter.readyToShoot());

    generalFsm
        .state(RobotState.CLOSING)
        .onEnter(
            () -> {
              setSubsystemRequests(ShooterRequest.STOP, IntakeRequest.IN, ConveyorRequest.STOP);
              drivetrain.setMaxSpeed(MetersPerSecond.of(DrivetrainConstants.MAX_SPEED));
            })
        .transitionTo(
            RobotState.CLOSED, () -> intake.atGoal() && conveyor.atGoal() && shooter.atGoal());

    generalFsm
        .state(RobotState.CLOSED)
        .onEnter(
            () -> {
              setSubsystemRequests(ShooterRequest.STOP, IntakeRequest.IN, ConveyorRequest.STOP);
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

  private boolean isInAllianceZone() {
    return allianceManager.isInAllianceZone(
        FieldConstants.Zones.getGeneralZone(drivetrain.getPose().getTranslation()));
  }

  private void log() {
    if (getCurrentCommand() != null) {
      Logger.recordOutput("SuperStructure/Command", getCurrentCommand().getName());
    }

    Logger.recordOutput("SuperStructure/GeneralFSM/State", generalFsm.getCurrentState());
    Logger.recordOutput("SuperStructure/GeneralFSM/Request", robotRequest);
    Logger.recordOutput("SuperStructure/GeneralFSM/TimeInState", generalFsm.getTimeInState());
    Logger.recordOutput("SuperStructure/Flags/DrivetrainAligned", isAtSetpointAngle());
    Logger.recordOutput("SuperStructure/Flags/InAllianceZone", isInAllianceZone());

    shooterCalculator.log(drivetrain.getPose(), isInAllianceZone());

    RobotPowerDistribution.getInstance().log();
  }
}
