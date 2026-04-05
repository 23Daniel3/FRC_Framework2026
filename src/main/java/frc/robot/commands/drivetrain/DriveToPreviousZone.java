package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.game.FieldConstants.Poses;
import frc.game.FieldConstants.Zones;
import frc.lib.interfaces.fsm.StateMachine;
import frc.lib.util.AllianceSelector;
import frc.lib.zones.Polygon2d;
import frc.robot.factories.DrivetrainCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class DriveToPreviousZone extends Command {

  private enum State {
    CALCULATE,
    DRIVE_TO_TRENCH,
    CROSSING_TRENCH,
    FINISHED
  }

  private final Drivetrain drivetrain;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final StateMachine<State> fsm;

  private Command activeCommand;
  private Polygon2d targetTrenchZone;
  private Pose2d intermediatePose;

  public DriveToPreviousZone(
      Drivetrain drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    this.drivetrain = drivetrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;

    addRequirements(drivetrain);

    this.fsm = new StateMachine<>("DriveToPrevious", State.class, State.CALCULATE);
    configureStateMachine();
  }

  private void configureStateMachine() {
    // 1. Decide o alvo
    fsm.state(State.CALCULATE)
        .transitionTo(State.DRIVE_TO_TRENCH, () -> targetTrenchZone != null)
        .transitionTo(State.FINISHED, () -> targetTrenchZone == null);

    // 2. Dirige até o centro da trincheira alvo
    fsm.state(State.DRIVE_TO_TRENCH)
        .onEnter(
            () -> {
              activeCommand = DrivetrainCommands.driveToPose(drivetrain, () -> intermediatePose);
              activeCommand.initialize();
            })
        .onUpdate(
            () -> {
              if (activeCommand != null) activeCommand.execute();
            })
        .onExit(
            () -> {
              if (activeCommand != null) activeCommand.end(true);
            })
        // Se entrar na zona da trincheira, muda para sucção
        .transitionTo(
            State.CROSSING_TRENCH,
            () -> targetTrenchZone.contains(drivetrain.getPose().getTranslation()))
        // Se chegou no ponto intermediário mas não entrou no poligono (caso raro), tenta forçar a
        // sucção ou finaliza
        .transitionTo(
            State.CROSSING_TRENCH, () -> activeCommand != null && activeCommand.isFinished());

    // 3. Atravessa usando Joystick + Zone Suction
    fsm.state(State.CROSSING_TRENCH)
        .onEnter(
            () -> {
              activeCommand =
                  DrivetrainCommands.joystickDriveWithZoneSuction(
                      drivetrain, xSupplier, ySupplier, omegaSupplier, targetTrenchZone, 1);
              activeCommand.initialize();
            })
        .onUpdate(
            () -> {
              if (activeCommand != null) activeCommand.execute();
            })
        .onExit(
            () -> {
              if (activeCommand != null) activeCommand.end(true);
            })
        // Se o comando de sucção terminar ou o robô sair da zona -> FIM
        .transitionTo(State.FINISHED, () -> activeCommand != null && activeCommand.isFinished())
        .transitionTo(
            State.FINISHED,
            () -> !targetTrenchZone.contains(drivetrain.getPose().getTranslation()));
  }

  @Override
  public void initialize() {
    calculateTargets();
    fsm.forceState(State.CALCULATE);
  }

  @Override
  public void execute() {
    fsm.update();
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return fsm.getCurrentState() == State.FINISHED;
  }

  private void calculateTargets() {
    Pose2d robotPose = drivetrain.getPose();
    Alliance alliance =
        DriverStation.getAlliance().orElse(AllianceSelector.getInstance().getAlliance());
    boolean isBlue = alliance == Alliance.Blue;

    boolean inNeutral = Zones.NEUTRAL_ZONE.contains(robotPose.getTranslation());
    boolean inRed = Zones.ALLIANCE_RED_ZONE.contains(robotPose.getTranslation());

    if (isBlue) {
      if (inRed) setupPath(Alliance.Red);
      else if (inNeutral) setupPath(Alliance.Blue);
      else clearTargets();
    } else {
      // Red Alliance
      boolean inBlueZone = Zones.ALLIANCE_BLUE_ZONE.contains(robotPose.getTranslation());
      if (inBlueZone) setupPath(Alliance.Blue);
      else if (inNeutral) setupPath(Alliance.Red);
      else clearTargets();
    }
  }

  private void setupPath(Alliance trenchAlliance) {
    targetTrenchZone = getNearestTrenchZone(drivetrain.getPose(), trenchAlliance);
    intermediatePose =
        new Pose2d(
            getNearestTrenchCenter(drivetrain.getPose(), trenchAlliance), drivetrain.getRotation());
  }

  private void clearTargets() {
    targetTrenchZone = null;
    intermediatePose = null;
  }

  private Polygon2d getNearestTrenchZone(Pose2d pose, Alliance alliance) {
    boolean isBlue = alliance == Alliance.Blue;
    Polygon2d left = isBlue ? Zones.TRENCH_LEFT_BLUE : Zones.TRENCH_LEFT_RED;
    Polygon2d right = isBlue ? Zones.TRENCH_RIGHT_BLUE : Zones.TRENCH_RIGHT_RED;
    return getDist(pose, left.getCenter()) <= getDist(pose, right.getCenter()) ? left : right;
  }

  private Translation2d getNearestTrenchCenter(Pose2d pose, Alliance alliance) {
    boolean isBlue = alliance == Alliance.Blue;
    Translation2d left =
        isBlue
            ? Poses.CENTER_TRENCH_LEFT_BLUE.getTranslation()
            : Poses.CENTER_TRENCH_LEFT_RED.getTranslation();
    Translation2d right =
        isBlue
            ? Poses.CENTER_TRENCH_RIGHT_BLUE.getTranslation()
            : Poses.CENTER_TRENCH_RIGHT_RED.getTranslation();
    return getDist(pose, left) <= getDist(pose, right) ? left : right;
  }

  private double getDist(Pose2d pose, Translation2d point) {
    return pose.getTranslation().getDistance(point);
  }
}
