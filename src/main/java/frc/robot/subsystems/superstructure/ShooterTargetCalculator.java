package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.game.AllianceManager;
import frc.game.FieldConstants.Poses;
import frc.lib.calculus.LoggedTunableMap;
import frc.lib.calculus.ShotOnTheMoveCalculator;
import frc.lib.calculus.ShotParameters;
import frc.lib.logger.LoggedTunableNumber;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Isolamento do sistema de cálculo e targeting do Shooter. Responsável por Shot-On-The-Move (Hub e
 * Feed), mapa de calibração do flywheel, parâmetros dinâmicos e resolução de alvos baseada na
 * aliança.
 */
public class ShooterTargetCalculator {

  private final AllianceManager allianceManager;

  private final ShotOnTheMoveCalculator hubCalculator;
  private final ShotOnTheMoveCalculator feedCalculator;

  private final LoggedTunableNumber aimScalar = new LoggedTunableNumber("SOTM/AimScalar", -0.8);
  private final LoggedTunableNumber rpmScalar = new LoggedTunableNumber("SOTM/RPMScalar", 0.13);
  private final LoggedTunableNumber shooterEfficiency =
      new LoggedTunableNumber("SOTM/ShooterEfficiency", 1.2);

  private final LoggedTunableMap flywheelMap =
      new LoggedTunableMap(
          "FlywheelCalibrate/Flywheel", true, ShooterConstants.FLYWHEEL_CALIBRATION_MAP);

  public ShooterTargetCalculator(AllianceManager allianceManager) {
    this.allianceManager = allianceManager;

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
  }

  /** Atualiza os mapas de calibração e realiza os cálculos de ShotOnTheMove a cada ciclo. */
  public void update(Pose2d pose, ChassisSpeeds speeds) {
    flywheelMap.calculate();
    hubCalculator.calculate(pose, speeds);
    feedCalculator.calculate(pose, speeds);
  }

  /**
   * Retorna os parâmetros de tiro ativos (Hub se estiver na zona de aliança, Feed caso contrário).
   */
  public ShotParameters getActiveShotParameters(boolean inAllianceZone) {
    return inAllianceZone ? hubCalculator.getLastResult() : feedCalculator.getLastResult();
  }

  /** Distância do robô até o centro do Hub. */
  public double distanceFromRobotToHub(Pose2d robotPose) {
    return robotPose
        .getTranslation()
        .getDistance(
            allianceManager.isBlue()
                ? Poses.HUB_CENTER_BLUE.getTranslation()
                : Poses.HUB_CENTER_RED.getTranslation());
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

  /** Registra os dados de telemetria do targeting do shooter. */
  public void log(Pose2d robotPose, boolean inAllianceZone) {
    ShotParameters active = getActiveShotParameters(inAllianceZone);
    Logger.recordOutput("SuperStructure/Shooting/DistanceToHub", distanceFromRobotToHub(robotPose));
    Logger.recordOutput("SuperStructure/Shooting/ActiveAimAngle", active.aimAngle().getDegrees());
    Logger.recordOutput("SuperStructure/Shooting/ActiveRPM", RPM.of(active.rpm()));
  }
}
