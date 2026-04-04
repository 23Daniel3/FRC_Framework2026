package frc.lib.calculus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.logger.LoggedTunableNumber;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * Calcula ângulo de mira e RPM ideal para um tiro em movimento (SOTM).
 *
 * <p>Totalmente genérico: o target é fornecido via {@code Function<Pose2d, Translation2d>},
 * permitindo múltiplas instâncias com targets diferentes (hub, feed, etc).
 *
 * <p>Exemplo de uso com dois targets distintos:
 * <pre>
 *   // Hub shot: target fixo baseado na aliança
 *   hubCalc = new ShotOnTheMoveCalculator("SOTM/Hub",
 *       pose -> allianceManager.isBlue() ? HUB_BLUE : HUB_RED, ...);
 *
 *   // Feed shot: target dinâmico — a pose de intake mais próxima
 *   feedCalc = new ShotOnTheMoveCalculator("SOTM/Feed",
 *       pose -> nearestOf(pose, LEFT_BLUE, RIGHT_BLUE), ...);
 * </pre>
 */
public class ShotOnTheMoveCalculator {

  /**
   * Parâmetros físicos imutáveis da mecânica de tiro.
   *
   * @param avgWheelDiameterMeters Diâmetro médio dos flywheels (m)
   * @param ballExitAngleDeg Ângulo de saída da bola (graus, relativo ao horizontal)
   * @param shooterOffsetMeters Offset lateral do shooter em relação ao centro do robô (m)
   * @param rpmSmootherAlpha Alpha do EMA para suavização do RPM (1.0 = sem suavização)
   */
  public record Config(
      double avgWheelDiameterMeters,
      double ballExitAngleDeg,
      double shooterOffsetMeters,
      double rpmSmootherAlpha) {}

  private final String logPrefix;
  private final Function<Pose2d, Translation2d> targetResolver;
  private final LoggedTunableMap flywheelMap;
  private final LoggedTunableNumber aimScalar;
  private final LoggedTunableNumber rpmScalar;
  private final LoggedTunableNumber shooterEfficiency;
  private final Config config;
  private final ExponentialMovingAverage rpmSmoother;

  private ShotParameters lastResult = ShotParameters.IDLE;

  public ShotOnTheMoveCalculator(
      String logPrefix,
      Function<Pose2d, Translation2d> targetResolver,
      LoggedTunableMap flywheelMap,
      LoggedTunableNumber aimScalar,
      LoggedTunableNumber rpmScalar,
      LoggedTunableNumber shooterEfficiency,
      Config config) {

    this.logPrefix = logPrefix;
    this.targetResolver = targetResolver;
    this.flywheelMap = flywheelMap;
    this.aimScalar = aimScalar;
    this.rpmScalar = rpmScalar;
    this.shooterEfficiency = shooterEfficiency;
    this.config = config;
    this.rpmSmoother = new ExponentialMovingAverage(config.rpmSmootherAlpha());
  }

  /**
   * Executa o cálculo completo de SOTM para o ciclo atual.
   *
   * <p>Deve ser chamado uma vez por ciclo (no periodic da SuperStructure)
   * independente de estar atirando, para manter os logs sempre atualizados.
   *
   * @return {@link ShotParameters} com ângulo e RPM calculados
   */
  public ShotParameters calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    Translation2d target = targetResolver.apply(robotPose);
    Translation2d robotToTarget = target.minus(robotPose.getTranslation());
    double distance = Math.max(robotToTarget.getNorm(), 0.1);

    double baseRPM = flywheelMap.applyThrottle(distance);
    double efficiency = shooterEfficiency.get();

    double vExitTotal = (baseRPM * Math.PI * config.avgWheelDiameterMeters() / 60.0) * efficiency;
    double vExitHorizontal =
        Math.max(vExitTotal * Math.cos(Math.toRadians(config.ballExitAngleDeg())), 1.0);

    double timeOfFlight = distance / vExitHorizontal;

    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d unitToTarget = robotToTarget.div(distance);

    double radialVelocity =
        robotVelocity.getX() * unitToTarget.getX()
            + robotVelocity.getY() * unitToTarget.getY();

    double tangentialVelocity =
        robotVelocity.getY() * unitToTarget.getX()
            - robotVelocity.getX() * unitToTarget.getY();

    double lateralDisplacement = tangentialVelocity * timeOfFlight * aimScalar.get();
    double compensationAngle = Math.atan2(lateralDisplacement, distance);

    double offsetRatio =
        Math.max(-0.99, Math.min(0.99, config.shooterOffsetMeters() / distance));
    double offsetCorrection = Math.asin(offsetRatio);

    Rotation2d aimAngle =
        robotToTarget.getAngle().plus(new Rotation2d(compensationAngle - offsetCorrection));

    double radialRpmEquivalent =
        (radialVelocity * 60.0) / (Math.PI * config.avgWheelDiameterMeters() * efficiency);
    double smoothedRPM =
        rpmSmoother.calculate(Math.max(0, baseRPM - radialRpmEquivalent * rpmScalar.get()));

    logResult(distance, radialVelocity, tangentialVelocity, timeOfFlight, aimAngle, smoothedRPM);

    lastResult = new ShotParameters(aimAngle, smoothedRPM);
    return lastResult;
  }

  /** Retorna o último resultado calculado sem reprocessar. */
  public ShotParameters getLastResult() {
    return lastResult;
  }

  private void logResult(
      double distance,
      double radialVelocity,
      double tangentialVelocity,
      double timeOfFlight,
      Rotation2d aimAngle,
      double rpm) {
    Logger.recordOutput(logPrefix + "/Distance", distance);
    Logger.recordOutput(logPrefix + "/RadialVelocity", radialVelocity);
    Logger.recordOutput(logPrefix + "/TangentialVelocity", tangentialVelocity);
    Logger.recordOutput(logPrefix + "/TimeOfFlight", timeOfFlight);
    Logger.recordOutput(logPrefix + "/AimAngleDeg", aimAngle.getDegrees());
    Logger.recordOutput(logPrefix + "/RPM", rpm);
  }
}