package frc.lib.calculus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.logger.LoggedTunableNumber;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates the aim angle and ideal RPM for a shot on the move (SOTM).
 *
 * <p>Fully generic: the target is provided via {@code Function<Pose2d, Translation2d>}, allowing
 * multiple instances with different targets (hub, feed, etc).
 *
 * <p>Example usage with two distinct targets:
 *
 * <pre>
 *   // Hub shot: fixed target based on alliance
 *   hubCalc = new ShotOnTheMoveCalculator("SOTM/Hub",
 *       pose -> allianceManager.isBlue() ? HUB_BLUE : HUB_RED, ...);
 *
 *   // Feed shot: dynamic target — the nearest intake pose
 *   feedCalc = new ShotOnTheMoveCalculator("SOTM/Feed",
 *       pose -> nearestOf(pose, LEFT_BLUE, RIGHT_BLUE), ...);
 * </pre>
 */
public class ShotOnTheMoveCalculator {

  /**
   * Immutable physical parameters of the shot mechanics.
   *
   * @param avgWheelDiameterMeters Average flywheel diameter (m)
   * @param ballExitAngleDeg Ball exit angle (degrees, relative to horizontal)
   * @param shooterOffsetMeters Lateral offset of the shooter relative to the robot center (m)
   * @param rpmSmootherAlpha Alpha of the EMA for RPM smoothing (1.0 = no smoothing)
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
   * Performs the complete SOTM calculation for the current cycle.
   *
   * <p>Should be called once per cycle (in the SuperStructure periodic) regardless of whether
   * shooting, to keep the logs always updated.
   *
   * @return {@link ShotParameters} with calculated angle and RPM
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
        robotVelocity.getX() * unitToTarget.getX() + robotVelocity.getY() * unitToTarget.getY();

    double tangentialVelocity =
        robotVelocity.getY() * unitToTarget.getX() - robotVelocity.getX() * unitToTarget.getY();

    double lateralDisplacement = tangentialVelocity * timeOfFlight * aimScalar.get();
    double compensationAngle = Math.atan2(lateralDisplacement, distance);

    double offsetRatio = Math.max(-0.99, Math.min(0.99, config.shooterOffsetMeters() / distance));
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

  /** Returns the last calculated result without reprocessing. */
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
