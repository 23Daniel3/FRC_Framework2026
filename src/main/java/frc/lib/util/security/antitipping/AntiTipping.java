package frc.lib.util.security.antitipping;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * {@code AntiTipping} provides a proportional correction system to prevent the robot from tipping
 * over during operation.
 */
public class AntiTipping {

  private double tippingThresholdDegrees;
  private double maxCorrectionSpeed; // m/s
  private double kP; // proportional gain

  private double pitch = 0.0;
  private double roll = 0.0;
  private double correctionSpeed = 0.0;
  private double inclinationMagnitude = 0.0;
  private double yawDirectionDeg = 0.0;
  private boolean isTipping = false;
  private Rotation2d tiltDirection = new Rotation2d();
  private ChassisSpeeds speeds = new ChassisSpeeds();

  public AntiTipping(double kP, double tippingThresholdDegrees, double maxCorrectionSpeed) {
    this.kP = kP;
    this.tippingThresholdDegrees = tippingThresholdDegrees;
    this.maxCorrectionSpeed = maxCorrectionSpeed;
  }

  public void setTippingThreshold(double degrees) {
    this.tippingThresholdDegrees = degrees;
  }

  public void setMaxCorrectionSpeed(double speedMetersPerSecond) {
    this.maxCorrectionSpeed = speedMetersPerSecond;
  }

  public void setKp(double kP) {
    this.kP = kP;
  }

  public Optional<ChassisSpeeds> calculate(Rotation3d robotRotation3d) {
    pitch = robotRotation3d.getY();
    roll = robotRotation3d.getX();

    double tippingThresholdRadians = Units.degreesToRadians(tippingThresholdDegrees);

    isTipping =
        Math.abs(pitch) > tippingThresholdRadians || Math.abs(roll) > tippingThresholdRadians;

    tiltDirection = new Rotation2d(Math.atan2(roll, pitch));
    yawDirectionDeg = tiltDirection.getDegrees();

    inclinationMagnitude = Math.hypot(Units.radiansToDegrees(pitch), Units.radiansToDegrees(roll));

    correctionSpeed = kP * inclinationMagnitude;
    correctionSpeed = MathUtil.clamp(correctionSpeed, -maxCorrectionSpeed, maxCorrectionSpeed);

    Translation2d correctionVector =
        new Translation2d(0, 1).rotateBy(tiltDirection).times(correctionSpeed);

    speeds = new ChassisSpeeds(correctionVector.getX(), -correctionVector.getY(), 0);

    Logger.recordOutput("Subsystems/Drivetrain/AntiTipping/PitchRad", pitch);
    Logger.recordOutput("Subsystems/Drivetrain/AntiTipping/RollRad", roll);
    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/PitchDeg", Units.radiansToDegrees(pitch));
    Logger.recordOutput("Subsystems/Drivetrain/AntiTipping/RollDeg", Units.radiansToDegrees(roll));

    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/TippingThresholdDeg", tippingThresholdDegrees);
    Logger.recordOutput("Subsystems/Drivetrain/AntiTipping/kP", kP);
    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/MaxCorrectionSpeedMps", maxCorrectionSpeed);

    Logger.recordOutput("Subsystems/Drivetrain/AntiTipping/IsTipping", isTipping);
    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/InclinationMagnitudeDeg", inclinationMagnitude);
    Logger.recordOutput("Subsystems/Drivetrain/AntiTipping/TiltDirectionDeg", yawDirectionDeg);
    Logger.recordOutput("Subsystems/Drivetrain/AntiTipping/CorrectionSpeedMps", correctionSpeed);

    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/CorrectionVectorX", correctionVector.getX());
    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/CorrectionVectorY", correctionVector.getY());

    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/ChassisSpeeds/Vx", speeds.vxMetersPerSecond);
    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/ChassisSpeeds/Vy", speeds.vyMetersPerSecond);
    Logger.recordOutput(
        "Subsystems/Drivetrain/AntiTipping/ChassisSpeeds/Omega", speeds.omegaRadiansPerSecond);

    /* --------------------------------------------------- */

    return isTipping ? Optional.of(speeds) : Optional.empty();
  }

  public double getLastInclinationMagnitude() {
    return inclinationMagnitude;
  }

  public double getLastYawDirectionDeg() {
    return yawDirectionDeg;
  }

  public boolean isTipping() {
    return isTipping;
  }

  public ChassisSpeeds getVelocityAntiTipping() {
    return speeds;
  }

  public Rotation2d getLastTiltDirection() {
    return tiltDirection;
  }
}
