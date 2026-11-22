package frc.robot.commands.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class CalibrateWheelDiameter extends Command {
  private final Drivetrain swerve;
  private final double targetAngleDeg;
  private final double angularSpeedRadPerSec;
  private final double driveGearRatio;

  private final Timer timer = new Timer();
  private boolean isRecording = false;

  private double startGyroYawDeg;
  private double[] startModuleRotations;
  private Translation2d[] moduleLocations;

  private StatusSignal<Angle> pigeonYawSignal;

  /**
   * High-precision wheel diameter calibration command.
   *
   * <p>The robot will rotate for 1 second to align the swerve modules and eliminate mechanical
   * backlash. After stabilization, angular displacement and wheel rotations are measured to compute
   * the effective wheel diameter.
   *
   * @param swerve The drivetrain subsystem (Phoenix 6 based).
   * @param targetAngleDegrees Total rotation in degrees to measure after stabilization (e.g. 720).
   * @param angularSpeedRadPerSec Constant angular speed during calibration (e.g. 1.0 rad/s).
   * @param driveGearRatio Drive reduction ratio (e.g. 6.75 for L2, 6.12 for L3).
   */
  public CalibrateWheelDiameter(
      Drivetrain swerve,
      double targetAngleDegrees,
      double angularSpeedRadPerSec,
      double driveGearRatio) {

    this.swerve = swerve;
    this.targetAngleDeg = Math.abs(targetAngleDegrees);
    this.angularSpeedRadPerSec = angularSpeedRadPerSec;
    this.driveGearRatio = driveGearRatio;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.restart();
    isRecording = false;

    pigeonYawSignal = swerve.getPigeon2().getYaw();
    swerve.getPigeon2().setYaw(0);

    moduleLocations = swerve.getModuleLocations();

    swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, angularSpeedRadPerSec));

    Logger.recordOutput("Calibration/Status", "Aligning Wheels (1s)...");
  }

  @Override
  public void execute() {
    swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, angularSpeedRadPerSec));

    if (!isRecording && timer.hasElapsed(1.0)) {
      startRecording();
    }

    if (isRecording) {
      double currentDelta = getCurrentGyroDelta();
      Logger.recordOutput("Calibration/ProgressDegrees", currentDelta);
      Logger.recordOutput("Calibration/TargetDegrees", targetAngleDeg);
      Logger.recordOutput("Calibration/Status", "Measuring...");
    }
  }

  private void startRecording() {
    isRecording = true;

    pigeonYawSignal.refresh();
    startGyroYawDeg = pigeonYawSignal.getValueAsDouble();

    var modules = swerve.getModules();
    startModuleRotations = new double[modules.length];

    for (int i = 0; i < modules.length; i++) {
      TalonFX driveMotor = (TalonFX) modules[i].getDriveMotor();
      driveMotor.getRotorPosition().refresh();
      startModuleRotations[i] = driveMotor.getRotorPosition().getValueAsDouble();
    }

    System.out.println("[CalibrateWheel] Wheels aligned. Starting measurement...");
  }

  private double getCurrentGyroDelta() {
    if (!isRecording) return 0.0;

    pigeonYawSignal.refresh();
    double currentYaw = pigeonYawSignal.getValueAsDouble();

    return Math.abs(currentYaw - startGyroYawDeg);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();

    if (interrupted || !isRecording) {
      Logger.recordOutput("Calibration/Status", "Interrupted/Failed");
      return;
    }

    calculateAndLogResults();
  }

  private void calculateAndLogResults() {
    double totalGyroDeltaDeg = getCurrentGyroDelta();
    double totalGyroDeltaRad = Math.toRadians(totalGyroDeltaDeg);

    if (totalGyroDeltaRad < 0.1) {
      System.out.println("[CalibrateWheel] ERROR: Insufficient rotation.");
      return;
    }

    Logger.recordOutput("Calibration/Status", "Finished");
    Logger.recordOutput("Calibration/FinalGyroDeltaDegrees", totalGyroDeltaDeg);

    System.out.println("\n=== CALIBRATION RESULTS (Phoenix 6) ===");
    System.out.printf("Gear Ratio Used: %.4f%n", driveGearRatio);
    System.out.printf("Measured Rotation (Gyro): %.4f degrees%n", totalGyroDeltaDeg);

    var modules = swerve.getModules();
    double[] radii = new double[modules.length];
    int validCount = 0;

    for (int i = 0; i < modules.length; i++) {
      TalonFX driveMotor = (TalonFX) modules[i].getDriveMotor();
      driveMotor.getRotorPosition().refresh();
      double endRotation = driveMotor.getRotorPosition().getValueAsDouble();
      double deltaMotorRotations = Math.abs(endRotation - startModuleRotations[i]);

      double driveBaseRadius = moduleLocations[i].getNorm();

      if (deltaMotorRotations <= 1e-9) {
        radii[i] = Double.NaN;
        Logger.recordOutput("Calibration/Module_" + i + "/RawMotorRotations", deltaMotorRotations);
        System.out.printf(">> Module %d: ERROR: zero rotations recorded%n", i);
        continue;
      }

      double wheelRadiusMeters =
          (totalGyroDeltaRad * driveBaseRadius * driveGearRatio)
              / (deltaMotorRotations * 2.0 * Math.PI);

      radii[i] = wheelRadiusMeters;
      validCount++;

      double wheelDiameterInches = (wheelRadiusMeters * 2.0) * 39.3701;

      Logger.recordOutput("Calibration/Module_" + i + "/CalculatedRadiusMeters", wheelRadiusMeters);
      Logger.recordOutput(
          "Calibration/Module_" + i + "/CalculatedDiameterInches", wheelDiameterInches);
      Logger.recordOutput("Calibration/Module_" + i + "/RawMotorRotations", deltaMotorRotations);

      System.out.printf(
          ">> Module %d: Diameter = %.5f inches | Radius = %.5f meters | RawRot = %.5f%n",
          i, wheelDiameterInches, wheelRadiusMeters, deltaMotorRotations);
    }

    if (validCount == 0) {
      System.out.println("[CalibrateWheel] ERROR: No valid module measurements.");
      return;
    }

    double sum = 0.0;
    int count = 0;
    for (double r : radii) {
      if (!Double.isNaN(r)) {
        sum += r;
        count++;
      }
    }
    double mean = sum / count;

    double varSum = 0.0;
    for (double r : radii) {
      if (!Double.isNaN(r)) {
        double d = r - mean;
        varSum += d * d;
      }
    }
    double stddev = Math.sqrt(varSum / count);

    double lower = mean - Math.max(0.05 * mean, 3.0 * stddev);
    double upper = mean + Math.max(0.05 * mean, 3.0 * stddev);

    double filteredSum = 0.0;
    int filteredCount = 0;
    for (double r : radii) {
      if (!Double.isNaN(r) && r >= lower && r <= upper) {
        filteredSum += r;
        filteredCount++;
      }
    }

    double finalRadiusMeters;
    if (filteredCount > 0) {
      finalRadiusMeters = filteredSum / filteredCount;
    } else {
      finalRadiusMeters = mean;
    }

    double finalDiameterInches = (finalRadiusMeters * 2.0) * 39.3701;

    Logger.recordOutput("Calibration/Final/RadiusMeters", finalRadiusMeters);
    Logger.recordOutput("Calibration/Final/DiameterInches", finalDiameterInches);
    Logger.recordOutput("Calibration/Final/UsedModules", filteredCount > 0 ? filteredCount : count);

    System.out.println("-----------------------------------------");
    System.out.printf(
        "FINAL (averaged) Radius = %.6f m | Diameter = %.6f in%n",
        finalRadiusMeters, finalDiameterInches);
    System.out.println("=========================================\n");
  }

  @Override
  public boolean isFinished() {
    return isRecording && getCurrentGyroDelta() >= targetAngleDeg;
  }
}
