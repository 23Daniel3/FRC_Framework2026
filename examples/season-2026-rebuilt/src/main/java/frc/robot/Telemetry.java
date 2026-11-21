package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * Telemetry helper that publishes telemetry to NetworkTables / SmartDashboard, writes CTRE
 * SignalLogger entries, and records via org.littletonrobotics.junction.Logger
 * (AdvantageKit-friendly).
 */
public class Telemetry {
  private final double MaxSpeed;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    MaxSpeed = maxSpeed;

    // Keep existing CTRE SignalLogger behavior

    // Note: AdvantageKit (and its file storage) listens to org.littletonrobotics.junction.Logger
    // so we simply record outputs with Logger.recordOutput(...) below.
  }

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable driveStateTable = inst.getTable("DriveState");
  private final StructPublisher<Pose2d> drivePose =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> driveSpeeds =
      driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
      driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
      driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
      driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
  private final DoublePublisher driveTimestamp =
      driveStateTable.getDoubleTopic("Timestamp").publish();
  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  /* Robot pose for field positioning */
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] m_moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] m_moduleSpeeds =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] m_moduleDirections =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      };

  private final double[] m_poseArray = new double[3];
  private final double[] m_moduleStatesArray = new double[8];
  private final double[] m_moduleTargetsArray = new double[8];

  /**
   * Accept the swerve drive state and telemeterize it to SmartDashboard, SignalLogger and
   * AdvantageKit (via org.littletonrobotics.junction.Logger).
   */
  public void telemeterize(SwerveDriveState state) {
    if (state == null) return;

    /* --- NetworkTables / NT publishers --- */
    drivePose.set(state.Pose);
    driveSpeeds.set(state.Speeds);
    driveModuleStates.set(state.ModuleStates);
    driveModuleTargets.set(state.ModuleTargets);
    driveModulePositions.set(state.ModulePositions);
    driveTimestamp.set(state.Timestamp);
    driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

    /* Also write to signal logger (existing behavior) */
    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    for (int i = 0; i < 4; ++i) {
      m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
      m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
      m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
      m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
    }

    SignalLogger.writeDoubleArray("Subsystems/Drivetrain/DriveState/Pose", m_poseArray);
    SignalLogger.writeDoubleArray(
        "Subsystems/Drivetrain/DriveState/ModuleStates", m_moduleStatesArray);
    SignalLogger.writeDoubleArray(
        "Subsystems/Drivetrain/DriveState/ModuleTargets", m_moduleTargetsArray);
    SignalLogger.writeDouble(
        "Subsystems/Drivetrain/DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

    /* --- Publish field pose for Field2d (NT) --- */
    fieldTypePub.set("Field2d");
    fieldPub.set(m_poseArray);

    /* --- SmartDashboard visualization (Mechanism2d) --- */
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

      SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
    }

    /* --- AdvantageKit / Logger outputs (records for replay) --- */
    // Pose values (x, y, heading)
    Logger.recordOutput("Subsystems/Drivetrain/DriveState/Pose/X", state.Pose.getX());
    Logger.recordOutput("Subsystems/Drivetrain/DriveState/Pose/Y", state.Pose.getY());
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/Pose/HeadingDeg", state.Pose.getRotation().getDegrees());

    // Full pose as array (helps some replay viewers)
    Logger.recordOutput("Subsystems/Drivetrain/DriveState/Pose/Array", m_poseArray);

    // Speeds
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/Speeds/Vx", state.Speeds.vxMetersPerSecond);
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/Speeds/Vy", state.Speeds.vyMetersPerSecond);
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/Speeds/Omega", state.Speeds.omegaRadiansPerSecond);

    // Module states and targets as arrays as well as per-module logging
    Logger.recordOutput("Subsystems/Drivetrain/DriveState/ModuleStates/Array", m_moduleStatesArray);
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/ModuleTargets/Array", m_moduleTargetsArray);
    for (int i = 0; i < 4; ++i) {
      Logger.recordOutput(
          String.format("Subsystems/Drivetrain/DriveState/Module/%d/AngleRad", i),
          state.ModuleStates[i].angle.getRadians());
      Logger.recordOutput(
          String.format("Subsystems/Drivetrain/DriveState/Module/%d/SpeedMps", i),
          state.ModuleStates[i].speedMetersPerSecond);

      Logger.recordOutput(
          String.format("Subsystems/Drivetrain/DriveState/Module/%d/TargetAngleRad", i),
          state.ModuleTargets[i].angle.getRadians());
      Logger.recordOutput(
          String.format("Subsystems/Drivetrain/DriveState/Module/%d/TargetSpeedMps", i),
          state.ModuleTargets[i].speedMetersPerSecond);

      // Optionally log module positions if available
      if (state.ModulePositions != null
          && state.ModulePositions.length > i
          && state.ModulePositions[i] != null) {
        Logger.recordOutput(
            String.format("Subsystems/Drivetrain/DriveState/Module/%d/PositionMeters", i),
            state.ModulePositions[i].distanceMeters);
      }
    }

    // Odometry timing
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/OdometryPeriodSeconds", state.OdometryPeriod);
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/OdometryFrequencyHz", 1.0 / state.OdometryPeriod);

    // Also log a compact snapshot to SignalLogger (already done above); duplicate for Logger as
    // well
    Logger.recordOutput("Subsystems/Drivetrain/DriveState/Snapshot/PoseArray", m_poseArray);
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/Snapshot/ModuleStatesArray", m_moduleStatesArray);
    Logger.recordOutput(
        "Subsystems/Drivetrain/DriveState/Snapshot/ModuleTargetsArray", m_moduleTargetsArray);
  }
}
