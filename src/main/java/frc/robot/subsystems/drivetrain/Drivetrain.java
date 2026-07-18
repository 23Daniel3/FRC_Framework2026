package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.game.FieldConstants.Poses;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.AllianceSelector;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.GeomUtil;
import frc.lib.util.PeriodicTimer;
import frc.lib.util.security.AngularDynamicSlewRateLimiter;
import frc.lib.util.security.DynamicSlewRateLimiter;
import frc.lib.util.security.antitipping.AntiTipping;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Zones;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();

  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  public final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  public final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  public final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  // TODO rodar as outras rotinas do SysID para extrair todos os valores.
  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

  private final SwerveRequest idle = new SwerveRequest.Idle();

  /*
   * Limite de velocidade com dois donos independentes (arbitragem por minimo):
   * - stateMaxSpeed: escrito pela SuperStructure a cada mudanca de estado (via setMaxSpeed);
   * - pilotMaxSpeed: escrito pelos bindings do piloto (modo lento, via setPilotMaxSpeed).
   * getMaxSpeed() retorna o MENOR dos dois, entao nenhum dos lados desfaz o outro.
   */
  private LinearVelocity stateMaxSpeed = MetersPerSecond.of(DrivetrainConstants.MAX_SPEED);
  private AngularVelocity stateMaxAngularSpeed =
      RadiansPerSecond.of(DrivetrainConstants.MAX_ANGULAR_SPEED);

  private LinearVelocity pilotMaxSpeed = MetersPerSecond.of(DrivetrainConstants.MAX_SPEED);
  private AngularVelocity pilotMaxAngularSpeed =
      RadiansPerSecond.of(DrivetrainConstants.MAX_ANGULAR_SPEED);

  private LinearAcceleration maxAcceleration =
      MetersPerSecondPerSecond.of(DrivetrainConstants.MAX_ACCELERATION);
  private AngularAcceleration maxAngularAcceleration =
      RadiansPerSecondPerSecond.of(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);

  private final AntiTipping antiTipping;

  private final LoggedTunableNumber kpTipping =
      new LoggedTunableNumber("/AntiTipping/kp", DrivetrainConstants.ANTI_TIPPING_KP);

  private final DynamicSlewRateLimiter xLimiter = new DynamicSlewRateLimiter(maxAcceleration);
  private final DynamicSlewRateLimiter yLimiter = new DynamicSlewRateLimiter(maxAcceleration);
  private final AngularDynamicSlewRateLimiter hLimiter =
      new AngularDynamicSlewRateLimiter(maxAngularAcceleration);

  private static final int NUM_MODULES = 4;

  private final String base = "/Drivetrain/Steer/";
  private final LoggedTunableNumber steerKP = new LoggedTunableNumber(base + "kP", 85.0);
  private final LoggedTunableNumber steerKI = new LoggedTunableNumber(base + "kI", 0.0);
  private final LoggedTunableNumber steerKD = new LoggedTunableNumber(base + "kD", 0.0);
  private final LoggedTunableNumber steerKS = new LoggedTunableNumber(base + "kS", 0.0);
  private final LoggedTunableNumber steerKV = new LoggedTunableNumber(base + "kV", 0.0);
  private final LoggedTunableNumber steerKA = new LoggedTunableNumber(base + "kA", 0.0);

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    antiTipping = setupAntiTipping();
    startPoseFromAlliance();
    ConstantsLogger.logConstants(DrivetrainConstants.class, getName());
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    antiTipping = setupAntiTipping();
    startPoseFromAlliance();
    ConstantsLogger.logConstants(DrivetrainConstants.class, getName());
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    antiTipping = setupAntiTipping();
    startPoseFromAlliance();
    ConstantsLogger.logConstants(DrivetrainConstants.class, getName());
  }

  public void startPoseFromAlliance() {
    boolean blueAlliance = AllianceSelector.getInstance().getResolvedAlliance() == Alliance.Blue;

    int station = DriverStation.getLocation().orElse(1);

    Pose2d[] poses = {
      Poses.START_AUTO_LEFT_BLUE, Poses.START_AUTO_CENTER_BLUE, Poses.START_AUTO_RIGHT_BLUE
    };

    Pose2d basePose = poses[MathUtil.clamp(station - 1, 0, poses.length - 1)];
    Pose2d startingPose = blueAlliance ? basePose : GeomUtil.flip(basePose);

    resetPose(startingPose);
  }

  private void configureAutoBuilder() {
    try {
      var config = Constants.ROBOT_CONFIG;
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> AllianceSelector.getInstance().shouldFlip(),
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  private AntiTipping setupAntiTipping() {
    AntiTipping anti =
        new AntiTipping(
            DrivetrainConstants.ANTI_TIPPING_KP,
            DrivetrainConstants.TIPPING_THRESHOLD,
            DrivetrainConstants.MAX_SPEED);
    return anti;
  }

  private void tuning() {
    antiTipping.setKp(kpTipping.get());

    boolean anyChanged =
        steerKP.hasChanged(hashCode())
            || steerKI.hasChanged(hashCode())
            || steerKD.hasChanged(hashCode())
            || steerKS.hasChanged(hashCode())
            || steerKV.hasChanged(hashCode())
            || steerKA.hasChanged(hashCode());

    if (!anyChanged) {
      return;
    }

    Slot0Configs slot = new Slot0Configs();
    slot.kP = steerKP.get();
    slot.kI = steerKI.get();
    slot.kD = steerKD.get();
    slot.kS = steerKS.get();
    slot.kV = steerKV.get();
    slot.kA = steerKA.get();

    for (int i = 0; i < NUM_MODULES; ++i) {
      try {
        var module = this.getModule(i);
        var steerMotor = module.getSteerMotor();
        StatusCode code = steerMotor.getConfigurator().apply(slot);
        if (!code.isOK()) {
          DriverStation.reportWarning(
              "Failed to apply steer Slot0Configs to module " + i + " : " + code.toString(), false);
        } else {
          SignalLogger.writeString("Tuning/Steer/Apply", "module " + i + " applied");
        }
      } catch (Exception ex) {
        DriverStation.reportWarning(
            "Exception while applying steer tunables for module " + i + " : " + ex.toString(),
            false);
      }
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

    ChassisSpeeds limited = applyLimitersRobotRelative(speeds);

    setControl(request.withSpeeds(limited));
  }

  public void driveRobotRelative(double vx, double vy, double omega) {
    driveRobotRelative(
        new ChassisSpeeds(
            MetersPerSecond.of(vx), MetersPerSecond.of(vy), RadiansPerSecond.of(omega)));
  }

  public void driveRobotRelative(LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
    driveRobotRelative(new ChassisSpeeds(vx, vy, omega));
  }

  public void driveFieldRelative(ChassisSpeeds fieldSpeeds) {
    SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

    ChassisSpeeds robotSpeeds = fieldToRobot(fieldSpeeds);
    ChassisSpeeds limited = applyLimitersRobotRelative(robotSpeeds);

    setControl(request.withSpeeds(limited));
  }

  public void driveFieldRelative(double vx, double vy, double omega) {
    driveFieldRelative(
        new ChassisSpeeds(
            MetersPerSecond.of(vx), MetersPerSecond.of(vy), RadiansPerSecond.of(omega)));
  }

  public void driveFieldRelative(LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
    driveFieldRelative(new ChassisSpeeds(vx, vy, omega));
  }

  public void driveWithBalanceRobotRelative(ChassisSpeeds driveSpeeds) {
    SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

    ChassisSpeeds balanced = antiTipping.calculate(getRotation3d()).orElse(driveSpeeds);

    ChassisSpeeds limited = applyLimitersRobotRelative(balanced);

    setControl(request.withSpeeds(limited));
  }

  public void driveWithBalanceRobotRelative(double vx, double vy, double omega) {
    driveWithBalanceRobotRelative(
        new ChassisSpeeds(
            MetersPerSecond.of(vx), MetersPerSecond.of(vy), RadiansPerSecond.of(omega)));
  }

  public void driveWithBalanceRobotRelative(
      LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
    driveWithBalanceRobotRelative(new ChassisSpeeds(vx, vy, omega));
  }

  public void driveWithBalanceFieldRelative(ChassisSpeeds fieldSpeeds) {
    SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

    ChassisSpeeds robotSpeeds = fieldToRobot(fieldSpeeds);

    ChassisSpeeds balanced = antiTipping.calculate(getRotation3d()).orElse(robotSpeeds);

    ChassisSpeeds limited = applyLimitersRobotRelative(balanced);

    setControl(request.withSpeeds(limited));
  }

  public void driveWithBalanceFieldRelative(double vx, double vy, double omega) {
    driveWithBalanceFieldRelative(
        new ChassisSpeeds(
            MetersPerSecond.of(vx), MetersPerSecond.of(vy), RadiansPerSecond.of(omega)));
  }

  public void driveWithBalanceFieldRelative(
      LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
    driveWithBalanceFieldRelative(new ChassisSpeeds(vx, vy, omega));
  }

  public void stop() {
    setControl(idle);
  }

  private ChassisSpeeds applyLimitersRobotRelative(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        xLimiter.calculate(MetersPerSecond.of(speeds.vxMetersPerSecond)).in(MetersPerSecond),
        yLimiter.calculate(MetersPerSecond.of(speeds.vyMetersPerSecond)).in(MetersPerSecond),
        hLimiter
            .calculate(RadiansPerSecond.of(speeds.omegaRadiansPerSecond))
            .in(RadiansPerSecond));
  }

  private ChassisSpeeds fieldToRobot(ChassisSpeeds fieldSpeeds) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getRotation());
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }

    if (Constants.tuningMode) {
      tuning();
    }
    xLimiter.setAccelerationLimit(maxAcceleration);
    yLimiter.setAccelerationLimit(maxAcceleration);
    hLimiter.setAccelerationLimit(maxAngularAcceleration);
    PeriodicTimer.stop(getName());
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public String getName() {
    return DrivetrainConstants.NAME;
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  private static LinearVelocity clampLinear(LinearVelocity v) {
    double mps =
        Math.max(0.0, Math.min(v.in(MetersPerSecond), DrivetrainConstants.MAX_SPEED));
    return MetersPerSecond.of(mps);
  }

  private static AngularVelocity clampAngular(AngularVelocity v) {
    double radps =
        Math.max(0.0, Math.min(v.in(RadiansPerSecond), DrivetrainConstants.MAX_ANGULAR_SPEED));
    return RadiansPerSecond.of(radps);
  }

  /** Limite de velocidade imposto pelo estado do robo (dono: SuperStructure). */
  public void setMaxSpeed(LinearVelocity maxSpeed) {
    this.stateMaxSpeed = clampLinear(maxSpeed);
  }

  /** Limite de velocidade imposto pelo piloto (ex.: modo lento no gatilho). */
  public void setPilotMaxSpeed(LinearVelocity maxSpeed) {
    this.pilotMaxSpeed = clampLinear(maxSpeed);
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/MaxSpeed")
  public LinearVelocity getMaxSpeed() {
    return stateMaxSpeed.in(MetersPerSecond) <= pilotMaxSpeed.in(MetersPerSecond)
        ? stateMaxSpeed
        : pilotMaxSpeed;
  }

  /** Limite angular imposto pelo estado do robo (dono: SuperStructure). */
  public void setMaxAngularSpeed(AngularVelocity maxAngularSpeed) {
    this.stateMaxAngularSpeed = clampAngular(maxAngularSpeed);
  }

  /** Limite angular imposto pelo piloto (ex.: modo lento no gatilho). */
  public void setPilotMaxAngularSpeed(AngularVelocity maxAngularSpeed) {
    this.pilotMaxAngularSpeed = clampAngular(maxAngularSpeed);
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/MaxAngularSpeed")
  public AngularVelocity getMaxAngularSpeed() {
    return stateMaxAngularSpeed.in(RadiansPerSecond) <= pilotMaxAngularSpeed.in(RadiansPerSecond)
        ? stateMaxAngularSpeed
        : pilotMaxAngularSpeed;
  }

  public void setMaxAcceleration(LinearAcceleration maxAcceleration) {
    if (maxAcceleration.in(MetersPerSecondPerSecond) >= DrivetrainConstants.MAX_ACCELERATION) {
      this.maxAcceleration = MetersPerSecondPerSecond.of(DrivetrainConstants.MAX_ACCELERATION);
    } else {
      this.maxAcceleration = maxAcceleration;
    }
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/MaxAcceleration")
  public LinearAcceleration getMaxAcceleration() {
    return this.maxAcceleration;
  }

  public void setMaxAngularAcceleration(AngularAcceleration maxAngularAcceleration) {
    if (maxAngularAcceleration.in(RadiansPerSecondPerSecond)
        >= DrivetrainConstants.MAX_ANGULAR_ACCELERATION) {
      this.maxAngularAcceleration =
          RadiansPerSecondPerSecond.of(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);
    } else {
      this.maxAngularAcceleration = maxAngularAcceleration;
    }
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/MaxAngularAcceleration")
  public AngularAcceleration getMaxAngularAcceleration() {
    return this.maxAngularAcceleration;
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/Odometry/Pose")
  public Pose2d getPose() {
    return getState().Pose;
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/Odometry/Rotation")
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/Odometry/Velocity")
  public ChassisSpeeds getRobotVelocity() {
    return getState().Speeds;
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/RobotTotalVelocity")
  public LinearVelocity getLinearVelocity() {
    var velocity = getRobotVelocity();
    double vx = velocity.vxMetersPerSecond;
    double vy = velocity.vyMetersPerSecond;

    double finalVelocity = Math.hypot(vx, vy);

    return MetersPerSecond.of(finalVelocity);
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/IsMoving")
  public boolean IsMoving() {
    // Deadband em vez de comparacao exata com zero: com ruido de odometria a velocidade
    // raramente e exatamente 0.0, o que tornava este predicado quase sempre verdadeiro.
    return getLinearVelocity().in(MetersPerSecond) > DrivetrainConstants.MOVING_DEADBAND_MPS;
  }

  public boolean isAtTrench() {
    Translation2d robot = getPose().getTranslation();
    return frc.game.FieldConstants.Zones.TRENCH_LEFT_BLUE.contains(robot)
        || frc.game.FieldConstants.Zones.TRENCH_LEFT_RED.contains(robot)
        || frc.game.FieldConstants.Zones.TRENCH_RIGHT_BLUE.contains(robot)
        || frc.game.FieldConstants.Zones.TRENCH_RIGHT_RED.contains(robot);
  }

  public boolean isAtBump() {
    Translation2d robot = getPose().getTranslation();
    return frc.game.FieldConstants.Zones.BUMP_LEFT_BLUE.contains(robot)
        || frc.game.FieldConstants.Zones.BUMP_LEFT_RED.contains(robot)
        || frc.game.FieldConstants.Zones.BUMP_RIGHT_BLUE.contains(robot)
        || frc.game.FieldConstants.Zones.BUMP_RIGHT_RED.contains(robot);
  }

  @AutoLogOutput(key = "Subsystems/Drivetrain/Odometry/Current Field General Zones")
  public Zones getCurrentGeneralZone() {
    Translation2d robot = getPose().getTranslation();
    if (frc.game.FieldConstants.Zones.ALLIANCE_BLUE_ZONE.contains(robot)) {
      return Zones.ALLIANCE_BLUE_ZONE;
    } else if (frc.game.FieldConstants.Zones.ALLIANCE_RED_ZONE.contains(robot)) {
      return Zones.ALLIANCE_RED_ZONE;
    } else if (frc.game.FieldConstants.Zones.NEUTRAL_ZONE.contains(robot)) {
      return Zones.NEUTRAL_ZONE;
    }
    return Zones.NOT_ZONE;
  }

  public boolean isNeutralMidZone() {
    return frc.game.FieldConstants.Zones.NEUTRAL_MID_ZONE.contains(getPose().getTranslation());
  }

  public boolean isBlue() {
    return AllianceSelector.getInstance().getResolvedAlliance() == Alliance.Blue;
  }
}
