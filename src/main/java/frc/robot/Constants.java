package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true;
  public static final boolean periodicTimer = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Idle Mode for Motors. */
  public static enum IdleMode {
    COAST,
    BRAKE
  }

  public static enum GeneralIntention {
    COLLECT,
    SHOOT,
    COLLECT_SHOOTING,
    IDLE,
    CLOSED
  }

  /** Alliance pre-seted just for security if have no connection */
  public static final Alliance alliance = Alliance.Blue;

  /** Robot Mass. */
  public static final double ROBOT_MASS_KG = 50.0;

  /** The Controller Deadband. */
  public static final double CONTROLLER_DEADBAND = 0.1;

  public static final double ROBOT_MOI = 4.3613;

  public static final RobotConfig ROBOT_CONFIG =
      new RobotConfig(
          Constants.ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              DrivetrainConstants.WHEEL_RADIUS.in(Meters),
              DrivetrainConstants.MAX_SPEED,
              DrivetrainConstants.WHEEL_COF,
              DCMotor.getKrakenX60(1).withReduction(DrivetrainConstants.DRIVE_GEAR_RATIO),
              DrivetrainConstants.kSlipCurrent.in(Amps),
              1),
          DrivetrainConstants.MODULE_TRANSLATIONS);
}
