package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class DrivetrainConstants {

  public static final String NAME = "Subsystems/Drivetrain";
  public static final Current kSlipCurrent = Amps.of(120.0);

  public static final double PROJECTILE_VELOCITY_MPS = 2;

  public static final double WHEEL_COF = 1.2;
  public static final Distance WHEEL_RADIUS = Inches.of(1.9015);

  public static final double ANTI_TIPPING_KP = 0.13;
  public static final double ANTI_TIPPING_KI = 0.0;
  public static final double ANTI_TIPPING_KD = 0.0;

  public static final double ANGLE_KP = 10.0;
  public static final double ANGLE_KI = 0.0;
  public static final double ANGLE_KD = 0.0;

  public static final double MAX_SPEED = 5.3;

  /** Abaixo desta velocidade linear (m/s) o robo e considerado parado (deadband anti-ruido). */
  public static final double MOVING_DEADBAND_MPS = 0.05;
  public static final double MAX_SPEED_LIMITED = 1.4;
  public static final double MAX_ACCELERATION = 6.0;

  public static final double MAX_ANGULAR_SPEED = Units.degreesToRadians(600);
  public static final double MAX_ANGULAR_SPEED_LIMITED = Units.degreesToRadians(100);
  public static final double MAX_ANGULAR_ACCELERATION = 5 * Math.PI;
  public static final double MAX_JERK = 10.0;

  public static final double PROFILED_PID_ANGLE_KP = 8.0;
  public static final double PROFILED_PID_ANGLE_KI = 0.0;
  public static final double PROFILED_PID_ANGLE_KD = 0.4;

  public static final double TIPPING_THRESHOLD = 3;

  // Gear ratios for SDS MK4i L3, adjust as necessary
  public static final double DRIVE_GEAR_RATIO = 6.122448979591837;
  public static final double TURN_GEAR_RATIO = 21.428571428571427;

  // Modules:
  // Front Left
  public static final Distance X_DISTANCE_FL = Inches.of(10.34375);
  public static final Distance Y_DISTANCE_FL = Inches.of(10.34375);

  // Front Right
  public static final Distance X_DISTANCE_FR = Inches.of(10.34375);
  public static final Distance Y_DISTANCE_FR = Inches.of(-10.34375);

  // Back Left
  public static final Distance X_DISTANCE_BL = Inches.of(-10.34375);
  public static final Distance Y_DISTANCE_BL = Inches.of(10.34375);

  // Back Right
  public static final Distance X_DISTANCE_BR = Inches.of(-10.34375);
  public static final Distance Y_DISTANCE_BR = Inches.of(-10.34375);

  /** An array of module translations. */
  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(DrivetrainConstants.X_DISTANCE_FL, DrivetrainConstants.Y_DISTANCE_FL),
        new Translation2d(DrivetrainConstants.X_DISTANCE_FR, DrivetrainConstants.Y_DISTANCE_FR),
        new Translation2d(DrivetrainConstants.X_DISTANCE_BL, DrivetrainConstants.Y_DISTANCE_BL),
        new Translation2d(DrivetrainConstants.X_DISTANCE_BR, DrivetrainConstants.Y_DISTANCE_BR)
      };

  public static enum Zones {
    NOT_ZONE,
    ALLIANCE_BLUE_ZONE,
    ALLIANCE_RED_ZONE,
    NEUTRAL_ZONE
  }
}
