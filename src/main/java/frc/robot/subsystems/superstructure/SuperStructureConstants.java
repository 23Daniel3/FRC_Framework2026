package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class SuperStructureConstants {

  public static final double CONVEYOR_WARNING_CURRENT = 38;
  public static final double ROLLER_WARNING_CURRENT = 38;

  public static final double TIMEOUT_TO_SHOOT_ON_KICKER_ACTIVE = 2;

  public static final double MAX_ERROR_ANGLE_DEG_SHOOT = 3;
  public static final double MAX_ERROR_ANGLE_DEG_NEUTRAL = 7;

  public static final double BALL_EXITING_ANGLE_DEG = 60;
  public static final double DIAMETER_WHEEL_UP_METERS = 0.05715;
  public static final double DIAMETER_WHEEL_DOWN_METERS = 0.01016;
  public static final double SHOOTER_OFFSET_METERS = 0.254;

  public static final double CONVEYOR_REVERSE_TIME = 1;
  public static final double CONVEYOR_MIN_VELOCITY = 20;
  public static final double MAX_TIME_LOCKED = 0.25;

  public static final double SENSOR_DELAY_SECONDS = 2.0;

  public static final double MAX_VELOCITY_TO_CLOSE_INTAKE = 4;

  public static final LinearVelocity MAX_VELOCITY_TO_SHOOT = MetersPerSecond.of(1);
}
