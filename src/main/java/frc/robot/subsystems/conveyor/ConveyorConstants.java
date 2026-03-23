package frc.robot.subsystems.conveyor;

public class ConveyorConstants {
  public static final int MOTOR_ID = 20;

  public static final double CURRENT_LIMIT = 32;
  public static final double VOLTAGE_COMPENSATION = 10;

  public static final double POWER = 0.6;
  public static final double SLOW_POWER = 0.25;
  public static final double REVERSE_POWER = -0.5;
  public static final double SLOW_REVERSE_POWER = -0.1;

  public enum ConveyorIntention {
    RUN,
    RUN_SLOW,
    REVERSE,
    SLOW_REVERSE,
    STOP,
    WIGGLE,
    NON_INTENTION
  }
}
