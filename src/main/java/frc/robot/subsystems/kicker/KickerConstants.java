package frc.robot.subsystems.kicker;

public class KickerConstants {
  public static final int ID = 17;

  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;

  public static final double KS = 0.17;
  public static final double KV = 0.01727;
  public static final double KA = 0.0;

  public static final double CURRENT_LIMIT = 40;
  public static final double VOLTAGE_COMPENSATION = 9;

  public static final double POWER = 0.7;
  public static final double REVERSE_POWER = -0.7;
  public static final double SLOW_REVERSE_POWER = -0.2;

  public static final double VELOCITY_TOLERANCE = 300;

  public static final int SENSOR_CHANNEL = 1;

  public static enum KickerIntention {
    SHOOT,
    REVERSE,
    SLOW_REVERSE,
    STOP,
    IDLE_SPIN,
    NON_INTENTION
  }
}
