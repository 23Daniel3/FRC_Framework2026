package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public static final int LEADER_ID = 15;
  public static final int FOLLOWER_ID = 16;

  public static final double CURRENT_LIMIT = 40;
  public static final double NOMINAL_VOLTAGE = 10;

  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.013;

  public static final double KS = 0.24;
  public static final double KV = 0.1157;
  public static final double KA = 0.0;

  public static final double REVERSE_POWER = -0.4;

  public static final double SHOOT_TOLERANCE = 100;
  public static final double START_KICKER_TOLERANCE = 900;

  public static final double IDLE_SPIN_VELOCITY = 1800;

  public static enum FlywheelIntention {
    SHOOT,
    IDLE_SPIN,
    REVERSE,
    STOP,
    NON_INTENTION
  }
}
