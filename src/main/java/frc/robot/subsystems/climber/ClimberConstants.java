package frc.robot.subsystems.climber;

public class ClimberConstants {
  public static final int motorID = 21;
  public static final double MAX_POSITION = 100.0;
  public static final double MIN_POSITION = -100.0;
  public static final double kP = 50.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kF = 0.0;

  public static final double HIGH_POSITION = 1.19;
  public static final double LOW_POSITION = -0.2;

  public static final double POWER_UP = 0.8;
  public static final double POWER_DOWN = -0.8;

  public static final double MAX_SECURITY_CURRENT = 45;

  public static final double THRESOLD_CURRENT_SPIKE = 0.5;

  public static enum ClimberIntention {
    HIGH,
    LOW,
    INIT,
    POWER_UP,
    POWER_DOWN,
    NON_INTENTION
  }
}
