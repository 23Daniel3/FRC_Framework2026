package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int ROLLER_MOTOR_ID = 18;
  public static final int INTAKE_MOTOR_ID = 19;

  public static final int SENSOR_PORT = 0;

  public static final double CURRENT_LIMIT_INTAKE_MOTOR = 35;
  public static final double CURRENT_LIMIT_ROLLER_MOTOR = 30;
  public static final double VOLTAGE_COMPENSATION_INTAKE_MOTOR = 10;
  public static final double NOMINAL_VOLTAGE_ROLLER_MOTOR = 10;

  public static final double ROLLER_KP = 0.0;
  public static final double ROLLER_KI = 0.0;
  public static final double ROLLER_KD = 0.0;
  public static final double ROLLER_KS = 0.42;
  public static final double ROLLER_KV = 0.115;

  public static final double INTAKE_KP = 0.08;
  public static final double INTAKE_KI = 0.0;
  public static final double INTAKE_KD = 0.0;
  public static final double INTAKE_KF = 0.0;

  public static final double INTAKE_START_POSITION = 0.0;
  public static final double INTAKE_IN_POSITION = 1.0;
  public static final double INTAKE_MIDDLE_POSITION = 8.2;
  public static final double INTAKE_OUT_POSITION = 10.7;

  public static final double INTAKE_POWER = 0.85;
  public static final double INTAKE_MAX_VELOCITY = 5500;

  public static final double INTAKE_REVERSE_POWER = -0.45;

  public enum IntakeIntention {
    IN,
    OUT,
    MIDDLE,
    NON_INTENTION
  }

  public enum RollerIntention {
    INTAKE,
    OUTAKE,
    STOP,
    NON_INTENTION
  }
}
