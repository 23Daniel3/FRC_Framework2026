package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import frc.lib.interfaces.motor.MotorConfig;

public class IntakeConstants {
  public static final int ROLLER_MOTOR_ID = 18;
  public static final int INTAKE_MOTOR_ID = 19;

  public static final int SENSOR_PORT = 0;

  public static final double INTAKE_START_POSITION = 0.0;
  public static final double INTAKE_IN_POSITION = 1.0;
  public static final double INTAKE_MIDDLE_POSITION = 8.2;
  public static final double INTAKE_OUT_POSITION = 10.7;

  public static final double INTAKE_POWER = 0.85;
  public static final double INTAKE_MAX_VELOCITY = 2500;

  public static final double INTAKE_REVERSE_POWER = -0.45;

  public static final MotorConfig CONFIG_ROLLER_MOTOR =
      new MotorConfig()
          .currentLimit(Amps.of(30))
          .coastMode()
          .nominalVoltage(Volts.of(10))
          .inverted(true)
          .svag(0, 0.42, 0.115, 0, 0);

  public static final MotorConfig CONFIG_INTAKE_MOTOR =
      new MotorConfig()
          .currentLimit(Amps.of(35))
          .brakeMode()
          .nominalVoltage(Volts.of(10.0))
          .inverted(false)
          .pid(0, 0.08, 0, 0)
          .withPositionTolerance(Rotations.of(1));

  public enum IntakeRequest {
    IN,
    OUT,
    COLLECT,
    STOP,
  }

  public enum IntakeState {
    IN,
    OUT,
    STOPPED,
    STOPING,
    COLLECT,
    GOING_IN,
    GOING_OUT
  }
}
