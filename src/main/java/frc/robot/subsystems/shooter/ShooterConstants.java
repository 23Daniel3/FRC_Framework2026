package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import frc.lib.interfaces.motor.MotorConfig;

public class ShooterConstants {

  public static final double KICKER_POWER = 0.7;
  public static final double KICKER_REVERSE_POWER = -0.7;
  public static final double KICKER_SLOW_REVERSE_POWER = -0.2;

  public static final int LEADER_ID = 15;
  public static final int FOLLOWER_ID = 16;
  public static final int KICKER_MOTOR_ID = 17;

  public static final double REVERSE_POWER = -0.4;

  public static final double START_KICKER_TOLERANCE = 900;

  public static final double IDLE_SPIN_VELOCITY = 1800;

  public static final MotorConfig KICKER_MOTOR_CONFIG =
      new MotorConfig()
          .currentLimit(Amps.of(40))
          .coastMode()
          .nominalVoltage(Volts.of(10.0))
          .svag(0, 0.17, 0.01727, 0, 0);

  public static final MotorConfig MOTOR_LEADER_CONFIG =
      new MotorConfig()
          .currentLimit(Amps.of(40))
          .coastMode()
          .nominalVoltage(Volts.of(10))
          .inverted(true)
          .pid(0, 0, 0, 0.013)
          .svag(0, 0.24, 0.1157, 0.0, 0.0)
          .withVelocityTolerance(RPM.of(100));

  public static final MotorConfig MOTOR_FOLLOWER_CONFIG =
      new MotorConfig()
          .currentLimit(Amps.of(40))
          .coastMode()
          .nominalVoltage(Volts.of(10))
          .withMotorLeader(LEADER_ID)
          .withFollowerInverted(true)
          .withVelocityTolerance(RPM.of(300));

  public enum ShooterRequest {
    STOP,
    SHOOT,
    REVERSE
  }

  public enum ShooterState {
    IDLE,
    FLYWHEEL_RAMPING,
    KICKER_RAMPING,
    SHOOTING,
    REVERSING
  }
}
