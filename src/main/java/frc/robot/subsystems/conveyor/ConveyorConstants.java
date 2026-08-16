package frc.robot.subsystems.conveyor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import frc.lib.interfaces.motor.advanced.MotorConfig;

public class ConveyorConstants {
  public static final int MOTOR_ID = 20;

  public static final double POWER = 0.6;
  public static final double SLOW_POWER = 0.25;
  public static final double REVERSE_POWER = -0.5;
  public static final double SLOW_REVERSE_POWER = -0.1;
  public static final double WIGGLE_PERIOD = 0.5;
  public static final double WIGGLE_POWER = 1.0;

  public static final MotorConfig MOTOR_CONFIG =
      new MotorConfig()
          .currentLimit(Amps.of(32))
          .coastMode()
          .nominalVoltage(Volts.of(10))
          .inverted(false);

  public enum ConveyorRequest {
    RUN,
    RUN_SLOW,
    REVERSE,
    SLOW_REVERSE,
    STOP,
    WIGGLE
  }

  public enum ConveyorState {
    IDLE,
    RUNNING,
    RUNNING_SLOW,
    REVERSING,
    REVERSING_SLOW,
    WIGGLING
  }
}
