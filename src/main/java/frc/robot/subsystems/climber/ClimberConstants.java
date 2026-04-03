package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import frc.lib.interfaces.motor.MotorConfig;

public class ClimberConstants {
  public static final int MOTOR_ID = 21;

  public static final Angle HIGH_POSITION = Rotations.of(1.19);
  public static final Angle LOW_POSITION = Rotations.of(-0.2);

  public static final MotorConfig MOTOR_CONFIG =
      new MotorConfig()
          .currentLimit(Amps.of(40))
          .inverted(true)
          .brakeMode()
          .nominalVoltage(Volts.of(10.0))
          .conversionFactors(0.014705000445246696, 1.0)
          .pid(0, 50, 0, 0)
          .outputRange(-0.7, 0.7);

  public enum ClimberRequest {
    LOW,
    HIGH
  }

  public enum ClimberState {
    RETRACTED,
    EXTENDED,
    RETRACTING,
    EXTENDING,
  }
}
