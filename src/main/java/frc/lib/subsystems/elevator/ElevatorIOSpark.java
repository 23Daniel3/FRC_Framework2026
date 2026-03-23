package frc.lib.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

public class ElevatorIOSpark implements ElevatorIO {

  private final MotorIO motorLeft;
  private final MotorIO motorRight;

  public ElevatorIOSpark() {
    MotorConfig leftConfig =
        new MotorConfig()
            .currentLimit(Amps.of(30))
            .brakeMode()
            .nominalVoltage(Volts.of(10.0))
            .softLimits(Rotations.of(-ElevatorConstants.ELEVATOR_MAX_POSITION), Rotations.of(0.0))
            .inverted(false);

    MotorConfig rightConfig =
        new MotorConfig()
            .currentLimit(Amps.of(30))
            .brakeMode()
            .nominalVoltage(Volts.of(10.0))
            .softLimits(Rotations.of(-ElevatorConstants.ELEVATOR_MAX_POSITION), Rotations.of(0.0))
            .inverted(false);

    motorLeft =
        new MotorIOSparkMax(ElevatorConstants.MOTOR_LEFT_ID, MotorType.kBrushless, leftConfig);
    motorRight =
        new MotorIOSparkMax(ElevatorConstants.MOTOR_RIGHT_ID, MotorType.kBrushless, rightConfig);

    motorLeft.configurePIDF(0, 0.2, 0, 0, 0);
    motorRight.configurePIDF(0, 0.2, 0, 0, 0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorLeftInputs = motorLeft.getMotorIOInputs();
    inputs.motorRightInputs = motorRight.getMotorIOInputs();
  }

  @Override
  public void setVoltage(Voltage volts) {
    motorLeft.setVoltage(volts);
    motorRight.setVoltage(volts);
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    motorLeft.setPercentOutput(percentOutput);
    motorRight.setPercentOutput(percentOutput);
  }

  @Override
  public void stop() {
    motorLeft.stop();
    motorRight.stop();
  }

  @Override
  public void runPosition(Angle position) {
    motorLeft.runPosition(position);
    motorRight.runPosition(position);
  }

  @Override
  public void reset() {
    motorLeft.setOffset(Rotations.of(0.0));
    motorRight.setOffset(Rotations.of(0.0));
  }
}
