package frc.robot.subsystems.conveyor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkFlex;

public class ConveyorIOSparkFlex implements ConveyorIO {

  private final MotorIO motor;

  public ConveyorIOSparkFlex() {
    MotorConfig config =
        new MotorConfig()
            .currentLimit(Amps.of(40))
            .coastMode()
            .nominalVoltage(Volts.of(12.0))
            .inverted(false);

    motor = new MotorIOSparkFlex(ConveyorConstants.MOTOR_ID, MotorType.kBrushless, config);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();
  }

  @Override
  public void runPercentOutput(double percentOutput) {
    motor.setPercentOutput(percentOutput);
  }

  @Override
  public void setCurrentLimit(Current current) {
    motor.setCurrentLimit(current);
  }

  @Override
  public void setVoltageCompensation(Voltage voltage) {
    motor.setVoltageCompensation(voltage);
  }

  @Override
  public void stop() {
    motor.stop();
  }
}
