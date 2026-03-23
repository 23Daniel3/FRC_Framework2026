package frc.robot.subsystems.conveyor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

public class ConveyorIOSpark implements ConveyorIO {

  private final MotorIO motor;

  public ConveyorIOSpark() {
    MotorConfig config =
        new MotorConfig()
            .currentLimit(Amps.of(ConveyorConstants.CURRENT_LIMIT))
            .coastMode()
            .nominalVoltage(Volts.of(ConveyorConstants.VOLTAGE_COMPENSATION))
            .inverted(false);

    motor = new MotorIOSparkMax(ConveyorConstants.MOTOR_ID, MotorType.kBrushless, config);
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
