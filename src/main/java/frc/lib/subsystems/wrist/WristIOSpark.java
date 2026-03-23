package frc.lib.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Angle;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

public class WristIOSpark implements WristIO {

  private final MotorIO motor;

  public WristIOSpark() {
    MotorConfig config =
        new MotorConfig()
            .currentLimit(Amps.of(80))
            .brakeMode()
            .nominalVoltage(Volts.of(10.0))
            .conversionFactors(0.125, 1.0);

    motor = new MotorIOSparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless, config);

    motor.setOffset(Rotation.of(0));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();
  }

  @Override
  public void stop() {
    motor.stop();
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    motor.setPercentOutput(percentOutput);
  }

  @Override
  public void runPosition(Angle position) {
    motor.runPosition(position);
  }

  @Override
  public void resetEncoder() {
    motor.setOffset(Rotation.of(0.0));
  }
}
