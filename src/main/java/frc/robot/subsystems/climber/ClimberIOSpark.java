package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Angle;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

public class ClimberIOSpark implements ClimberIO {

  private final MotorIO motor;

  public ClimberIOSpark() {
    MotorConfig config =
        new MotorConfig()
            .currentLimit(Amps.of(40))
            .inverted(true)
            .brakeMode()
            .nominalVoltage(Volts.of(10.0))
            .conversionFactors(0.014705000445246696, 1.0);

    motor = new MotorIOSparkMax(ClimberConstants.motorID, MotorType.kBrushless, config);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();
  }

  @Override
  public void stop() {
    motor.stop();
  }

  @Override
  public void configurePIDF(double kP, double kI, double kD, double kF) {
    motor.configurePIDF(0, kP, kI, kD, kF);
  }

  @Override
  public void configureMaxOutput(double maxOutput) {
    motor.setMaxOutputSlot(maxOutput, 0);
  }

  @Override
  public void setOffset(Angle offset) {
    motor.setOffset(offset);
  }

  @Override
  public void runPercentOutput(double percentOutput) {
    motor.setPercentOutput(percentOutput);
  }

  @Override
  public void runPosition(Angle position) {
    motor.runPosition(position);
  }
}
