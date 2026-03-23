package frc.lib.subsystems.funnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

public class FunnelIOSpark implements FunnelIO {

  private final MotorIO motor;

  public FunnelIOSpark() {
    MotorConfig config =
        new MotorConfig()
            .currentLimit(Amps.of(30))
            .brakeMode()
            .nominalVoltage(Volts.of(10.0))
            .conversionFactors(7.0 / 150.0, 1.0);

    motor = new MotorIOSparkMax(FunnelConstants.CONVEYOR_MOTOR_ID, MotorType.kBrushless, config);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    motor.runVelocity(velocity);
  }

  @Override
  public void stop() {
    motor.stop();
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    motor.setPercentOutput(percentOutput);
  }
}
