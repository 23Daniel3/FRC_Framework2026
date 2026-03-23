package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkFlex;

public class KickerIOSparkFlex implements KickerIO {

  private final MotorIO motor;
  private final DigitalInput sensor = new DigitalInput(KickerConstants.SENSOR_CHANNEL);

  public KickerIOSparkFlex() {
    MotorConfig config =
        new MotorConfig().currentLimit(Amps.of(40)).coastMode().nominalVoltage(Volts.of(12.0));

    motor = new MotorIOSparkFlex(KickerConstants.ID, MotorType.kBrushless, config);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();
    inputs.isSensorActive = !sensor.get();
  }

  @Override
  public void runVelocity(AngularVelocity velocity, Voltage voltage) {
    motor.runVelocity(velocity, 0, voltage);
  }

  @Override
  public void runPercentOutput(double percentOutput) {
    motor.setPercentOutput(percentOutput);
  }

  @Override
  public void stop() {
    motor.stop();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    motor.configurePIDF(0, kP, kI, kD, 0);
  }

  @Override
  public void setCurrentLimit(Current current) {
    motor.setCurrentLimit(current);
  }

  @Override
  public void setVoltageCompensation(Voltage voltage) {
    motor.setVoltageCompensation(voltage);
  }
}
