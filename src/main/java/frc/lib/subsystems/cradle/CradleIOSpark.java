package frc.lib.subsystems.cradle;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

public class CradleIOSpark implements CradleIO {

  private final MotorIO motorLeft;
  private final MotorIO motorRight;
  private final DigitalInput frontSensor;

  public CradleIOSpark() {
    MotorConfig baseConfig =
        new MotorConfig().currentLimit(Amps.of(30)).brakeMode().nominalVoltage(Volts.of(10.0));

    motorLeft =
        new MotorIOSparkMax(
            CradleConstants.MOTOR_LEFT_ID, MotorType.kBrushless, baseConfig.inverted(false));

    motorRight =
        new MotorIOSparkMax(
            CradleConstants.MOTOR_RIGHT_ID, MotorType.kBrushless, baseConfig.inverted(true));

    frontSensor = new DigitalInput(CradleConstants.FRONT_SENSOR_PORT);
  }

  @Override
  public void updateInputs(CradleIOInputs inputs) {
    inputs.motorLeftInputs = motorLeft.getMotorIOInputs();
    inputs.motorRightInputs = motorRight.getMotorIOInputs();
    inputs.sensorIsTrue = !frontSensor.get();
  }

  @Override
  public void setVoltage(Voltage volts) {
    motorLeft.setVoltage(volts);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    motorLeft.runVelocity(velocity);
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    if (percentOutput > 0.7) percentOutput = 0.7;
    motorLeft.setPercentOutput(percentOutput);
    motorRight.setPercentOutput(percentOutput);
  }

  @Override
  public void setInvertPercentOutput(double percentOutput) {
    motorLeft.setPercentOutput(-percentOutput);
    motorRight.setPercentOutput(-percentOutput);
  }

  @Override
  public void setStop() {
    motorLeft.stop();
    motorRight.stop();
  }
}
