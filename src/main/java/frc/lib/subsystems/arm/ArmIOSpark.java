package frc.lib.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Angle;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;
import frc.lib.logger.LoggedTunableNumber;

public class ArmIOSpark implements ArmIO {

  private final MotorIO motor;

  private LoggedTunableNumber kp = new LoggedTunableNumber("Subsystems/Arm/P", ArmConstants.ARM_kP);
  private LoggedTunableNumber ki = new LoggedTunableNumber("Subsystems/Arm/I", ArmConstants.ARM_kI);
  private LoggedTunableNumber kd = new LoggedTunableNumber("Subsystems/Arm/D", ArmConstants.ARM_kD);
  private LoggedTunableNumber kF = new LoggedTunableNumber("Subsystems/Arm/F", ArmConstants.ARM_kF);

  public ArmIOSpark() {
    MotorConfig config =
        new MotorConfig()
            .currentLimit(Amps.of(40))
            .inverted(true)
            .brakeMode()
            .nominalVoltage(Volts.of(10.0))
            .conversionFactors(0.014705000445246696, 1.0);

    motor = new MotorIOSparkMax(ArmConstants.motorID, MotorType.kBrushless, config);

    motor.configurePIDF(
        0, ArmConstants.ARM_kP, ArmConstants.ARM_kI, ArmConstants.ARM_kD, ArmConstants.ARM_kF);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();

    if (kp.hasChanged(hashCode())
        || ki.hasChanged(hashCode())
        || kd.hasChanged(hashCode())
        || kF.hasChanged(hashCode())) {
      motor.configurePIDF(0, kp.get(), ki.get(), kd.get(), kF.get());
    }
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
  public void setOffset(Angle offset) {
    motor.setOffset(offset);
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    motor.setPercentOutput(percentOutput);
  }

  @Override
  public void runPosition(Angle position) {
    motor.runPosition(position);
  }
}
