package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

public class ClimberIOSpark implements ClimberIO {

  private final MotorIO motor;

  public ClimberIOSpark() {
    motor =
        new MotorIOSparkMax(
            "ClimberMotor",
            ClimberConstants.MOTOR_ID,
            MotorType.kBrushless,
            ClimberConstants.MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();
  }

  @Override
  public MotorController controlMotor() {
    return motor.getMotorController();
  }
}
