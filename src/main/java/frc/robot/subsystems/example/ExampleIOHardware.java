package frc.robot.subsystems.example;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

/**
 * Implementacao de hardware real. Trocar de fornecedor = trocar MotorIOSparkMax por MotorIOTalonFX
 * (ou MotorIOSparkFlex) — o resto do subsistema nao muda.
 */
public class ExampleIOHardware implements ExampleIO {

  private final MotorIO motor;

  public ExampleIOHardware() {
    motor =
        new MotorIOSparkMax(
            "ExampleMotor",
            ExampleConstants.MOTOR_ID,
            MotorType.kBrushless,
            ExampleConstants.MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(ExampleIOInputsAutoLogged inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();
  }

  @Override
  public MotorController controlMotor() {
    return motor.getMotorController();
  }
}
