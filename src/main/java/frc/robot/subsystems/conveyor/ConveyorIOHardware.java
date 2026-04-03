package frc.robot.subsystems.conveyor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;

public class ConveyorIOHardware implements ConveyorIO {

  private final MotorIO motor;

  public ConveyorIOHardware() {
    motor =
        new MotorIOSparkMax(
            "ConveyorMotor",
            ConveyorConstants.MOTOR_ID,
            MotorType.kBrushless,
            ConveyorConstants.MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(ConveyorIOInputsAutoLogged inputs) {
    inputs.motorInputs = motor.getMotorIOInputs();
  }

  @Override
  public MotorController controlMotor() {
    return motor.getMotorController();
  }
}
