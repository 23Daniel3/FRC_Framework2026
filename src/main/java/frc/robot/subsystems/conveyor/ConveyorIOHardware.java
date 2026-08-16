package frc.robot.subsystems.conveyor;

import frc.lib.interfaces.motor.MotorIOSparkMax;
import frc.lib.interfaces.motor.advanced.MotorController;
import frc.lib.interfaces.motor.advanced.MotorIO;

public class ConveyorIOHardware implements ConveyorIO {

  private final MotorIO motor;

  public ConveyorIOHardware() {
    motor =
        new MotorIOSparkMax(
            "ConveyorMotor", ConveyorConstants.MOTOR_ID, ConveyorConstants.MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(ConveyorIOInputsAutoLogged inputs) {
    motor.updateInputs(inputs.motorInputs);
  }

  @Override
  public MotorController controlMotor() {
    return motor.getMotorController();
  }
}
