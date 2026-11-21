package frc.robot.subsystems.roller;

import com.ctre.phoenix6.CANBus;
import org.ciprix.interfaces.motor.MotorIOTalonFX;
import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.interfaces.motor.advanced.MotorIO;

/** O roller no robo: um Kraken X60 (Talon FX) no barramento CAN do roboRIO. */
public class RollerIOTalonFX implements RollerIO {
  private final MotorIO motor =
      new MotorIOTalonFX(
          "Roller/Motor", RollerConstants.MOTOR_ID, new CANBus(), RollerConstants.MOTOR_CONFIG);

  @Override
  public void updateInputs(RollerIOInputsAutoLogged inputs) {
    motor.updateInputs(inputs.motorInputs);
  }

  @Override
  public MotorController controlMotor() {
    return motor.getMotorController();
  }
}
