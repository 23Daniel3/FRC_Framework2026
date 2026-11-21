package frc.robot.subsystems.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ciprix.interfaces.motor.MotorIOSim;
import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.sim.SimMechanism;

/** O roller simulado, com a mesma configuracao e o mesmo nome do robo: ganhos e logs batem. */
public class RollerIOSim implements RollerIO {
  private final MotorIOSim motor =
      MotorIOSim.talonFX(
          "Roller/Motor",
          RollerConstants.MOTOR_CONFIG,
          SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001).withFriction(0.05, 0.0));

  @Override
  public void updateInputs(RollerIOInputsAutoLogged inputs) {
    motor.updateInputs(inputs.motorInputs);
  }

  @Override
  public MotorController controlMotor() {
    return motor.getMotorController();
  }
}
