package frc.robot.subsystems.conveyor;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ciprix.interfaces.motor.MotorIOSim;
import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.sim.SimMechanism;

/**
 * Simulated conveyor: the same SPARK MAX configuration used on the robot, driving a light roller.
 * The motor has the same name as on the robot, so logs and tunable values match.
 */
public class ConveyorIOSim implements ConveyorIO {

  // What the mechanism is made of. Estimates: replace them with the CAD values when known.
  private static final double GEARING = 4.0;
  private static final double INERTIA_KG_M2 = 0.0005;
  private static final double FRICTION_NM = 0.05; // a free conveyor still draws about 0.7 A

  private final MotorIOSim motor =
      MotorIOSim.spark(
          "ConveyorMotor",
          ConveyorConstants.MOTOR_CONFIG,
          SimMechanism.roller(DCMotor.getNEO(1), GEARING, INERTIA_KG_M2)
              .withFriction(FRICTION_NM, 0.0));

  @Override
  public void updateInputs(ConveyorIOInputsAutoLogged inputs) {
    motor.updateInputs(inputs.motorInputs);
  }

  @Override
  public MotorController controlMotor() {
    return motor.getMotorController();
  }

  /** The simulated motor, for tests that inject a jam or a disconnection. */
  public MotorIOSim getMotor() {
    return motor;
  }
}
