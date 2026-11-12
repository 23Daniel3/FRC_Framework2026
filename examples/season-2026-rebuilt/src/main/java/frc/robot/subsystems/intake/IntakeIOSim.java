package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ciprix.interfaces.motor.MotorIOSim;
import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.sim.SimMechanism;

/**
 * Simulated intake: the roller (Kraken X60 on a Talon FX) and the pivot (NEO on a SPARK MAX), with
 * the same configurations and names used on the robot, so the gains and the logs match.
 */
public class IntakeIOSim implements IntakeIO {

  // What the mechanisms are made of. Estimates: replace them with the CAD values when known.
  private static final double ROLLER_GEARING = 3.0;
  private static final double ROLLER_INERTIA_KG_M2 = 0.001;
  private static final double PIVOT_GEARING = 12.0;
  private static final double PIVOT_INERTIA_KG_M2 = 0.02;

  private final MotorIOSim rollerMotor =
      MotorIOSim.talonFX(
          "Intake/Roller",
          IntakeConstants.CONFIG_ROLLER_MOTOR,
          SimMechanism.roller(DCMotor.getKrakenX60(1), ROLLER_GEARING, ROLLER_INERTIA_KG_M2));

  private final MotorIOSim intakeMotor =
      MotorIOSim.spark(
          "Intake/Intake",
          IntakeConstants.CONFIG_INTAKE_MOTOR,
          SimMechanism.roller(DCMotor.getNEO(1), PIVOT_GEARING, PIVOT_INERTIA_KG_M2));

  // Simulates the coast button of the robot (pressed = true).
  private boolean simulatedCoastButton = false;

  @Override
  public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    rollerMotor.updateInputs(inputs.rollerMotorInputs);
    intakeMotor.updateInputs(inputs.intakeMotorInputs);
    inputs.coastButtonPressed = simulatedCoastButton;
  }

  @Override
  public MotorController controlIntakeMotor() {
    return intakeMotor.getMotorController();
  }

  @Override
  public MotorController controlRollerMotor() {
    return rollerMotor.getMotorController();
  }

  /** Presses or releases the simulated coast button, to see how the state machine reacts. */
  public void setSimulatedCoastButton(boolean pressed) {
    this.simulatedCoastButton = pressed;
  }

  /** The simulated roller, for tests that inject a jam or a disconnection. */
  public MotorIOSim getRollerMotor() {
    return rollerMotor;
  }
}
