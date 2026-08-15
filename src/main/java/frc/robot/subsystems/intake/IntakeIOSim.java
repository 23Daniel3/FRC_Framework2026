package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.interfaces.motor.MotorIOSim;
import frc.lib.interfaces.motor.advanced.MotorController;

public class IntakeIOSim implements IntakeIO {
  private final MotorIOSim rollerMotor;
  private final MotorIOSim intakeMotor;

  // Variable to simulate the physical button/sensor in the software environment
  private boolean simulatedCoastButton = false;

  public IntakeIOSim() {
    /**
     * Roller Motor Simulation - Motor: Kraken X60 (or whichever is being used) - Reduction: 2:1
     * (example) - Inertia: 0.001 (very light, just a roller)
     */
    rollerMotor =
        new MotorIOSim(
            "Intake/Roller",
            IntakeConstants.CONFIG_ROLLER_MOTOR,
            DCMotor.getKrakenX60(1),
            3.0,
            0.001);

    /**
     * Pivot Motor Simulation (Intake) - Motor: Neo Vortex / SparkFlex - Reduction: 50:1 (Pivot
     * usually has high reduction) - Inertia: 0.02 (Moderate load to simulate arm weight)
     */
    intakeMotor =
        new MotorIOSim(
            "Intake/Pivot",
            IntakeConstants.CONFIG_INTAKE_MOTOR,
            DCMotor.getNeoVortex(1),
            12.0,
            0.02);
  }

  @Override
  public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    // Updates physics and logs for the simulated motors
    rollerMotor.updateInputs(inputs.rollerMotorInputs);
    intakeMotor.updateInputs(inputs.intakeMotorInputs);

    // In simulation, the coast button is usually false,
    // unless you want to simulate a click via GUI
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

  /**
   * Utility method for testing: allows forcing the sensor state during simulation to see how your
   * FSM reacts.
   */
  public void setSimulatedCoastButton(boolean pressed) {
    this.simulatedCoastButton = pressed;
  }
}
