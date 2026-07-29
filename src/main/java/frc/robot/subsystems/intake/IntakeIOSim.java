package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIOSim;

public class IntakeIOSim implements IntakeIO {
  private final MotorIOSim rollerMotor;
  private final MotorIOSim intakeMotor;

  // Variável para simular o botão físico/sensor no ambiente de software
  private boolean simulatedCoastButton = false;

  public IntakeIOSim() {
    /**
     * Simulação do Motor do Rolo (Roller) - Motor: Kraken X60 (ou o que estiver usando) - Redução:
     * 2:1 (exemplo) - Inércia: 0.001 (muito leve, apenas um rolo)
     */
    rollerMotor =
        new MotorIOSim(
            "Intake/Roller",
            IntakeConstants.CONFIG_ROLLER_MOTOR,
            DCMotor.getKrakenX60(1),
            3.0,
            0.001);

    /**
     * Simulação do Motor do Pivot (Intake) - Motor: Neo Vortex / SparkFlex - Redução: 50:1 (Pivot
     * costuma ter redução alta) - Inércia: 0.02 (Carga moderada para simular o peso do braço)
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
    // Atualiza a física e os logs dos motores simulados
    rollerMotor.updateInputs(inputs.rollerMotorInputs);
    intakeMotor.updateInputs(inputs.intakeMotorInputs);

    // No simulador, o botão de coast geralmente fica falso,
    // a menos que você queira simular um clique via GUI
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
   * Método utilitário para testes: permite que você force o estado do sensor durante a simulação
   * para ver como a sua FSM reage.
   */
  public void setSimulatedCoastButton(boolean pressed) {
    this.simulatedCoastButton = pressed;
  }
}
