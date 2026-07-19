package frc.robot.subsystems.example;

import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorControllerNone;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.interfaces.subsystem.SubsystemIO;
import org.littletonrobotics.junction.AutoLog;

/**
 * Camada IO no padrao AdvantageKit: a interface define O QUE o subsistema le (inputs) e quais
 * atuadores controla; as implementacoes definem COMO (hardware real, simulacao, ou nada — para
 * replay de log).
 *
 * <p>Os `default` vazios sao a implementacao de "replay": com eles, `new ExampleIO() {}` e um IO
 * valido que nao toca hardware, e os inputs vem do log gravado.
 */
public interface ExampleIO extends SubsystemIO<ExampleIOInputsAutoLogged> {

  /** Tudo que o subsistema LE do hardware. @AutoLog gera a classe *AutoLogged usada no replay. */
  @AutoLog
  public static class ExampleIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public default void updateInputs(ExampleIOInputsAutoLogged inputs) {}

  /** Um MotorController por atuador. MotorControllerNone e um no-op seguro. */
  public default MotorController controlMotor() {
    return new MotorControllerNone();
  }
}
