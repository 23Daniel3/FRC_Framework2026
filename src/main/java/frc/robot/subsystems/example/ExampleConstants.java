package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import frc.lib.interfaces.motor.MotorConfig;

/**
 * TEMPLATE DE REFERENCIA — este subsistema NAO e instanciado no robo. Copie a pasta `example`,
 * renomeie (ou use `tools/new_subsystem.py`) e adapte para criar um mecanismo novo.
 *
 * <p>Convencao: TODAS as constantes do subsistema vivem aqui — IDs, potencias, tolerancias,
 * configuracao de motor e os DOIS enums do padrao Request → State:
 *
 * <ul>
 *   <li><b>Request</b>: o que o mundo externo (SuperStructure) pode PEDIR. E intencao, nao
 *       mecanica. Poucos valores, nomes de negocio (COLLECT, SHOOT...).
 *   <li><b>State</b>: como a FSM interna executa. Pode ter estados intermediarios (GOING_*,
 *       RAMPING...) que o mundo externo nunca precisa conhecer.
 * </ul>
 */
public class ExampleConstants {

  /** ID CAN do motor — confira no Phoenix Tuner / REV Hardware Client. */
  public static final int MOTOR_ID = 99;

  public static final double FORWARD_POWER = 0.5;

  /** Deadband anti-ruido: nunca compare velocidade de sensor com == 0. */
  public static final double STOPPED_RPM_TOLERANCE = 20.0;

  public static final MotorConfig MOTOR_CONFIG =
      new MotorConfig()
          .currentLimit(Amps.of(30))
          .brakeMode()
          .nominalVoltage(Volts.of(10))
          .inverted(false);

  /** Intencoes que a SuperStructure pode expressar para este subsistema. */
  public enum ExampleRequest {
    RUN,
    STOP
  }

  /** Estados internos da FSM. Note o par "settling/settled": STOPPING → STOPPED. */
  public enum ExampleState {
    RUNNING,
    STOPPING,
    STOPPED
  }
}
