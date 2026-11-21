package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation3d;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.power.PowerPriority;

public class IntakeConstants {
  public static final int ROLLER_MOTOR_ID = 18;
  public static final int INTAKE_MOTOR_ID = 19;

  public static final int SENSOR_PORT = 0;

  public static final double INTAKE_START_POSITION = 0.0;
  public static final double INTAKE_IN_POSITION = 1.0;
  public static final double INTAKE_MIDDLE_POSITION = 8.2;
  public static final double INTAKE_OUT_POSITION = 10.7;

  public static final double INTAKE_POWER = 0.85;
  public static final double INTAKE_MAX_VELOCITY = 2500;

  public static final double INTAKE_REVERSE_POWER = -0.45;

  /**
   * Eixo do pivot do intake no robo (metros: X para frente, Y para a esquerda, Z para cima, a
   * partir do centro do chassi no chao). Usado so na visualizacao 3D. Valor provisorio: medir no
   * CAD.
   */
  public static final Translation3d PIVOT_AXIS = new Translation3d(0.30, 0.0, 0.18);

  /**
   * Quanto o pivot gira da posicao inicial (a desenhada no CAD) ate OUT, em graus. Negativo abaixa
   * a frente, que e o intake abrindo para fora. Usado so na visualizacao 3D. Valor provisorio:
   * medir no CAD.
   */
  public static final double PIVOT_DEPLOY_DEGREES = -100.0;

  /** Abaixo deste modulo de RPM o roller e considerado parado (deadband anti-ruido). */
  public static final double STOPPED_RPM_TOLERANCE = 20.0;

  public static final MotorConfig CONFIG_ROLLER_MOTOR =
      new MotorConfig()
          .currentLimit(Amps.of(35))
          .coastMode()
          .nominalVoltage(Volts.of(10))
          .inverted(true)
          .svag(0, 0.42, 0.115, 0, 0)
          .powerPriority(PowerPriority.LOW);

  public static final MotorConfig CONFIG_INTAKE_MOTOR =
      new MotorConfig()
          .currentLimit(Amps.of(25))
          .brakeMode()
          .nominalVoltage(Volts.of(10.0))
          .inverted(false)
          .pid(0, 0.08, 0, 0)
          .withPositionTolerance(Rotations.of(1));

  public enum IntakeRequest {
    IN,
    OUT,
    COLLECT,
    STOP,
    REVERSE
  }

  public enum IntakeState {
    IN,
    OUT,
    STOPPED,
    STOPPING,
    COLLECT,
    GOING_IN,
    GOING_OUT,
    REVERSING
  }
}
