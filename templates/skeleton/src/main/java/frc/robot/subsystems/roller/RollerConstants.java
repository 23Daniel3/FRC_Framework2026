package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Amps;

import org.ciprix.interfaces.motor.advanced.MotorConfig;

/** Constantes do roller: ids, potencias, configuracao do motor, requests e estados. */
public final class RollerConstants {
  private RollerConstants() {}

  public static final int MOTOR_ID = 10;

  public static final double INTAKE_OUTPUT = 0.6;
  public static final double EJECT_OUTPUT = -0.5;

  public static final MotorConfig MOTOR_CONFIG =
      new MotorConfig().currentLimit(Amps.of(40)).coastMode();

  /** O que a SuperStructure (ou o copiloto) pede ao roller. */
  public enum RollerRequest {
    STOP,
    INTAKE,
    EJECT
  }

  /** Estados da maquina de estados do roller. */
  public enum RollerState {
    STOPPED,
    INTAKING,
    EJECTING
  }
}
