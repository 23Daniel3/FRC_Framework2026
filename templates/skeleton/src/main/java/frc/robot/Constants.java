package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/** Constantes do robo inteiro. As de cada subsistema ficam em {@code <Subsistema>Constants}. */
public final class Constants {
  private Constants() {}

  /** O que roda fora do robo: SIM (simulacao com fisica) ou REPLAY (roda um log de novo). */
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** Valores ajustaveis pelo dashboard. Desligue nas competicoes. */
  public static final boolean tuningMode = true;

  public static final double CONTROLLER_DEADBAND = 0.08;

  /** Alianca usada quando nao ha FMS nem Driver Station. */
  public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

  /** Campo de 2026 (REBUILT, versao soldada), em metros. */
  public static final double FIELD_LENGTH_METERS = 16.541;

  public static final double FIELD_WIDTH_METERS = 8.069;

  public enum Mode {
    /** No robo de verdade. */
    REAL,
    /** Simulacao com fisica. */
    SIM,
    /** Rodando um log de novo. */
    REPLAY
  }

  /** O que o piloto pede ao robo. Um botao (ou combinacao) por request. */
  public enum RobotRequest {
    IDLE,
    INTAKE,
    EJECT
  }

  /** Estados da maquina de estados geral (a SuperStructure). */
  public enum RobotState {
    IDLE,
    INTAKING,
    EJECTING
  }
}
