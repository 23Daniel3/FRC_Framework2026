package frc.robot.commands;

public final class CommandConstants {

  /** Constantes dos autonomos (tempos minimos/maximos das fases de mira e tiro). */
  public static final class AutoConstants {
    /** Tempo minimo (s) mirando+atirando no inicio dos autos de ciclo. */
    public static final double AIM_SHOOT_MIN_TIME_SEC = 5.0;

    /** Tempo minimo (s) da fase de mira+tiro do auto "so mirar e atirar". */
    public static final double AIM_ONLY_SHOOT_TIME_SEC = 10.0;
  }

  public static final class MoveXConstants {
    public static final double k_P = 0.4;
    public static final double k_I = 0.2;
    public static final double k_D = 0;
  }

  public static final class MoveYConstants {
    public static final double k_P = 0.4;
    public static final double k_I = 0.2;
    public static final double k_D = 0;
  }

  public static final class MoveHConstants {
    public static final double k_P = 0.1;
    public static final double k_I = 0;
    public static final double k_D = 0;
  }





  public static final class IntakeBallHConstants {
    public static final double k_P = 0.1;
    public static final double k_I = 0.0;
    public static final double k_D = 0.0;
  }

  public static final class IntakeBallGeneralConstants {
    public static final double H_SETPOINT = 0;
    public static final double H_TOLERANCE = 1.0;
  }
}
