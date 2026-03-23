package frc.robot.commands;

public final class CommandConstants {

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

  public static final class AlignToReefXConstants {
    public static final double k_P = 3.2;
    public static final double k_I = 0.0;
    public static final double k_D = 0;
  }

  public static final class AlignToReefYConstants {
    public static final double k_P = 3.8;
    public static final double k_I = 0.0;
    public static final double k_D = 0.0;
  }

  public static final class AlignToReefHConstants {
    public static final double k_P = 0.1;
    public static final double k_I = 0.0;
    public static final double k_D = 0.0;
  }

  public static final class AlignToReefGeneralConstants {
    public static final double X_SETPOINT = -0.61;
    public static final double Y_SETPOINT_LEFT = -0.2;
    public static final double Y_SETPOINT_RIGHT = 0.15;
    public static final double H_SETPOINT = -2.0;

    public static final double X_TOLERANCE = 0.02;
    public static final double Y_TOLERANCE = 0.02;
    public static final double H_TOLERANCE = 3.0;

    public static final int CAMERA_X_VALUE = 2;
    public static final int CAMERA_Y_VALUE = 0;
    public static final int CAMERA_H_VALUE = 4;
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
