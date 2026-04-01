package frc.robot.subsystems.climber;

import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default MotorController controlMotor() {
    return null;
  }
}
