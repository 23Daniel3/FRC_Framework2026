package frc.robot.subsystems.climber;

import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.interfaces.subsystem.SubsystemIO;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends SubsystemIO<ClimberIOInputsAutoLogged> {
  @AutoLog
  public static class ClimberIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  @Override
  public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

  public default MotorController controlMotor() {
    return null;
  }
}
