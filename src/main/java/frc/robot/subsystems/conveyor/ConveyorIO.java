package frc.robot.subsystems.conveyor;

import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.interfaces.subsystem.SubsystemIO;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO extends SubsystemIO<ConveyorIOInputsAutoLogged> {
  @AutoLog
  public static class ConveyorIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public void updateInputs(ConveyorIOInputsAutoLogged inputs);

  public MotorController controlMotor();
}
