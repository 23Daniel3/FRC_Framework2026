package frc.robot.subsystems.conveyor;

import frc.lib.interfaces.motor.advanced.MotorController;
import frc.lib.interfaces.motor.advanced.MotorControllerNone;
import frc.lib.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import frc.lib.interfaces.subsystem.SubsystemIO;
import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO extends SubsystemIO<ConveyorIOInputsAutoLogged> {
  @AutoLog
  public static class ConveyorIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  public default void updateInputs(ConveyorIOInputsAutoLogged inputs) {}

  public default MotorController controlMotor() {
    return new MotorControllerNone();
  }
}
