package frc.robot.subsystems.conveyor;

import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.interfaces.motor.advanced.MotorControllerNone;
import org.ciprix.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import org.ciprix.interfaces.subsystem.SubsystemIO;
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
