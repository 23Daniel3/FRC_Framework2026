package frc.robot.subsystems.roller;

import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.interfaces.motor.advanced.MotorControllerNone;
import org.ciprix.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import org.ciprix.interfaces.subsystem.SubsystemIO;
import org.littletonrobotics.junction.AutoLog;

/** Camada de hardware do roller: o subsistema so fala com o motor por aqui. */
public interface RollerIO extends SubsystemIO<RollerIOInputsAutoLogged> {
  @AutoLog
  class RollerIOInputs {
    public MotorIOInputs motorInputs = new MotorIOInputs();
  }

  @Override
  default void updateInputs(RollerIOInputsAutoLogged inputs) {}

  default MotorController controlMotor() {
    return new MotorControllerNone();
  }
}
