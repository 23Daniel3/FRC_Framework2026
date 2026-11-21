package frc.robot.subsystems.shooter;

import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.interfaces.motor.advanced.MotorControllerNone;
import org.ciprix.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import org.ciprix.interfaces.subsystem.SubsystemIO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO extends SubsystemIO<ShooterIOInputsAutoLogged> {
  @AutoLog
  public static class ShooterIOInputs {
    public MotorIOInputs leaderInputs = new MotorIOInputs();
    public MotorIOInputs followerInputs = new MotorIOInputs();
    public MotorIOInputs kickerInputs = new MotorIOInputs();
  }

  public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}

  public default MotorController controlFlywheel() {
    return new MotorControllerNone();
  }

  public default MotorController controlKicker() {
    return new MotorControllerNone();
  }
}
