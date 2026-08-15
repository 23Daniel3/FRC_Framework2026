package frc.robot.subsystems.shooter;

import frc.lib.interfaces.motor.advanced.MotorController;
import frc.lib.interfaces.motor.advanced.MotorControllerNone;
import frc.lib.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import frc.lib.interfaces.subsystem.SubsystemIO;
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
