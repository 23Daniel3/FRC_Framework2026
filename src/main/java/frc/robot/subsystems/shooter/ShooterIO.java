package frc.robot.subsystems.shooter;

import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.interfaces.subsystem.SubsystemIO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO extends SubsystemIO<ShooterIOInputsAutoLogged> {
  @AutoLog
  public static class ShooterIOInputs {
    public MotorIOInputs leaderInputs = new MotorIOInputs();
    public MotorIOInputs followerInputs = new MotorIOInputs();
    public MotorIOInputs kickerInputs = new MotorIOInputs();
  }

  public void updateInputs(ShooterIOInputsAutoLogged inputs);

  public MotorController controlFlywheel();

  public MotorController controlKicker();
}
