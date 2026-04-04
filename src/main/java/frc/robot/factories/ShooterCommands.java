package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRequest;
import frc.robot.subsystems.superstructure.SuperStructure;

public final class ShooterCommands {
  
  private ShooterCommands() {}

  public static Command defaultCommand(SuperStructure superStructure, Shooter shooter) {
    return Commands.run(() -> shooter.setRequest(superStructure.getShooterRequest()), shooter);
  }

  public static Command stop(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setRequest(ShooterRequest.STOP), shooter);
  }

  public static Command shoot(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setRequest(ShooterRequest.SHOOT), shooter);
  }

  public static Command reverse(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setRequest(ShooterRequest.REVERSE), shooter);
  }
}