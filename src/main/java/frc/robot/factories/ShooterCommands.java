package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRequest;
import frc.robot.subsystems.superstructure.SuperStructure;

public final class ShooterCommands {

  private ShooterCommands() {}

  /** Default Command: Relays the target request calculated by the SuperStructure. */
  public static Command defaultCommand(SuperStructure superStructure, Shooter shooter) {
    return Commands.run(() -> shooter.setRequest(superStructure.getShooterRequest()), shooter)
        .withName("Shooter Default (Superstructure Relay)");
  }

  /** Manual Override: Stop shooter. */
  public static Command stop(Shooter shooter) {
    return Commands.run(() -> shooter.setRequest(ShooterRequest.STOP), shooter)
        .withName("Shooter Override STOP");
  }

  /** Manual Override: Run flywheel to shoot. */
  public static Command shoot(Shooter shooter) {
    return Commands.run(() -> shooter.setRequest(ShooterRequest.SHOOT), shooter)
        .withName("Shooter Override SHOOT");
  }

  /** Manual Override: Reverse shooter motors. */
  public static Command reverse(Shooter shooter) {
    return Commands.run(() -> shooter.setRequest(ShooterRequest.REVERSE), shooter)
        .withName("Shooter Override REVERSE");
  }
}
