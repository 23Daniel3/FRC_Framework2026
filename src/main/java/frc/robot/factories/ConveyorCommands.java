package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorRequest;
import frc.robot.subsystems.superstructure.SuperStructure;

public final class ConveyorCommands {

  private ConveyorCommands() {}

  /** Default Command: Relays the target request calculated by the SuperStructure. */
  public static Command defaultCommand(SuperStructure superStructure, Conveyor conveyor) {
    return Commands.run(() -> conveyor.setRequest(superStructure.getConveyorRequest()), conveyor)
        .withName("Conveyor Default (Superstructure Relay)");
  }

  /** Manual Override: Run conveyor. */
  public static Command run(Conveyor conveyor) {
    return Commands.run(() -> conveyor.setRequest(ConveyorRequest.RUN), conveyor)
        .withName("Conveyor Override RUN");
  }

  /** Manual Override: Run conveyor slow. */
  public static Command runSlow(Conveyor conveyor) {
    return Commands.run(() -> conveyor.setRequest(ConveyorRequest.RUN_SLOW), conveyor)
        .withName("Conveyor Override RUN_SLOW");
  }

  /** Manual Override: Reverse conveyor. */
  public static Command reverse(Conveyor conveyor) {
    return Commands.run(() -> conveyor.setRequest(ConveyorRequest.REVERSE), conveyor)
        .withName("Conveyor Override REVERSE");
  }

  /** Manual Override: Slow reverse conveyor. */
  public static Command slowReverse(Conveyor conveyor) {
    return Commands.run(() -> conveyor.setRequest(ConveyorRequest.SLOW_REVERSE), conveyor)
        .withName("Conveyor Override SLOW_REVERSE");
  }

  /** Manual Override: Stop conveyor. */
  public static Command stop(Conveyor conveyor) {
    return Commands.run(() -> conveyor.setRequest(ConveyorRequest.STOP), conveyor)
        .withName("Conveyor Override STOP");
  }

  /** Manual Override: Wiggle conveyor. */
  public static Command wiggle(Conveyor conveyor) {
    return Commands.run(() -> conveyor.setRequest(ConveyorRequest.WIGGLE), conveyor)
        .withName("Conveyor Override WIGGLE");
  }
}
