package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorRequest;
import frc.robot.subsystems.superstructure.SuperStructure;

public final class ConveyorCommands {

  private ConveyorCommands() {}

  public static Command defaultCommand(SuperStructure superStructure, Conveyor conveyor) {
    return Commands.run(() -> conveyor.setRequest(superStructure.getConveyorRequest()), conveyor);
  }

  public static Command run(Conveyor conveyor) {
    return Commands.runOnce(() -> conveyor.setRequest(ConveyorRequest.RUN), conveyor);
  }

  public static Command runSlow(Conveyor conveyor) {
    return Commands.runOnce(() -> conveyor.setRequest(ConveyorRequest.RUN_SLOW), conveyor);
  }

  public static Command reverse(Conveyor conveyor) {
    return Commands.runOnce(() -> conveyor.setRequest(ConveyorRequest.REVERSE), conveyor);
  }

  public static Command slowReverse(Conveyor conveyor) {
    return Commands.runOnce(() -> conveyor.setRequest(ConveyorRequest.SLOW_REVERSE), conveyor);
  }

  public static Command stop(Conveyor conveyor) {
    return Commands.runOnce(() -> conveyor.setRequest(ConveyorRequest.STOP), conveyor);
  }

  public static Command wiggle(Conveyor conveyor) {
    return Commands.runOnce(() -> conveyor.setRequest(ConveyorRequest.WIGGLE), conveyor);
  }
}
