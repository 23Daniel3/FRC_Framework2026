package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.interfaces.subsystem.StateSubsystem;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorRequest;
import frc.robot.subsystems.conveyor.ConveyorConstants.ConveyorState;

public class Conveyor
    extends StateSubsystem<ConveyorRequest, ConveyorState, ConveyorIOInputsAutoLogged, ConveyorIO> {

  public Conveyor(ConveyorIO io) {
    super(
        "Subsystems/Conveyor",
        new ConveyorIOInputsAutoLogged(),
        io,
        ConveyorState.class,
        ConveyorState.IDLE,
        ConveyorRequest.STOP,
        ConveyorConstants.class);

    // --- Definicao dos Estados ---

    fsm.state(ConveyorState.IDLE).onEnter(() -> io.controlMotor().stop());

    fsm.state(ConveyorState.RUNNING)
        .onEnter(() -> io.controlMotor().runPercentOutput(ConveyorConstants.POWER));

    fsm.state(ConveyorState.RUNNING_SLOW)
        .onEnter(() -> io.controlMotor().runPercentOutput(ConveyorConstants.SLOW_POWER));

    fsm.state(ConveyorState.REVERSING)
        .onEnter(() -> io.controlMotor().runPercentOutput(ConveyorConstants.REVERSE_POWER));

    fsm.state(ConveyorState.REVERSING_SLOW)
        .onEnter(() -> io.controlMotor().runPercentOutput(ConveyorConstants.SLOW_REVERSE_POWER));

    fsm.state(ConveyorState.WIGGLING)
        .onUpdate(
            () -> {
              double currentTime = Timer.getFPGATimestamp();
              double phase = currentTime % ConveyorConstants.WIGGLE_PERIOD;
              boolean forward = phase < (ConveyorConstants.WIGGLE_PERIOD / 2.0);

              double power =
                  forward ? ConveyorConstants.WIGGLE_POWER : -ConveyorConstants.WIGGLE_POWER;
              io.controlMotor().runPercentOutput(power);
            });

    // Requests diretos: entrada == goal. atGoal() e derivado automaticamente.
    bindRequest(ConveyorRequest.STOP, ConveyorState.IDLE, ConveyorState.IDLE);
    bindRequest(ConveyorRequest.RUN, ConveyorState.RUNNING, ConveyorState.RUNNING);
    bindRequest(ConveyorRequest.RUN_SLOW, ConveyorState.RUNNING_SLOW, ConveyorState.RUNNING_SLOW);
    bindRequest(ConveyorRequest.REVERSE, ConveyorState.REVERSING, ConveyorState.REVERSING);
    bindRequest(
        ConveyorRequest.SLOW_REVERSE, ConveyorState.REVERSING_SLOW, ConveyorState.REVERSING_SLOW);
    bindRequest(ConveyorRequest.WIGGLE, ConveyorState.WIGGLING, ConveyorState.WIGGLING);

    fsm.validateComplete();
  }
}
