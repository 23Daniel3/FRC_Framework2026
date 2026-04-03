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

    // --- Definição dos Estados ---

    fsm.state(ConveyorState.IDLE)
        .onEnter(() -> io.controlMotor().stop());

    fsm.state(ConveyorState.RUNNING)
        .onEnter(() -> io.controlMotor().runPercentOutput(ConveyorConstants.POWER));

    fsm.state(ConveyorState.RUNNING_SLOW)
        .onEnter(() -> io.controlMotor().runPercentOutput(ConveyorConstants.SLOW_POWER));

    fsm.state(ConveyorState.REVERSING)
        .onEnter(() -> io.controlMotor().runPercentOutput(ConveyorConstants.REVERSE_POWER));

    fsm.state(ConveyorState.REVERSING_SLOW)
        .onEnter(() -> io.controlMotor().runPercentOutput(ConveyorConstants.SLOW_REVERSE_POWER));

    fsm.state(ConveyorState.WIGGLING)
        .onUpdate(() -> {
            double currentTime = Timer.getFPGATimestamp();
            double phase = currentTime % ConveyorConstants.WIGGLE_PERIOD;
            boolean forward = phase < (ConveyorConstants.WIGGLE_PERIOD / 2.0);

            double power =
                forward ? ConveyorConstants.WIGGLE_POWER : -ConveyorConstants.WIGGLE_POWER;
            io.controlMotor().runPercentOutput(power);
        });

    fsm.addGlobalTransition(ConveyorState.IDLE, 
        () -> isRequest(ConveyorRequest.STOP) && notInState(ConveyorState.IDLE));

    fsm.addGlobalTransition(ConveyorState.RUNNING, 
        () -> isRequest(ConveyorRequest.RUN) && notInState(ConveyorState.RUNNING));

    fsm.addGlobalTransition(ConveyorState.RUNNING_SLOW, 
        () -> isRequest(ConveyorRequest.RUN_SLOW) && notInState(ConveyorState.RUNNING_SLOW));

    fsm.addGlobalTransition(ConveyorState.REVERSING, 
        () -> isRequest(ConveyorRequest.REVERSE) && notInState(ConveyorState.REVERSING));

    fsm.addGlobalTransition(ConveyorState.REVERSING_SLOW, 
        () -> isRequest(ConveyorRequest.SLOW_REVERSE) && notInState(ConveyorState.REVERSING_SLOW));

    fsm.addGlobalTransition(ConveyorState.WIGGLING, 
        () -> isRequest(ConveyorRequest.WIGGLE) && notInState(ConveyorState.WIGGLING));
  }

  @Override
  public boolean atGoal() {
    return switch (currentRequest) {
      case RUN -> getState() == ConveyorState.RUNNING;
      case RUN_SLOW -> getState() == ConveyorState.RUNNING_SLOW;
      case REVERSE -> getState() == ConveyorState.REVERSING;
      case SLOW_REVERSE -> getState() == ConveyorState.REVERSING_SLOW;
      case WIGGLE -> getState() == ConveyorState.WIGGLING;
      case STOP -> getState() == ConveyorState.IDLE;
      case NON_INTENTION -> true;
    };
  }
}