package frc.robot.subsystems.climber;

import frc.lib.interfaces.subsystem.StateSubsystem;
import frc.robot.subsystems.climber.ClimberConstants.ClimberRequest;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;

public class Climber
    extends StateSubsystem<ClimberRequest, ClimberState, ClimberIOInputsAutoLogged, ClimberIO> {

  public Climber(ClimberIO io) {
    super(
        "Subsystems/Climber",
        new ClimberIOInputsAutoLogged(),
        io,
        ClimberState.class,
        ClimberState.RETRACTED,
        ClimberRequest.LOW,
        ClimberConstants.class);

    fsm.state(ClimberState.EXTENDING)
        .onEnter(() -> io.controlMotor().runPosition(ClimberConstants.HIGH_POSITION))
        .transitionTo(ClimberState.EXTENDED, () -> inputs.motorInputs.atSetpoint);

    fsm.state(ClimberState.RETRACTING)
        .onEnter(() -> io.controlMotor().runPosition(ClimberConstants.LOW_POSITION))
        .transitionTo(ClimberState.RETRACTED, () -> inputs.motorInputs.atSetpoint);

    fsm.addGlobalTransition(
        ClimberState.EXTENDING,
        () ->
            currentRequest == ClimberRequest.HIGH
                && getState() != ClimberState.EXTENDED);

    fsm.addGlobalTransition(
        ClimberState.RETRACTING,
        () ->
            currentRequest == ClimberRequest.LOW
                && getState() != ClimberState.RETRACTED);
  }

  @Override
  public boolean atGoal() {
    return switch (currentRequest) {
      case HIGH -> getState() == ClimberState.EXTENDED;
      case LOW -> getState() == ClimberState.RETRACTED;
    };
  }
}
