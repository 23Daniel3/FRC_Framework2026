package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import frc.lib.interfaces.subsystem.StateSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

public class Intake
    extends StateSubsystem<
        IntakeConstants.IntakeRequest,
        IntakeConstants.IntakeState,
        IntakeIOInputsAutoLogged,
        IntakeIO> {

  public Intake(IntakeIO io) {
    super(
        "Subsystems/Intake",
        new IntakeIOInputsAutoLogged(),
        io,
        IntakeState.class,
        IntakeState.IN,
        IntakeRequest.IN,
        IntakeConstants.class);

    fsm.state(IntakeState.GOING_OUT)
        .onEnter(
            () -> {
              io.controlIntakeMotor()
                  .runPosition(Rotations.of(IntakeConstants.INTAKE_OUT_POSITION));
              io.controlRollerMotor().stop();
            })
        .transitionTo(
            IntakeState.OUT,
            () -> inputs.intakeMotorInputs.atSetpoint && currentRequest == IntakeRequest.OUT)
        .transitionTo(
            IntakeState.COLLECT,
            () -> inputs.intakeMotorInputs.atSetpoint && currentRequest == IntakeRequest.COLLECT);

    fsm.state(IntakeState.GOING_IN)
        .onEnter(
            () -> {
              io.controlIntakeMotor().runPosition(Rotations.of(IntakeConstants.INTAKE_IN_POSITION));
              io.controlRollerMotor().stop();
            })
        .transitionTo(IntakeState.IN, () -> inputs.intakeMotorInputs.atSetpoint);

    fsm.state(IntakeState.COLLECT)
        .onEnter(
            () -> {
              io.controlRollerMotor().runVelocity(RPM.of(IntakeConstants.INTAKE_MAX_VELOCITY));
            });

    fsm.state(IntakeState.IN)
        .onEnter(
            () -> {
              io.controlRollerMotor().stop();
            });

    fsm.state(IntakeState.STOPING)
        .onEnter(
            () -> {
              io.controlRollerMotor().stop();
            })
        .transitionTo(IntakeState.STOPPED, () -> inputs.rollerMotorInputs.velocity.in(RPM) == 0);

    fsm.addGlobalTransition(
        IntakeState.GOING_OUT,
        () -> (currentRequest == IntakeRequest.COLLECT) && (getState() != IntakeState.COLLECT));

    fsm.addGlobalTransition(
        IntakeState.GOING_OUT,
        () -> (currentRequest == IntakeRequest.OUT) && (getState() != IntakeState.OUT));

    fsm.addGlobalTransition(IntakeState.GOING_IN, () -> currentRequest == IntakeRequest.IN);
  }

  @Override
  public boolean atGoal() {
    return switch (currentRequest) {
      case IN -> getState() == IntakeState.IN;
      case OUT -> getState() == IntakeState.OUT;
      case COLLECT -> getState() == IntakeState.COLLECT;
      case STOP -> getState() == IntakeState.STOPPED;
    };
  }
}
