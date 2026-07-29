package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import frc.lib.interfaces.subsystem.StateSubsystem;
import frc.robot.annotations.AutoCommandFactory;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

@AutoCommandFactory(requestEnum = IntakeConstants.IntakeRequest.class)
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

    fsm.state(IntakeState.OUT).onEnter(() -> io.controlRollerMotor().stop());

    fsm.state(IntakeState.STOPPING)
        .onEnter(
            () -> {
              io.controlRollerMotor().stop();
            })
        // Deadband em vez de igualdade exata com zero: com ruido de sensor a leitura
        // raramente e exatamente 0.0, o que travava a FSM em STOPPING para sempre.
        .transitionTo(
            IntakeState.STOPPED,
            () ->
                Math.abs(inputs.rollerMotorInputs.velocity.in(RPM))
                    < IntakeConstants.STOPPED_RPM_TOLERANCE);

    fsm.state(IntakeState.STOPPED).onEnter(() -> io.controlRollerMotor().stop());

    // Request → (estado de entrada, estado goal, intermediarios protegidos).
    // OUT e COLLECT compartilham a mesma entrada (GOING_OUT); o alvo final e decidido
    // pelas transicoes locais de GOING_OUT com base no request atual.
    bindRequest(IntakeRequest.IN, IntakeState.GOING_IN, IntakeState.IN);
    bindRequest(IntakeRequest.OUT, IntakeState.GOING_OUT, IntakeState.OUT);
    bindRequest(IntakeRequest.COLLECT, IntakeState.GOING_OUT, IntakeState.COLLECT);
    bindRequest(IntakeRequest.STOP, IntakeState.STOPPING, IntakeState.STOPPED);

    fsm.validateComplete();
  }
}
