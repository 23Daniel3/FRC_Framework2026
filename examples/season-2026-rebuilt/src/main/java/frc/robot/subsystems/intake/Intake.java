package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import org.ciprix.annotations.AutoCommandFactory;
import org.ciprix.interfaces.subsystem.StateSubsystem;

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

    // Devolve o que entrou: o pivot fica onde esta e so o roller inverte.
    fsm.state(IntakeState.REVERSING)
        .onEnter(
            () -> io.controlRollerMotor().runPercentOutput(IntakeConstants.INTAKE_REVERSE_POWER));

    // Request → (estado de entrada, estado goal, intermediarios protegidos).
    // OUT e COLLECT compartilham a mesma entrada (GOING_OUT); o alvo final e decidido
    // pelas transicoes locais de GOING_OUT com base no request atual.
    bindRequest(IntakeRequest.IN, IntakeState.GOING_IN, IntakeState.IN);
    bindRequest(IntakeRequest.OUT, IntakeState.GOING_OUT, IntakeState.OUT);
    bindRequest(IntakeRequest.COLLECT, IntakeState.GOING_OUT, IntakeState.COLLECT);
    bindRequest(IntakeRequest.STOP, IntakeState.STOPPING, IntakeState.STOPPED);
    bindRequest(IntakeRequest.REVERSE, IntakeState.REVERSING, IntakeState.REVERSING);

    fsm.validateComplete();

    // Autoteste de pit: so o roller, que pode girar livre (o pivot bateria no batente).
    selfTest()
        .motor("Roller", io::controlRollerMotor, () -> inputs.rollerMotorInputs)
        .output(0.3)
        .currentBetween(Amps.of(0.3), Amps.of(25))
        .mustTurn(RotationsPerSecond.of(5));
  }

  /**
   * Angulo do pivot para a visualizacao 3D: zero na posicao inicial (como no CAD) e {@link
   * IntakeConstants#PIVOT_DEPLOY_DEGREES} em OUT. Vem dos inputs, entao vale no robo, na simulacao
   * e no replay.
   */
  public Angle getPivotAngle() {
    double travel =
        inputs.intakeMotorInputs.position.in(Rotations) - IntakeConstants.INTAKE_START_POSITION;
    double fullTravel = IntakeConstants.INTAKE_OUT_POSITION - IntakeConstants.INTAKE_START_POSITION;
    return Degrees.of(travel / fullTravel * IntakeConstants.PIVOT_DEPLOY_DEGREES);
  }
}
