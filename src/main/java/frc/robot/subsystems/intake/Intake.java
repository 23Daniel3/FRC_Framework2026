package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.interfaces.subsystem.StateSubsystem;
import frc.lib.util.ConstantsLogger;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRequest;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

public class Intake extends StateSubsystem<
        IntakeConstants.IntakeRequest, IntakeConstants.IntakeState, IntakeIOInputsAutoLogged> {

  private final IntakeIO io;

  public Intake(IntakeIO io) {
    super(
        "Subsystems/Intake",
        new IntakeIOInputsAutoLogged(),
        IntakeState.class,
        IntakeState.IN,
        IntakeRequest.IN);
    this.io = io;
    setName("Subsystems/Intake");
    ConstantsLogger.logConstants(IntakeConstants.class, getName());

    fsm.state(IntakeState.GOING_OUT)
        .onEnter(
            () -> {
              io.controlIntakeMotor()
                  .runPosition(Rotations.of(IntakeConstants.INTAKE_OUT_POSITION));
              io.controlRollerMotor().runVelocity(RPM.of(0));
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
              io.controlRollerMotor().runVelocity(RPM.of(0));
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
              io.controlRollerMotor().runVelocity(RPM.of(0));
            });

    fsm.addGlobalTransition(
        IntakeState.GOING_OUT,
        () -> (currentRequest == IntakeRequest.COLLECT || currentRequest == IntakeRequest.OUT) );
    fsm.addGlobalTransition(IntakeState.GOING_IN, () -> currentRequest == IntakeRequest.IN);
  }

  @Override
  public void sPeriodic() {
  }

  public MotorIOInputs getRollerMotorInputs() {
    return inputs.rollerMotorInputs;
  }

  public MotorIOInputs getIntakeMotorInputs() {
    return inputs.intakeMotorInputs;
  }

  @Override
  public boolean atGoal() {
    return false;
  }

  @Override
  protected void updateInputs() {
    io.updateInputs(inputs);
  }
}
