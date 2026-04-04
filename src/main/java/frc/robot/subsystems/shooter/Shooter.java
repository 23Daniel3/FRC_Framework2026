package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.interfaces.subsystem.StateSubsystem;
import frc.lib.util.SetpointTracker;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRequest;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class Shooter
    extends StateSubsystem<ShooterRequest, ShooterState, ShooterIOInputsAutoLogged, ShooterIO> {

  private AngularVelocity velocity = RPM.of(0.0);

  public Shooter(ShooterIO io) {
    super(
        "Subsystems/Shooter",
        new ShooterIOInputsAutoLogged(),
        io,
        ShooterState.class,
        ShooterState.IDLE,
        ShooterRequest.STOP,
        ShooterConstants.class);

    fsm.state(ShooterState.IDLE)
        .onEnter(
            () -> {
              io.controlFlywheel().stop();
              io.controlKicker().stop();
            });

    fsm.state(ShooterState.FLYWHEEL_RAMPING)
        .onEnter(
            () -> {
              io.controlFlywheel().runVelocity(velocity);
              io.controlKicker().stop();
            })
        .transitionTo(ShooterState.KICKER_RAMPING, () -> isFlywheelReadyToKick());

    fsm.state(ShooterState.KICKER_RAMPING)
        .onEnter(
            () -> {
              io.controlFlywheel().runVelocity(velocity);
              io.controlKicker().runVelocity(velocity);
            })
        .transitionTo(ShooterState.SHOOTING, () -> readyToStateShooting());

    fsm.state(ShooterState.SHOOTING)
        .onEnter(
            () -> {
              io.controlFlywheel().runVelocity(velocity);
              io.controlKicker().runVelocity(velocity);
            })
        .transitionTo(ShooterState.FLYWHEEL_RAMPING, () -> !readyToStateShooting());

    fsm.state(ShooterState.REVERSING)
        .onEnter(
            () -> {
              io.controlFlywheel().runPercentOutput(ShooterConstants.REVERSE_POWER);
              io.controlKicker().runPercentOutput(ShooterConstants.KICKER_REVERSE_POWER);
            });

    fsm.addGlobalTransition(
        ShooterState.IDLE, () -> isRequest(ShooterRequest.STOP) && notInState(ShooterState.IDLE));

    fsm.addGlobalTransition(
        ShooterState.FLYWHEEL_RAMPING,
        () ->
            isRequest(ShooterRequest.SHOOT)
                && notInState(ShooterState.KICKER_RAMPING)
                && notInState(ShooterState.SHOOTING));

    fsm.addGlobalTransition(
        ShooterState.REVERSING,
        () -> isRequest(ShooterRequest.REVERSE) && notInState(ShooterState.REVERSING));
  }

  private boolean isFlywheelReadyToKick() {
    return SetpointTracker.atSetpoint(
        velocity.in(RPM),
        ShooterConstants.START_KICKER_TOLERANCE,
        inputs.leaderInputs.velocity.in(RPM));
  }

  private boolean readyToStateShooting() {
    return inputs.leaderInputs.atSetpoint && inputs.kickerInputs.atSetpoint;
  }

  public boolean readyToShoot() {
    return getState() == ShooterState.SHOOTING;
  }

  public boolean almostReadyToShoot() {
    return getState() == ShooterState.KICKER_RAMPING;
  }

  @Override
  public boolean atGoal() {
    return switch (currentRequest) {
      case STOP -> getState() == ShooterState.IDLE;
      case SHOOT -> getState() == ShooterState.SHOOTING;
      case REVERSE -> getState() == ShooterState.REVERSING;
    };
  }

  public void setVelocity(AngularVelocity velocity) {
    this.velocity = velocity;
  }
}
