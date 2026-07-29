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

    // Setpoints aplicados no onUpdate: o RPM do SOTM muda a cada ciclo enquanto o robo
    // se move, entao o alvo precisa ser reaplicado continuamente (nao apenas no onEnter).
    fsm.state(ShooterState.FLYWHEEL_RAMPING)
        .onEnter(() -> io.controlKicker().stop())
        .onUpdate(() -> io.controlFlywheel().runVelocity(velocity))
        .transitionTo(ShooterState.KICKER_RAMPING, () -> isFlywheelReadyToKick());

    fsm.state(ShooterState.KICKER_RAMPING)
        .onUpdate(
            () -> {
              io.controlFlywheel().runVelocity(velocity);
              io.controlKicker().runVelocity(velocity);
            })
        .transitionTo(ShooterState.SHOOTING, () -> readyToStateShooting());

    fsm.state(ShooterState.SHOOTING)
        .onUpdate(
            () -> {
              io.controlFlywheel().runVelocity(velocity);
              io.controlKicker().runVelocity(velocity);
            })
        .transitionTo(ShooterState.KICKER_RAMPING, () -> !readyToStateShooting());

    fsm.state(ShooterState.REVERSING)
        .onEnter(
            () -> {
              io.controlFlywheel().runPercentOutput(ShooterConstants.REVERSE_POWER);
              io.controlKicker().runPercentOutput(ShooterConstants.KICKER_REVERSE_POWER);
            });

    // Request → (estado de entrada, estado goal, intermediarios protegidos).
    // atGoal() e derivado automaticamente destes vinculos.
    bindRequest(ShooterRequest.STOP, ShooterState.IDLE, ShooterState.IDLE);
    bindRequest(
        ShooterRequest.SHOOT,
        ShooterState.FLYWHEEL_RAMPING,
        ShooterState.SHOOTING,
        ShooterState.KICKER_RAMPING);
    bindRequest(ShooterRequest.REVERSE, ShooterState.REVERSING, ShooterState.REVERSING);

    fsm.validateComplete();
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

  /**
   * Define a velocidade alvo do flywheel/kicker. Deve ser alimentada continuamente (todo ciclo)
   * pela SuperStructure com o RPM calculado pelo SOTM.
   */
  public void setVelocity(AngularVelocity velocity) {
    this.velocity = velocity;
  }
}
