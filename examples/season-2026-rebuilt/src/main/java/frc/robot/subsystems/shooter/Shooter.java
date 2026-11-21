package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRequest;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import org.ciprix.annotations.AutoCommandFactory;
import org.ciprix.interfaces.subsystem.StateSubsystem;
import org.ciprix.util.SetpointTracker;

@AutoCommandFactory(requestEnum = ShooterConstants.ShooterRequest.class)
public class Shooter
    extends StateSubsystem<ShooterRequest, ShooterState, ShooterIOInputsAutoLogged, ShooterIO> {

  private AngularVelocity velocity = RPM.of(3000.0);

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
        .onEnter(() -> io.controlKicker().runVelocity(velocity))
        .onUpdate(() -> io.controlFlywheel().runVelocity(velocity))
        .transitionTo(ShooterState.KICKER_RAMPING, () -> isFlywheelReadyToKick());

    fsm.state(ShooterState.KICKER_RAMPING)
        .onUpdate(
            () -> {
              io.controlFlywheel().runVelocity(velocity);
              io.controlKicker().runVelocity(velocity);
            })
        .transitionTo(ShooterState.SHOOTING, () -> almostReadyToShoot());

    fsm.state(ShooterState.SHOOTING)
        .onUpdate(
            () -> {
              io.controlFlywheel().runVelocity(velocity);
              io.controlKicker().runVelocity(velocity);
            });
    // .transitionTo(ShooterState.KICKER_RAMPING, () -> !readyToStateShooting());

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

    // Autoteste de pit. O flywheel tem mais inercia: mais tempo para chegar na velocidade antes
    // de medir. O follower acompanha o lider, entao a corrente dele aparece nos logs tambem.
    selfTest()
        .motor("Flywheel", io::controlFlywheel, () -> inputs.leaderInputs)
        .output(0.2)
        .forSeconds(1.5)
        .currentBetween(Amps.of(0.3), Amps.of(30))
        .mustTurn(RotationsPerSecond.of(5));
    selfTest()
        .motor("Kicker", io::controlKicker, () -> inputs.kickerInputs)
        .output(0.3)
        .currentBetween(Amps.of(0.2), Amps.of(25))
        .mustTurn(RotationsPerSecond.of(5));
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
    return inputs.kickerInputs.atSetpoint;
  }

  /**
   * Define a velocidade alvo do flywheel/kicker. Deve ser alimentada continuamente (todo ciclo)
   * pela SuperStructure com o RPM calculado pelo SOTM.
   */
  public void setVelocity(AngularVelocity velocity) {
    this.velocity = velocity;
  }
}
