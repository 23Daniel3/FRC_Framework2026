package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.roller.RollerConstants.RollerRequest;
import frc.robot.subsystems.roller.RollerConstants.RollerState;
import org.ciprix.annotations.AutoCommandFactory;
import org.ciprix.interfaces.subsystem.StateSubsystem;

/**
 * Um roller (intake simples), o menor subsistema possivel no padrao: cada request leva a um estado
 * e cada estado diz o que o motor faz. O {@code @AutoCommandFactory} gera {@code RollerCommands},
 * com o comando padrao (relay da SuperStructure) e um comando por request para o copiloto.
 */
@AutoCommandFactory(requestEnum = RollerRequest.class)
public class Roller
    extends StateSubsystem<RollerRequest, RollerState, RollerIOInputsAutoLogged, RollerIO> {

  public Roller(RollerIO io) {
    super(
        "Subsystems/Roller",
        new RollerIOInputsAutoLogged(),
        io,
        RollerState.class,
        RollerState.STOPPED,
        RollerRequest.STOP,
        RollerConstants.class);

    fsm.state(RollerState.STOPPED).onEnter(() -> io.controlMotor().stop());
    fsm.state(RollerState.INTAKING)
        .onEnter(() -> io.controlMotor().runPercentOutput(RollerConstants.INTAKE_OUTPUT));
    fsm.state(RollerState.EJECTING)
        .onEnter(() -> io.controlMotor().runPercentOutput(RollerConstants.EJECT_OUTPUT));

    // Request -> (estado de entrada, estado goal).
    bindRequest(RollerRequest.STOP, RollerState.STOPPED, RollerState.STOPPED);
    bindRequest(RollerRequest.INTAKE, RollerState.INTAKING, RollerState.INTAKING);
    bindRequest(RollerRequest.EJECT, RollerState.EJECTING, RollerState.EJECTING);
    fsm.validateComplete();

    // Autoteste de pit: gira o motor por um instante e confere corrente e velocidade.
    selfTest()
        .motor("Motor", io::controlMotor, () -> inputs.motorInputs)
        .output(0.3)
        .mustTurn(RotationsPerSecond.of(5));
  }
}
