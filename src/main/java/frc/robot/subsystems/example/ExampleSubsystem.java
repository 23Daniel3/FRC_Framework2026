package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.RPM;

import frc.lib.interfaces.subsystem.StateSubsystem;
import frc.robot.subsystems.example.ExampleConstants.ExampleRequest;
import frc.robot.subsystems.example.ExampleConstants.ExampleState;

/**
 * TEMPLATE DE REFERENCIA — NAO instanciado no robo. Este arquivo demonstra o padrao completo de um
 * subsistema neste codigo: StateSubsystem + FSM declarativa + bindRequest.
 *
 * <p><b>Fluxo de controle (Modelo A):</b>
 *
 * <pre>
 * piloto → manageRequests → SuperStructure (RobotRequest)
 *        → FSM geral (onEnter) → exampleSubsystem.setRequest(ExampleRequest)
 *        → FSM deste subsistema → io.controlMotor() → hardware
 * </pre>
 *
 * <p>NINGUEM alem da SuperStructure deve chamar setRequest em teleop — nao crie Commands que
 * escrevem requests de subsistemas diretamente (eles brigariam com a FSM geral).
 *
 * <p><b>Regras que este exemplo demonstra:</b>
 *
 * <ol>
 *   <li>{@code onEnter} para efeitos discretos; {@code onUpdate} para setpoints dinamicos;
 *   <li>{@code bindRequest(request, entrada, goal, intermediarios...)} declara a transicao global
 *       E deriva {@code atGoal()} — nao sobrescreva atGoal com switch manual;
 *   <li>Deadband em comparacoes com sensores (nunca {@code == 0});
 *   <li>{@code fsm.validateComplete()} no fim do construtor pega estados esquecidos em bancada.
 * </ol>
 *
 * <p>Para criar um subsistema novo a partir deste: use {@code tools/new_subsystem.py NomeDoMecanismo}
 * ou copie a pasta e renomeie Example → NomeDoMecanismo em classes, arquivos e enums. Depois:
 * instancie no RobotContainer com o IO certo por modo (Hardware / Sim / replay {}), adicione os
 * requests aos onEnter da FSM geral da SuperStructure, e mapeie a "cara" do robo em
 * LedCommands.STATE_EFFECTS se o novo estado geral existir.
 */
public class ExampleSubsystem
    extends StateSubsystem<ExampleRequest, ExampleState, ExampleIOInputsAutoLogged, ExampleIO> {

  public ExampleSubsystem(ExampleIO io) {
    super(
        "Subsystems/Example", // prefixo dos logs no AdvantageKit
        new ExampleIOInputsAutoLogged(), // inputs gerados pelo @AutoLog
        io, // Hardware, Sim ou replay, decidido no RobotContainer
        ExampleState.class,
        ExampleState.STOPPED, // estado inicial
        ExampleRequest.STOP, // request inicial
        ExampleConstants.class); // constantes logadas automaticamente

    // --- Estados -------------------------------------------------------------------------
    // Efeito discreto: aplicar uma vez, no onEnter. Se o setpoint fosse dinamico
    // (ex.: RPM vindo do SOTM), seria io.controlMotor().runVelocity(alvo) no onUpdate.
    fsm.state(ExampleState.RUNNING)
        .onEnter(() -> io.controlMotor().runPercentOutput(ExampleConstants.FORWARD_POWER));

    fsm.state(ExampleState.STOPPING)
        .onEnter(() -> io.controlMotor().stop())
        // Par settling/settled com deadband: espera o mecanismo REALMENTE parar.
        .transitionTo(
            ExampleState.STOPPED,
            () ->
                Math.abs(inputs.motorInputs.velocity.in(RPM))
                    < ExampleConstants.STOPPED_RPM_TOLERANCE);

    fsm.state(ExampleState.STOPPED).onEnter(() -> io.controlMotor().stop());

    // --- Requests ------------------------------------------------------------------------
    // bindRequest(request, estadoDeEntrada, estadoGoal, intermediarios...):
    // registra a transicao global (com as exclusoes certas) e alimenta o atGoal() derivado.
    bindRequest(ExampleRequest.RUN, ExampleState.RUNNING, ExampleState.RUNNING);
    bindRequest(ExampleRequest.STOP, ExampleState.STOPPING, ExampleState.STOPPED);

    // Reporta warning no DriverStation para qualquer estado do enum sem configuracao.
    fsm.validateComplete();
  }
}
