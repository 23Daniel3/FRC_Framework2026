package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SuperStructureCommands;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerCommands;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonFX;
import frc.robot.subsystems.superstructure.SuperStructure;
import org.ciprix.util.PreMatchChecklist;

/** A estrutura do robo: subsistemas, botoes do piloto e do copiloto, autonomo. */
public class RobotContainer {

  private final Roller roller;
  private final SuperStructure superStructure;

  private final CommandXboxController pilot = new CommandXboxController(0);
  private final CommandXboxController copilot = new CommandXboxController(1);

  public RobotContainer() {
    roller =
        new Roller(
            switch (Constants.currentMode) {
              case REAL -> new RollerIOTalonFX();
              case SIM -> new RollerIOSim();
              case REPLAY -> new RollerIO() {}; // os inputs vem do log
            });
    superStructure = new SuperStructure(roller);

    configureBindings();
    PreMatchChecklist.addDefaults(); // bateria, dispositivos conectados, sem falhas
  }

  private void configureBindings() {
    // Piloto: escolhe o request do robo; a FSM geral faz o resto.
    superStructure.setDefaultCommand(
        SuperStructureCommands.manageRequests(
            superStructure, pilot.leftBumper(), pilot.rightBumper()));

    // Cada subsistema segue a SuperStructure pelo relay gerado.
    roller.setDefaultCommand(RollerCommands.defaultCommand(superStructure, roller));

    // Copiloto: backup manual, so enquanto o botao estiver pressionado.
    copilot.a().whileTrue(RollerCommands.intake(roller));
    copilot.b().whileTrue(RollerCommands.eject(roller));

    // Autoteste de pit (robo habilitado, mecanismo livre).
    SmartDashboard.putData("SelfTest/Roller", roller.selfTestCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
