package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ciprix.CiprixSettings;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * O robo: configura a Ciprix e o AdvantageKit e roda o agendador de comandos. A estrutura
 * (subsistemas, botoes, autonomo) fica em {@link RobotContainer}.
 */
public class Robot extends LoggedRobot {
  private final RobotContainer robotContainer;
  private Command autonomousCommand;

  public Robot() {
    // A Ciprix vem primeiro: tudo o que e criado depois le estas configuracoes.
    configureCiprix();

    Logger.recordMetadata("ProjectName", "skeleton");
    Logger.recordMetadata("BuildTime", java.time.Instant.now().toString());
    switch (Constants.currentMode) {
      case REAL -> {
        // No robo: log no pendrive ("/U/logs") e ao vivo no NetworkTables.
        Logger.addDataReceiver(new WPILOGWriter("/U/logs/"));
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> Logger.addDataReceiver(new NT4Publisher());
      case REPLAY -> {
        // Replay: le um log e escreve outro com o sufixo "_sim", o mais rapido possivel.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }
    Logger.start();

    robotContainer = new RobotContainer();
  }

  /** Entrega as constantes do projeto para a Ciprix (ela nunca le {@link Constants} direto). */
  private static void configureCiprix() {
    CiprixSettings.setMode(
        switch (Constants.currentMode) {
          case REAL -> CiprixSettings.Mode.REAL;
          case SIM -> CiprixSettings.Mode.SIM;
          case REPLAY -> CiprixSettings.Mode.REPLAY;
        });
    CiprixSettings.setTuningMode(Constants.tuningMode);
    CiprixSettings.setControllerDeadband(Constants.CONTROLLER_DEADBAND);
    CiprixSettings.setDefaultAlliance(Constants.DEFAULT_ALLIANCE);
    CiprixSettings.setFieldSize(Constants.FIELD_LENGTH_METERS, Constants.FIELD_WIDTH_METERS);
    AutoLogOutputManager.addPackage("org.ciprix");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
