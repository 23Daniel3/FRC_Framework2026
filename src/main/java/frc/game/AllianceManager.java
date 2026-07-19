package frc.game;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.AllianceSelector;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Zones;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AllianceManager {

  private static AllianceManager instance;

  private final LoggedDashboardChooser<Alliance> allianceStartsScoring =
      new LoggedDashboardChooser<>("Alliance Start Selector");

  private double timeUntilNextWindow = -1;
  private double timeRemainingInCurrentWindow = 0;
  private boolean isInsideShootingWindow = false;

  private AllianceManager() {
    allianceStartsScoring.addDefaultOption("Red Starts", Alliance.Red);
    allianceStartsScoring.addOption("Blue Starts", Alliance.Blue);
  }

  public static AllianceManager getInstance() {
    if (instance == null) {
      instance = new AllianceManager();
    }
    return instance;
  }

  public Alliance getStartAllianceScoringIfNotPresent() {
    return allianceStartsScoring.get();
  }

  public Alliance myAlliance() {
    return AllianceSelector.getInstance().getResolvedAlliance();
  }

  public boolean isBlue() {
    return myAlliance() == Alliance.Blue;
  }

  public boolean isRed() {
    return myAlliance() == Alliance.Red;
  }

  /** true se as poses/velocidades de campo devem ser espelhadas (aliança Vermelha). */
  public boolean shouldFlip() {
    return isRed();
  }

  public Zones myAllianceZone() {
    if (isBlue()) {
      return Zones.ALLIANCE_BLUE_ZONE;
    }
    return Zones.ALLIANCE_RED_ZONE;
  }

  public void showAllianceMessageOnDashboard() {
    String gameData = DriverStation.getGameSpecificMessage();
    double matchTime = Timer.getMatchTime();
    boolean isFmsAttached = DriverStation.isFMSAttached();

    Logger.recordOutput("GameInfo/MatchTime", matchTime);
    Logger.recordOutput("GameInfo/GameSpecificMessage", gameData != null ? gameData : "");
    Logger.recordOutput("GameInfo/IsFMSAttached", isFmsAttached);

    Alliance myAlliance = myAlliance();
    Logger.recordOutput("GameInfo/Alliance", myAlliance.toString());

    char dataChar;
    if (gameData != null && gameData.length() > 0) {
      dataChar = gameData.charAt(0);
    } else {
      dataChar = getStartAllianceScoringIfNotPresent() == Alliance.Red ? 'R' : 'B';
    }

    boolean iShootInEvens = false;
    boolean iShootInOdds = false;

    if (myAlliance == Alliance.Red) {
      if (dataChar == 'R') {
        iShootInEvens = true;
      } else if (dataChar == 'B') {
        iShootInOdds = true;
      }
    } else if (myAlliance == Alliance.Blue) {
      if (dataChar == 'B') {
        iShootInEvens = true;
      } else if (dataChar == 'R') {
        iShootInOdds = true;
      }
    }

    Logger.recordOutput("ShootingLogic/MyAllianceChar", myAlliance.toString());
    Logger.recordOutput("ShootingLogic/TargetChar", String.valueOf(dataChar));
    Logger.recordOutput("ShootingLogic/Config/ShootInEvens", iShootInEvens);
    Logger.recordOutput("ShootingLogic/Config/ShootInOdds", iShootInOdds);

    boolean warningActive = false;
    String currentPeriodDebug = "Idle/Other";
    String userMsg = "Aguarde...";

    timeUntilNextWindow = -1;
    timeRemainingInCurrentWindow = 0;
    isInsideShootingWindow = false;

    if (!(Double.isNaN(matchTime) || matchTime < 0.0)) {
      for (int i = 0; i < AllianceConstants.SHIFT_COUNT; i++) {
        double sStart = AllianceConstants.SHIFT_START_TIMES[i];
        double sEnd = AllianceConstants.SHIFT_END_TIMES[i];

        if (matchTime <= sStart && matchTime > sEnd) {
          timeRemainingInCurrentWindow = matchTime - sEnd;
        }

        boolean thisShiftIsActiveForMe =
            ((i % 2 == 0) && iShootInOdds) || ((i % 2 == 1) && iShootInEvens);

        if (!thisShiftIsActiveForMe) {
          continue;
        }

        if (matchTime > sStart) {
          double diff = matchTime - sStart;
          if (timeUntilNextWindow == -1 || diff < timeUntilNextWindow) {
            timeUntilNextWindow = diff;
          }
        } else if (matchTime <= sStart && matchTime > sEnd) {
          isInsideShootingWindow = true;
          timeUntilNextWindow = 0;
        }

        if (matchTime <= (sStart + AllianceConstants.WARNING_WINDOW) && matchTime > sStart) {
          warningActive = true;
          currentPeriodDebug = "Warning_Shift" + (i + 1) + "_BeforeStart";
          userMsg = "ATIRAR!!";
        }

        if (matchTime <= (sEnd + AllianceConstants.WARNING_WINDOW) && matchTime > sEnd) {
          warningActive = true;
          currentPeriodDebug = "Warning_Shift" + (i + 1) + "_BeforeEnd";
          userMsg = "COLETAR!!";
        }
      }
    }

    Logger.recordOutput("ShootingLogic/WarningActive", warningActive);
    Logger.recordOutput("ShootingLogic/DebugPeriod", currentPeriodDebug);

    Logger.recordOutput("Alerta 5 Segundos", warningActive);
    Logger.recordOutput("Status Tiro", userMsg);
    Logger.recordOutput("Tempo Ate Proxima Janela", timeUntilNextWindow);
    Logger.recordOutput("Tempo Restante Janela", timeRemainingInCurrentWindow);

    Logger.recordOutput("ShootingLogic/UserMessage", userMsg);
  }

  public double getTimeUntilNextWindow() {
    return timeUntilNextWindow;
  }

  public double getTimeRemainingInCurrentWindow() {
    return timeRemainingInCurrentWindow;
  }

  public boolean isInsideShootingWindow() {
    return isInsideShootingWindow;
  }

  public boolean isInAllianceZone(Zones currentZone) {
    return currentZone == myAllianceZone();
  }

  public Command setRedStartsScoring() {
    return new InstantCommand(
            () ->
                NetworkTableInstance.getDefault()
                    .getTable("SmartDashboard")
                    .getSubTable("Alliance Start Selector")
                    .getEntry("selected")
                    .setString("Red Starts"))
        .ignoringDisable(true);
  }

  public Command setBlueStartsScoring() {
    return new InstantCommand(
            () ->
                NetworkTableInstance.getDefault()
                    .getTable("SmartDashboard")
                    .getSubTable("Alliance Start Selector")
                    .getEntry("selected")
                    .setString("Blue Starts"))
        .ignoringDisable(true);
  }
}
