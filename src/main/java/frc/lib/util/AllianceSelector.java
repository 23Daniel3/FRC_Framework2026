package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Gerenciador de Aliança com suporte a logs e determinismo. */
public class AllianceSelector {
  private static AllianceSelector instance;
  private final LoggedDashboardChooser<Alliance> chooser;

  private AllianceSelector() {
    chooser = new LoggedDashboardChooser<>("Driverstation/Alliance");

    if (Constants.alliance == Alliance.Red) {
      chooser.addDefaultOption("Red", Alliance.Red);
      chooser.addOption("Blue", Alliance.Blue);
    } else {
      chooser.addDefaultOption("Blue", Alliance.Blue);
      chooser.addOption("Red", Alliance.Red);
    }
  }

  /** Retorna a instância única do seletor. */
  public static AllianceSelector getInstance() {
    if (instance == null) {
      instance = new AllianceSelector();
    }
    return instance;
  }

  /** Retorna a aliança selecionada (Garante um valor não nulo). */
  public Alliance getAlliance() {
    return chooser.get();
  }

  /** Atalho para verificar se o robô está no lado Vermelho. */
  public boolean isRed() {
    return getAlliance() == Alliance.Red;
  }

  /** Atalho para verificar se o robô está no lado Azul. */
  public boolean isBlue() {
    return getAlliance() == Alliance.Blue;
  }
}
