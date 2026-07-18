package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
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

  /** Retorna a aliança selecionada no dashboard (Garante um valor não nulo). */
  public Alliance getAlliance() {
    return chooser.get();
  }

  /**
   * Fonte canônica da aliança do robô: usa o DriverStation/FMS quando conectado e cai para o
   * seletor do dashboard (cujo default vem de {@code Constants.alliance}) caso contrário.
   *
   * <p>Todo código que precisa da aliança deve passar por aqui (diretamente na lib, ou via
   * {@code AllianceManager} no código de robô/jogo) — nunca reimplementar o fallback.
   */
  public Alliance getResolvedAlliance() {
    return DriverStation.getAlliance().orElseGet(this::getAlliance);
  }

  /** true se a aliança resolvida é a Vermelha (convenção de flip do campo). */
  public boolean shouldFlip() {
    return getResolvedAlliance() == Alliance.Red;
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
