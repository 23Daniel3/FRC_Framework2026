package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Alliance manager with logging support and determinism. */
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

  /** Returns the singleton instance of the selector. */
  public static AllianceSelector getInstance() {
    if (instance == null) {
      instance = new AllianceSelector();
    }
    return instance;
  }

  /** Returns the selected alliance from the dashboard (guarantees a non-null value). */
  public Alliance getAlliance() {
    return chooser.get();
  }

  /**
   * Canonical source of the robot's alliance: uses DriverStation/FMS when connected and falls back
   * to the dashboard selector (whose default comes from {@code Constants.alliance}) otherwise.
   *
   * <p>All code requiring alliance information must go through here (directly in lib, or via {@code
   * AllianceManager} in robot/game code) — never reimplement the fallback.
   */
  public Alliance getResolvedAlliance() {
    return DriverStation.getAlliance().orElseGet(this::getAlliance);
  }

  /** Returns true if the resolved alliance is Red (field flip convention). */
  public boolean shouldFlip() {
    return getResolvedAlliance() == Alliance.Red;
  }

  /** Shortcut to check if the robot is on the Red alliance. */
  public boolean isRed() {
    return getAlliance() == Alliance.Red;
  }

  /** Shortcut to check if the robot is on the Blue alliance. */
  public boolean isBlue() {
    return getAlliance() == Alliance.Blue;
  }
}
