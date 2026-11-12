package frc.robot.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.Logger;

/**
 * Singleton component to centralize PDH/PDP telemetry readings and enable future power management.
 */
public class RobotPowerDistribution {

  private static final RobotPowerDistribution instance = new RobotPowerDistribution();

  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  private RobotPowerDistribution() {}

  /**
   * Gets the singleton instance of {@link RobotPowerDistribution}.
   *
   * @return unique instance of RobotPowerDistribution
   */
  public static RobotPowerDistribution getInstance() {
    return instance;
  }

  /** Logs PDH/PDP telemetry data to Logger (AdvantageKit). */
  public void log() {
    Logger.recordOutput("Subsystems/PDH/totalCurrent", pdh.getTotalCurrent());
    Logger.recordOutput("Subsystems/PDH/voltage", pdh.getVoltage());
    Logger.recordOutput("Subsystems/PDH/totalEnergy", pdh.getTotalEnergy());
    Logger.recordOutput("Subsystems/PDH/totalPower", pdh.getTotalPower());
  }
}
