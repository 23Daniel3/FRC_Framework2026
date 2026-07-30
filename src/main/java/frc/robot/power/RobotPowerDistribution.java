package frc.robot.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.Logger;

/**
 * Singleton component extending {@link PowerDistribution} to centralize PDH/PDP telemetry readings
 * and enable future power management.
 */
public class RobotPowerDistribution extends PowerDistribution {

  private static RobotPowerDistribution instance;

  private RobotPowerDistribution() {
    super();
  }

  /**
   * Gets the singleton instance of {@link RobotPowerDistribution}.
   *
   * @return unique instance of RobotPowerDistribution
   */
  public static RobotPowerDistribution getInstance() {
    if (instance == null) {
      instance = new RobotPowerDistribution();
    }
    return instance;
  }

  /** Logs PDH/PDP telemetry data to Logger (AdvantageKit). */
  public void log() {
    Logger.recordOutput("Subsystems/PDH/totalCurrent", getTotalCurrent());
    Logger.recordOutput("Subsystems/PDH/voltage", getVoltage());
    Logger.recordOutput("Subsystems/PDH/totalEnergy", getTotalEnergy());
    Logger.recordOutput("Subsystems/PDH/totalPower", getTotalPower());
  }
}
