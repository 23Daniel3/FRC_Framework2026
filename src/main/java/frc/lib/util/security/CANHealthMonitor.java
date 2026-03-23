package frc.lib.util.security;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Monitors CAN bus health by checking devices in physical connection order.
 *
 * <p>This class does NOT determine actual CAN topology. It assumes the user provides devices
 * ordered according to the physical CAN chain.
 *
 * <p>If a device is disconnected, the monitor logs the most likely failure point (before or between
 * devices).
 */
public class CANHealthMonitor {

  private final List<CANDeviceStatus> devices;
  private final String canName;

  /**
   * Creates a CAN health monitor using the physical connection order.
   *
   * @param devices Devices ordered according to the physical CAN chain
   */
  public CANHealthMonitor(String canName, CANDeviceStatus... devices) {
    this.canName = canName;
    this.devices = Arrays.asList(devices);
  }

  /**
   * Checks CAN bus health and logs results to AdvantageKit.
   *
   * <p>Call this periodically (e.g., in robotPeriodic).
   */
  public void checkHealth() {
    boolean canHealthy = true;
    String faultMessage = "OK";

    for (int i = 0; i < devices.size(); i++) {
      CANDeviceStatus device = devices.get(i);

      if (!device.connected.getAsBoolean()) {
        canHealthy = false;

        if (i == 0) {
          faultMessage = "CAN failure before " + device.deviceName + " (ID " + device.canId + ")";
        } else {
          CANDeviceStatus previous = devices.get(i - 1);
          faultMessage =
              "CAN failure between "
                  + previous.deviceName
                  + " (ID "
                  + previous.canId
                  + ") and "
                  + device.deviceName
                  + " (ID "
                  + device.canId
                  + ")";
        }
        break;
      }
    }

    // AdvantageKit logging
    Logger.recordOutput("CAN/" + canName + "/Healthy/", canHealthy);
    Logger.recordOutput("CAN/" + canName + "/FaultMessage/", faultMessage);

    // Log per-device connectivity
    for (CANDeviceStatus device : devices) {
      Logger.recordOutput(
          "CAN/" + canName + "/Devices/" + device.deviceName + "/Connected",
          device.connected.getAsBoolean());
    }
  }

  /** Represents a device on the CAN bus. */
  public static class CANDeviceStatus {

    public final String deviceName;
    public final int canId;
    public final BooleanSupplier connected;

    public CANDeviceStatus(String deviceName, int canId, BooleanSupplier connected) {
      this.deviceName = deviceName;
      this.canId = canId;
      this.connected = connected;
    }
  }
}
