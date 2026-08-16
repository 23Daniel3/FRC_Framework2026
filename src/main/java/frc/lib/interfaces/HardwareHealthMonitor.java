package frc.lib.interfaces;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.PeriodicSystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Singleton that monitors the health of all registered hardware devices (motors, encoders, etc.).
 * It aggregates disconnection status and active faults across the robot, raising WPILib Alerts for
 * the dashboard. It also includes a boot self-test grace period.
 */
public class HardwareHealthMonitor extends PeriodicSystem {

  private static HardwareHealthMonitor instance;

  private record Device(
      String name, BooleanSupplier isConnected, Supplier<String[]> activeFaults) {}

  private final List<Device> devices = new ArrayList<>();
  private final Alert disconnectedAlert;
  private final Alert faultAlert;

  private final Alert bootFailedAlert = new Alert("Boot Self-Test Failed", AlertType.kError);

  private final double bootGracePeriodSeconds = 5.0;
  private final double bootTime = Timer.getFPGATimestamp();
  private boolean passedBootTest = false;

  private HardwareHealthMonitor() {
    super("HardwareHealthMonitor");
    disconnectedAlert = new Alert("Hardware Disconnected", AlertType.kError);
    faultAlert = new Alert("Hardware Faults", AlertType.kWarning);
  }

  public static HardwareHealthMonitor getInstance() {
    if (instance == null) {
      instance = new HardwareHealthMonitor();
    }
    return instance;
  }

  /**
   * Registers a hardware device to be monitored.
   *
   * @param name The name of the device (e.g., "DriveMotorLeft")
   * @param isConnected A supplier returning true if the device is currently communicating.
   * @param activeFaults A supplier returning an array of active fault strings.
   */
  public static void register(
      String name, BooleanSupplier isConnected, Supplier<String[]> activeFaults) {
    getInstance().devices.add(new Device(name, isConnected, activeFaults));
  }

  @Override
  public void periodic() {
    List<String> disconnectedDevices = new ArrayList<>();
    List<String> allFaults = new ArrayList<>();

    boolean allConnected = true;

    for (Device device : devices) {
      if (!device.isConnected.getAsBoolean()) {
        disconnectedDevices.add(device.name);
        allConnected = false;
      }

      String[] faults = device.activeFaults.get();
      if (faults != null && faults.length > 0) {
        for (String fault : faults) {
          allFaults.add(device.name + ": " + fault);
        }
      }
    }

    if (!disconnectedDevices.isEmpty()) {
      disconnectedAlert.setText("Disconnected Devices: " + String.join(", ", disconnectedDevices));
      disconnectedAlert.set(true);
    } else {
      disconnectedAlert.set(false);
    }

    if (!allFaults.isEmpty()) {
      faultAlert.setText("Active Faults:\n" + String.join("\n", allFaults));
      faultAlert.set(true);
    } else {
      faultAlert.set(false);
    }

    // Boot self-test logic
    if (!passedBootTest) {
      if (allConnected) {
        passedBootTest = true;
      } else if (Timer.getFPGATimestamp() - bootTime > bootGracePeriodSeconds) {
        bootFailedAlert.setText(
            "Boot Self-Test Failed: Not all devices connected within grace period. Check: "
                + String.join(", ", disconnectedDevices));
        bootFailedAlert.set(true);
        passedBootTest = true;
      }
    }
  }
}
