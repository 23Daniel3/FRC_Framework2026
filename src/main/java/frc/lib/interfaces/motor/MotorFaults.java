package frc.lib.interfaces.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility class to translate hardware-specific fault flags into human-readable Strings.
 *
 * <p>This class aggregates both Faults (critical errors) and Warnings (operational alerts) into a
 * unified list of strings suitable for driver station dashboards or log files.
 */
public class MotorFaults {

  /**
   * Retrieves active faults and warnings from a REV SparkBase motor controller.
   *
   * <p>Covers:
   *
   * <ul>
   *   <li>Critical Hardware Faults (Sensor, Gate Driver, Firmware, etc.)
   *   <li>Warnings (Brownout, Stall, Overcurrent, HasReset)
   *   <li>Last API Error (if the last command failed)
   *   <li>Bus Voltage / Disconnection check
   * </ul>
   *
   * * @param motor The SparkBase instance to check.
   *
   * @return An array of strings describing the active faults.
   */
  public static String[] getSparkFaults(SparkBase motor) {
    List<String> messages = new ArrayList<>();

    // 1. Check Critical Faults
    Faults f = motor.getFaults();
    if (f.other) messages.add("Fault: Other");
    if (f.motorType) messages.add("Fault: Motor Type Mismatch");
    if (f.sensor) messages.add("Fault: Sensor Error");
    if (f.can) messages.add("Fault: CAN Error");
    if (f.temperature) messages.add("Fault: Over Temperature");
    if (f.gateDriver) messages.add("Fault: Gate Driver");
    if (f.escEeprom) messages.add("Fault: ESC EEPROM");
    if (f.firmware) messages.add("Fault: Firmware Error");

    // 2. Check Warnings
    Warnings w = motor.getWarnings();
    if (w.brownout) messages.add("Warn: Brownout");
    if (w.overcurrent) messages.add("Warn: Overcurrent");
    if (w.escEeprom) messages.add("Warn: ESC EEPROM");
    if (w.extEeprom) messages.add("Warn: Ext EEPROM");
    if (w.sensor) messages.add("Warn: Sensor Signal");
    if (w.stall) messages.add("Warn: Stall Detected");
    if (w.hasReset) messages.add("Warn: Motor Reset Detected");
    if (w.other) messages.add("Warn: Other");

    // 3. Check Last API Error (REVLibError)
    // Helps detect if configuration calls are failing silently
    REVLibError lastError = motor.getLastError();
    if (lastError != REVLibError.kOk) {
      messages.add("API Error: " + lastError.name());
    }

    // 4. Check Disconnection / Power Loss
    // If the bus voltage is near zero, the CAN frame might be stale or power is cut.
    if (motor.getBusVoltage() < 1.0) {
      messages.add("CRITICAL: DISCONNECTED / NO POWER");
    }

    return messages.toArray(new String[0]);
  }

  /**
   * Retrieves active faults from a CTRE TalonFX motor controller.
   *
   * <p>Checks all available Fault signals in Phoenix 6 API. Note: Ensure status signals are
   * refreshed before calling this. * @param motor The TalonFX instance to check.
   *
   * @return An array of strings describing the active faults.
   */
  public static String[] getTalonFaults(TalonFX motor) {
    List<String> faults = new ArrayList<>();

    // Hardware & Critical
    if (Boolean.TRUE.equals(motor.getFault_Hardware().getValue())) faults.add("Hardware Failure");
    if (Boolean.TRUE.equals(motor.getFault_BootDuringEnable().getValue()))
      faults.add("Boot During Enable");

    // Power & Voltage
    if (Boolean.TRUE.equals(motor.getFault_Undervoltage().getValue()))
      faults.add("Under Voltage (< 4.5V)");
    if (Boolean.TRUE.equals(motor.getFault_OverSupplyV().getValue()))
      faults.add("Over Supply Voltage");
    if (Boolean.TRUE.equals(motor.getFault_UnstableSupplyV().getValue()))
      faults.add("Unstable Supply V");
    if (Boolean.TRUE.equals(motor.getFault_BridgeBrownout().getValue()))
      faults.add("Bridge Brownout");

    // Temperature
    if (Boolean.TRUE.equals(motor.getFault_ProcTemp().getValue()))
      faults.add("Processor Over Temp");
    if (Boolean.TRUE.equals(motor.getFault_DeviceTemp().getValue())) faults.add("Device Over Temp");

    // Limits & Current
    if (Boolean.TRUE.equals(motor.getFault_StatorCurrLimit().getValue()))
      faults.add("Stator Current Limit");
    if (Boolean.TRUE.equals(motor.getFault_SupplyCurrLimit().getValue()))
      faults.add("Supply Current Limit");

    // Sensors & Features
    if (Boolean.TRUE.equals(motor.getFault_FusedSensorOutOfSync().getValue()))
      faults.add("Fused Sensor Out of Sync");
    if (Boolean.TRUE.equals(motor.getFault_RemoteSensorDataInvalid().getValue()))
      faults.add("Remote Sensor Invalid");
    if (Boolean.TRUE.equals(motor.getFault_RemoteSensorPosOverflow().getValue()))
      faults.add("Remote Sensor Overflow");
    if (Boolean.TRUE.equals(motor.getFault_RemoteSensorReset().getValue()))
      faults.add("Remote Sensor Reset");
    if (Boolean.TRUE.equals(motor.getFault_MissingDifferentialFX().getValue()))
      faults.add("Missing Diff TalonFX");
    if (Boolean.TRUE.equals(motor.getFault_UnlicensedFeatureInUse().getValue()))
      faults.add("Unlicensed Feature");

    // Connection Check
    // Using BaseStatusSignal checks or specific connection flags
    if (!motor.isConnected()) {
      faults.add("CRITICAL: CAN DISCONNECTED");
    }

    return faults.toArray(new String[0]);
  }
}
