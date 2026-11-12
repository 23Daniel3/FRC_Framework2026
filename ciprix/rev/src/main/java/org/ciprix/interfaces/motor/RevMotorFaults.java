package org.ciprix.interfaces.motor;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.ArrayList;
import java.util.List;

/**
 * Translates REV fault and warning flags into human-readable Strings.
 *
 * <p>Aggregates both Faults (critical errors) and Warnings (operational alerts) into a unified list
 * of strings suitable for driver station dashboards or log files.
 */
public final class RevMotorFaults {
  private RevMotorFaults() {}

  /**
   * Retrieves active faults and warnings from a REV SparkBase motor controller (SparkMax or
   * SparkFlex, brushed or brushless — this call is identical either way).
   *
   * @param motor The SparkBase instance to check.
   * @return An array of strings describing the active faults.
   */
  public static String[] getSparkFaults(SparkBase motor) {
    List<String> messages = new ArrayList<>();

    SparkBase.Faults f = motor.getFaults();
    if (f.other) messages.add("Fault: Other");
    if (f.motorType) messages.add("Fault: Motor Type Mismatch");
    if (f.sensor) messages.add("Fault: Sensor Error");
    if (f.can) messages.add("Fault: CAN Error");
    if (f.temperature) messages.add("Fault: Over Temperature");
    if (f.gateDriver) messages.add("Fault: Gate Driver");
    if (f.escEeprom) messages.add("Fault: ESC EEPROM");
    if (f.firmware) messages.add("Fault: Firmware Error");

    SparkBase.Warnings w = motor.getWarnings();
    if (w.brownout) messages.add("Warn: Brownout");
    if (w.overcurrent) messages.add("Warn: Overcurrent");
    if (w.escEeprom) messages.add("Warn: ESC EEPROM");
    if (w.extEeprom) messages.add("Warn: Ext EEPROM");
    if (w.sensor) messages.add("Warn: Sensor Signal");
    if (w.stall) messages.add("Warn: Stall Detected");
    if (w.hasReset) messages.add("Warn: Motor Reset Detected");
    if (w.other) messages.add("Warn: Other");

    REVLibError lastError = motor.getLastError();
    if (lastError != REVLibError.kOk) {
      messages.add("API Error: " + lastError.name());
    }

    if (motor.getBusVoltage() < 1.0) {
      messages.add("CRITICAL: DISCONNECTED / NO POWER");
    }

    return messages.toArray(new String[0]);
  }
}
