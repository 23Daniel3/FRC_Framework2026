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
   * Describes the active faults and warnings of a SPARK, for example {@code "Fault: Over
   * Temperature"} or {@code "Warn: Brownout"}.
   *
   * @param faults active faults ({@code motor.getFaults()})
   * @param warnings active warnings ({@code motor.getWarnings()})
   * @return one line per active flag, empty when there is none
   */
  public static String[] describe(SparkBase.Faults faults, SparkBase.Warnings warnings) {
    List<String> messages = new ArrayList<>();
    if (faults.other) messages.add("Fault: Other");
    if (faults.motorType) messages.add("Fault: Motor Type Mismatch");
    if (faults.sensor) messages.add("Fault: Sensor Error");
    if (faults.can) messages.add("Fault: CAN Error");
    if (faults.temperature) messages.add("Fault: Over Temperature");
    if (faults.gateDriver) messages.add("Fault: Gate Driver");
    if (faults.escEeprom) messages.add("Fault: ESC EEPROM");
    if (faults.firmware) messages.add("Fault: Firmware Error");

    if (warnings.brownout) messages.add("Warn: Brownout");
    if (warnings.overcurrent) messages.add("Warn: Overcurrent");
    if (warnings.escEeprom) messages.add("Warn: ESC EEPROM");
    if (warnings.extEeprom) messages.add("Warn: Ext EEPROM");
    if (warnings.sensor) messages.add("Warn: Sensor Signal");
    if (warnings.stall) messages.add("Warn: Stall Detected");
    if (warnings.hasReset) messages.add("Warn: Motor Reset Detected");
    if (warnings.other) messages.add("Warn: Other");
    return messages.toArray(new String[0]);
  }

  /**
   * Retrieves active faults and warnings from a REV SparkBase motor controller (SparkMax or
   * SparkFlex, brushed or brushless), plus the last API error and a missing supply voltage.
   *
   * @param motor The SparkBase instance to check.
   * @return An array of strings describing the active faults.
   */
  public static String[] getSparkFaults(SparkBase motor) {
    List<String> messages =
        new ArrayList<>(List.of(describe(motor.getFaults(), motor.getWarnings())));

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
