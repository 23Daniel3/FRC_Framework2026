package org.ciprix.interfaces.motor;

import com.ctre.phoenix.motorcontrol.Faults;
import java.util.ArrayList;
import java.util.List;

/** Translates CTRE Phoenix 5 fault flags (Talon SRX, Victor SPX) into human-readable Strings. */
public final class Phoenix5MotorFaults {
  private Phoenix5MotorFaults() {}

  /**
   * Retrieves active faults from a CTRE Phoenix 5 device (Talon SRX, Victor SPX).
   *
   * @param motor The BaseTalon (or VictorSPX) instance to check.
   * @return An array of strings describing the active faults.
   */
  public static String[] getTalonSRXFaults(
      com.ctre.phoenix.motorcontrol.can.BaseMotorController motor) {
    List<String> messages = new ArrayList<>();
    Faults faults = new Faults();
    motor.getFaults(faults);

    if (faults.UnderVoltage) messages.add("Fault: Under Voltage");
    if (faults.ForwardLimitSwitch) messages.add("Fault: Forward Limit Switch");
    if (faults.ReverseLimitSwitch) messages.add("Fault: Reverse Limit Switch");
    if (faults.ForwardSoftLimit) messages.add("Fault: Forward Soft Limit");
    if (faults.ReverseSoftLimit) messages.add("Fault: Reverse Soft Limit");
    if (faults.HardwareFailure) messages.add("Fault: Hardware Failure");
    if (faults.ResetDuringEn) messages.add("Fault: Reset During Enable");
    if (faults.SensorOutOfPhase) messages.add("Fault: Sensor Out of Phase");
    if (faults.APIError) messages.add("Fault: API Error");

    if (motor.getBusVoltage() < 1.0) {
      messages.add("CRITICAL: DISCONNECTED / NO POWER");
    }

    return messages.toArray(new String[0]);
  }
}
