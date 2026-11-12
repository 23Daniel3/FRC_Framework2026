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

  /**
   * Bit mask of the faults that mean a real problem (under voltage, hardware failure, reset during
   * enable, sensor out of phase, API error). Limit switches and soft limits are normal states and
   * are left out.
   */
  static int problemBits(Faults faults) {
    int bits = 0;
    if (faults.UnderVoltage) bits |= 1;
    if (faults.HardwareFailure) bits |= 2;
    if (faults.ResetDuringEn) bits |= 4;
    if (faults.SensorOutOfPhase) bits |= 8;
    if (faults.APIError) bits |= 16;
    return bits;
  }

  /** Text of the faults in {@link #problemBits(Faults)}. */
  static String[] describeProblems(int bits) {
    List<String> messages = new ArrayList<>();
    if ((bits & 1) != 0) messages.add("Fault: Under Voltage");
    if ((bits & 2) != 0) messages.add("Fault: Hardware Failure");
    if ((bits & 4) != 0) messages.add("Fault: Reset During Enable");
    if ((bits & 8) != 0) messages.add("Fault: Sensor Out of Phase");
    if ((bits & 16) != 0) messages.add("Fault: API Error");
    return messages.toArray(new String[0]);
  }
}
