package org.ciprix.interfaces.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import java.util.ArrayList;
import java.util.List;

/**
 * Translates CTRE Phoenix 6 fault flags into human-readable Strings.
 *
 * <p>The result is a list of strings suitable for driver station dashboards or log files. Shared by
 * every Phoenix 6 motor IO implementation.
 */
public final class CtreMotorFaults {
  private CtreMotorFaults() {}

  /**
   * Retrieves active faults from a CTRE TalonFX motor controller (Phoenix 6).
   *
   * <p>Note: Ensure status signals are refreshed before calling this.
   *
   * @param motor The TalonFX instance to check.
   * @return An array of strings describing the active faults.
   */
  public static String[] getTalonFaults(TalonFX motor) {
    List<String> faults = new ArrayList<>();

    if (Boolean.TRUE.equals(motor.getFault_Hardware().getValue())) faults.add("Hardware Failure");
    if (Boolean.TRUE.equals(motor.getFault_BootDuringEnable().getValue()))
      faults.add("Boot During Enable");

    if (Boolean.TRUE.equals(motor.getFault_Undervoltage().getValue()))
      faults.add("Under Voltage (< 4.5V)");
    if (Boolean.TRUE.equals(motor.getFault_OverSupplyV().getValue()))
      faults.add("Over Supply Voltage");
    if (Boolean.TRUE.equals(motor.getFault_UnstableSupplyV().getValue()))
      faults.add("Unstable Supply V");
    if (Boolean.TRUE.equals(motor.getFault_BridgeBrownout().getValue()))
      faults.add("Bridge Brownout");

    if (Boolean.TRUE.equals(motor.getFault_ProcTemp().getValue()))
      faults.add("Processor Over Temp");
    if (Boolean.TRUE.equals(motor.getFault_DeviceTemp().getValue())) faults.add("Device Over Temp");

    if (Boolean.TRUE.equals(motor.getFault_StatorCurrLimit().getValue()))
      faults.add("Stator Current Limit");
    if (Boolean.TRUE.equals(motor.getFault_SupplyCurrLimit().getValue()))
      faults.add("Supply Current Limit");

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

    if (!motor.isConnected()) {
      faults.add("CRITICAL: CAN DISCONNECTED");
    }

    return faults.toArray(new String[0]);
  }

  /**
   * Retrieves active faults from a CTRE TalonFXS motor controller (Phoenix 6).
   *
   * <p>Note: Ensure status signals are refreshed before calling this.
   *
   * @param motor The TalonFXS instance to check.
   * @return An array of strings describing the active faults.
   */
  public static String[] getTalonFXSFaults(TalonFXS motor) {
    List<String> faults = new ArrayList<>();

    if (Boolean.TRUE.equals(motor.getFault_Hardware().getValue())) faults.add("Hardware Failure");
    if (Boolean.TRUE.equals(motor.getFault_BootDuringEnable().getValue()))
      faults.add("Boot During Enable");

    if (Boolean.TRUE.equals(motor.getFault_Undervoltage().getValue()))
      faults.add("Under Voltage (< 4.5V)");
    if (Boolean.TRUE.equals(motor.getFault_OverSupplyV().getValue()))
      faults.add("Over Supply Voltage");
    if (Boolean.TRUE.equals(motor.getFault_UnstableSupplyV().getValue()))
      faults.add("Unstable Supply V");
    if (Boolean.TRUE.equals(motor.getFault_BridgeBrownout().getValue()))
      faults.add("Bridge Brownout");

    if (Boolean.TRUE.equals(motor.getFault_ProcTemp().getValue()))
      faults.add("Processor Over Temp");
    if (Boolean.TRUE.equals(motor.getFault_DeviceTemp().getValue())) faults.add("Device Over Temp");

    if (Boolean.TRUE.equals(motor.getFault_StatorCurrLimit().getValue()))
      faults.add("Stator Current Limit");
    if (Boolean.TRUE.equals(motor.getFault_SupplyCurrLimit().getValue()))
      faults.add("Supply Current Limit");

    if (Boolean.TRUE.equals(motor.getFault_UnlicensedFeatureInUse().getValue()))
      faults.add("Unlicensed Feature");

    if (!motor.isConnected()) {
      faults.add("CRITICAL: CAN DISCONNECTED");
    }

    return faults.toArray(new String[0]);
  }

  /** Fault reader of a Talon FX, used by {@link MotorIOTalonFX} every cycle. */
  static CtreFaultReader readerFor(TalonFX motor) {
    return new CtreFaultReader(motor.getFaultField())
        .add(motor.getFault_Hardware(), "Hardware Failure")
        .add(motor.getFault_BootDuringEnable(), "Boot During Enable")
        .add(motor.getFault_Undervoltage(), "Under Voltage (< 4.5V)")
        .add(motor.getFault_OverSupplyV(), "Over Supply Voltage")
        .add(motor.getFault_UnstableSupplyV(), "Unstable Supply V")
        .add(motor.getFault_BridgeBrownout(), "Bridge Brownout")
        .add(motor.getFault_ProcTemp(), "Processor Over Temp")
        .add(motor.getFault_DeviceTemp(), "Device Over Temp")
        .add(motor.getFault_FusedSensorOutOfSync(), "Fused Sensor Out of Sync")
        .add(motor.getFault_RemoteSensorDataInvalid(), "Remote Sensor Invalid")
        .add(motor.getFault_RemoteSensorPosOverflow(), "Remote Sensor Overflow")
        .add(motor.getFault_RemoteSensorReset(), "Remote Sensor Reset")
        .add(motor.getFault_MissingDifferentialFX(), "Missing Diff TalonFX")
        .add(motor.getFault_UnlicensedFeatureInUse(), "Unlicensed Feature");
  }

  /** Fault reader of a Talon FXS, used by {@link MotorIOTalonFXS} every cycle. */
  static CtreFaultReader readerFor(TalonFXS motor) {
    return new CtreFaultReader(motor.getFaultField())
        .add(motor.getFault_Hardware(), "Hardware Failure")
        .add(motor.getFault_BootDuringEnable(), "Boot During Enable")
        .add(motor.getFault_Undervoltage(), "Under Voltage (< 4.5V)")
        .add(motor.getFault_OverSupplyV(), "Over Supply Voltage")
        .add(motor.getFault_UnstableSupplyV(), "Unstable Supply V")
        .add(motor.getFault_BridgeBrownout(), "Bridge Brownout")
        .add(motor.getFault_ProcTemp(), "Processor Over Temp")
        .add(motor.getFault_DeviceTemp(), "Device Over Temp")
        .add(motor.getFault_UnlicensedFeatureInUse(), "Unlicensed Feature");
  }
}
