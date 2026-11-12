package org.ciprix.interfaces.encoder;

import com.ctre.phoenix6.hardware.CANcoder;
import java.util.ArrayList;
import java.util.List;

/** Utility class to generate fault strings for CTRE Phoenix 6 encoders. */
public final class CtreEncoderFaults {
  private CtreEncoderFaults() {}

  /** Checks for faults in a CTRE CANcoder (Phoenix 6). */
  public static String[] getCANcoderFaults(CANcoder encoder) {
    List<String> faults = new ArrayList<>();

    if (Boolean.TRUE.equals(encoder.getFault_Hardware().getValue())) {
      faults.add("Fault: Hardware Failure");
    }
    if (Boolean.TRUE.equals(encoder.getFault_BootDuringEnable().getValue())) {
      faults.add("Fault: Boot During Enable");
    }
    if (Boolean.TRUE.equals(encoder.getFault_Undervoltage().getValue())) {
      faults.add("Fault: Undervoltage");
    }
    if (Boolean.TRUE.equals(encoder.getFault_BadMagnet().getValue())) {
      faults.add("CRITICAL: Bad Magnet / Out of Range");
    }
    if (Boolean.TRUE.equals(encoder.getFault_UnlicensedFeatureInUse().getValue())) {
      faults.add("Fault: Unlicensed Feature In Use");
    }
    if (!encoder.getPosition().getStatus().isOK()) {
      faults.add("CRITICAL: CAN DISCONNECTED");
    }

    return faults.toArray(new String[0]);
  }
}
