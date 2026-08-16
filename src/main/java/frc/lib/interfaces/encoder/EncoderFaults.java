package frc.lib.interfaces.encoder;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.ArrayList;
import java.util.List;

/** Utility class to generate fault strings for Encoders. */
public class EncoderFaults {

  /** Checks for faults in a DutyCycleEncoder (Through Bore connected to DIO). */
  public static String[] getDutyCycleFaults(DutyCycleEncoder encoder) {
    List<String> faults = new ArrayList<>();

    // Check connection frequency
    // A disconnected absolute encoder usually reads frequency 0 or doesn't update.
    if (!encoder.isConnected()) {
      faults.add("CRITICAL: Encoder Disconnected");
    }

    // Check if the frequency is within the Through Bore spec (approx 976Hz)
    // We allow a margin of error.
    if (encoder.isConnected() && encoder.getFrequency() < 10) {
      faults.add("Warn: Low Frequency Signal");
    }

    return faults.toArray(new String[0]);
  }

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