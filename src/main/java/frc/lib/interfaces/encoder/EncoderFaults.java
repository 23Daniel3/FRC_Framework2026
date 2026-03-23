package frc.lib.interfaces.encoder;

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
}
