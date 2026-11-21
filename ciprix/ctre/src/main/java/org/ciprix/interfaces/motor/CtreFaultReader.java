package org.ciprix.interfaces.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.List;

/**
 * Reads the faults of a Phoenix 6 device cheaply: the single "fault field" signal (all the flags of
 * the device in one integer) is checked every cycle, and the individual flags are only read and
 * turned into text when that field changes.
 *
 * <p>Flags that only say that a limit is doing its job (stator or supply current limit engaged) are
 * not listed: they are normal while a mechanism accelerates or pushes, and the stall detection of
 * the library already reports a motor that pushes for too long.
 */
final class CtreFaultReader {

  private static final String[] NO_FAULTS = new String[0];

  private final BaseStatusSignal field;
  private final List<StatusSignal<Boolean>> flags = new ArrayList<>();
  private final List<String> labels = new ArrayList<>();
  private BaseStatusSignal[] flagArray = new BaseStatusSignal[0];

  private long lastBits = 0;
  private String[] faults = NO_FAULTS;

  /**
   * @param field the fault field of the device ({@code getFaultField()})
   */
  CtreFaultReader(BaseStatusSignal field) {
    this.field = field;
  }

  /** Adds one flag and the text shown when it is active. */
  CtreFaultReader add(StatusSignal<Boolean> flag, String label) {
    flags.add(flag);
    labels.add(label);
    flagArray = flags.toArray(new BaseStatusSignal[0]);
    return this;
  }

  /** The fault field, to be refreshed together with the other signals of the device. */
  BaseStatusSignal field() {
    return field;
  }

  /** Every signal used, to set their update frequency before {@code optimizeBusUtilization()}. */
  BaseStatusSignal[] allSignals() {
    BaseStatusSignal[] all = new BaseStatusSignal[flagArray.length + 1];
    all[0] = field;
    System.arraycopy(flagArray, 0, all, 1, flagArray.length);
    return all;
  }

  /** Active faults. Call after the fault field was refreshed. */
  String[] read() {
    long bits = (long) field.getValueAsDouble();
    if (bits == 0) {
      lastBits = 0;
      faults = NO_FAULTS;
      return faults;
    }
    if (bits != lastBits) {
      lastBits = bits;
      BaseStatusSignal.refreshAll(flagArray);
      List<String> active = new ArrayList<>();
      for (int i = 0; i < flags.size(); i++) {
        if (Boolean.TRUE.equals(flags.get(i).getValue())) {
          active.add(labels.get(i));
        }
      }
      faults = active.isEmpty() ? NO_FAULTS : active.toArray(new String[0]);
    }
    return faults;
  }
}
