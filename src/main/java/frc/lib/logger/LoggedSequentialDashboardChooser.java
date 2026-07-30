package frc.lib.logger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.PeriodicSystem;
import java.util.*;
import java.util.function.Predicate;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Sequential chooser with N slots and support for dynamic per-slot filters.
 *
 * <p>- Remains generic (V can be Supplier&lt;Command&gt;, etc). - Publishes each SendableChooser
 * and only replaces it when the set of visible options changes, to avoid publisher spam on
 * NetworkTables.
 *
 * <p>Note: call {@link #periodic()} periodically (e.g., robotPeriodic()).
 *
 * <p>NOTE: public methods were added to allow cascading updates by whoever coordinates multiple
 * choosers (e.g., AutoTrajectories).
 */
public class LoggedSequentialDashboardChooser<V> extends PeriodicSystem {
  private final String keyPrefix;
  private final int numSlots;
  private final SendableChooser<String>[] choosers;
  private final String[] selectedValues;
  private final Map<String, V> options = new LinkedHashMap<>(); // preserves insertion order
  private String defaultOptionKey = null;

  private final Map<Integer, Predicate<String>> slotFilters = new HashMap<>();
  private final List<Set<String>> visibleOptionsBySlot = new ArrayList<>();

  private final LoggableInputs inputs =
      new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
          for (int i = 0; i < numSlots; i++) {
            table.put(keyPrefix + "_" + i, selectedValues[i]);
          }
        }

        @Override
        public void fromLog(LogTable table) {
          for (int i = 0; i < numSlots; i++) {
            selectedValues[i] = table.get(keyPrefix + "_" + i, selectedValues[i]);
          }
        }
      };

  @SuppressWarnings("unchecked")
  public LoggedSequentialDashboardChooser(String keyPrefix, int numSlots) {
    if (numSlots <= 0) throw new IllegalArgumentException("numSlots must be > 0");
    this.keyPrefix = Objects.requireNonNull(keyPrefix);
    this.numSlots = numSlots;
    this.choosers = (SendableChooser<String>[]) new SendableChooser[numSlots];
    this.selectedValues = new String[numSlots];

    for (int i = 0; i < numSlots; i++) {
      choosers[i] = new SendableChooser<>();
      visibleOptionsBySlot.add(new LinkedHashSet<>());
      // publish each chooser ONCE; subsequent updates happen only if the option set changes
      SmartDashboard.putData(keyPrefix + "_" + i, choosers[i]);
    }
  }

  public void addOption(String optionKey, V value) {
    if (optionKey == null) throw new IllegalArgumentException("optionKey cannot be null");
    options.put(optionKey, value);
    // periodic() will recalculate and publish if necessary
  }

  public void addDefaultOption(String optionKey, V value) {
    addOption(optionKey, value);
    defaultOptionKey = optionKey;
  }

  public void setFilter(int slot, Predicate<String> filter) {
    checkSlot(slot);
    if (filter == null) slotFilters.remove(slot);
    else slotFilters.put(slot, filter);
    // periodic() will apply the filter and publish if necessary
  }

  public void clearFilter(int slot) {
    checkSlot(slot);
    slotFilters.remove(slot);
  }

  public int getNumSlots() {
    return numSlots;
  }

  public String getSelectedKey(int slot) {
    checkSlot(slot);
    return selectedValues[slot];
  }

  public List<V> get() {
    List<V> result = new ArrayList<>();
    for (int i = 0; i < numSlots; i++) result.add(options.get(selectedValues[i]));
    return result;
  }

  public V get(int slot) {
    checkSlot(slot);
    return options.get(selectedValues[slot]);
  }

  public SendableChooser<String>[] getSendableChoosers() {
    return choosers;
  }

  /**
   * Must be called periodically (e.g., robotPeriodic()). Order: 1) reads the current selections
   * (chooser.getSelected()) 2) recomputes visible options and replaces the slot's SendableChooser
   * only if the visible set changed 3) processes logger
   *
   * <p>Note: this method preserves historical behavior. For coordination across multiple choosers
   * (cascade), use the public methods {@link #readSelectionsFromDashboard()}, {@link
   * #refreshSlot(int)}, {@link #refreshAllSlots()} and {@link #processInputs()}.
   */
  @Override
  public void periodic() {
    // 1) read current selections from dashboard
    if (!Logger.hasReplaySource()) {
      for (int i = 0; i < numSlots; i++) {
        try {
          selectedValues[i] = choosers[i].getSelected();
        } catch (Exception e) {
          // keep previous value on error
        }
      }
    }

    // 2) update visible options/publications ONLY if changed
    for (int i = 0; i < numSlots; i++) {
      refreshSlotIfNeeded(i);
    }

    // 3) logger
    Logger.processInputs(keyPrefix, inputs);
  }

  /* ---------- novos helpers públicos para coordenação entre choosers ---------- */

  /**
   * Reads the current Dashboard selections into selectedValues[] without changing visible options
   * or publications. Useful for collecting state before executing cascading refreshes.
   */
  public void readSelectionsFromDashboard() {
    if (!Logger.hasReplaySource()) {
      for (int i = 0; i < numSlots; i++) {
        try {
          selectedValues[i] = choosers[i].getSelected();
        } catch (Exception e) {
          // keep previous value on error
        }
      }
    }
  }

  /** Updates only the specified slot (re-publishes if the visible set changed). */
  public void refreshSlot(int slot) {
    checkSlot(slot);
    refreshSlotIfNeeded(slot);
  }

  /** Updates all slots (equivalent to step 2 of periodic). */
  public void refreshAllSlots() {
    for (int i = 0; i < numSlots; i++) {
      refreshSlotIfNeeded(i);
    }
  }

  /** Executes logger processing (equivalent to step 3 of periodic). */
  public void processInputs() {
    Logger.processInputs(keyPrefix, inputs);
  }

  /* ---------- private helpers ---------- */

  private void refreshSlotIfNeeded(int slot) {
    checkSlot(slot);
    Set<String> newVisible = computeVisibleOptions(slot);
    Set<String> oldVisible = visibleOptionsBySlot.get(slot);

    if (Objects.equals(oldVisible, newVisible)) {
      return; // no change -> avoids re-publication
    }

    // build a new chooser containing only the visible options
    SendableChooser<String> newChooser = new SendableChooser<>();
    boolean defaultSet = false;

    if (newVisible.isEmpty()) {
      if (defaultOptionKey != null && options.containsKey(defaultOptionKey)) {
        newChooser.setDefaultOption(defaultOptionKey, defaultOptionKey);
        defaultSet = true;
      }
    } else {
      for (String k : newVisible) {
        if (!defaultSet && defaultOptionKey != null && defaultOptionKey.equals(k)) {
          newChooser.setDefaultOption(k, k);
          defaultSet = true;
        } else {
          newChooser.addOption(k, k);
        }
      }
    }

    // replace the published chooser (publication only happens when there was a change)
    choosers[slot] = newChooser;
    SmartDashboard.putData(keyPrefix + "_" + slot, newChooser);

    // tries to preserve the previous selection if still valid (updates selectedValues)
    String prev = selectedValues[slot];
    if (prev != null && newVisible.contains(prev)) {
      // keep prev
    } else {
      // if prev is no longer valid, fall back to defaultOptionKey (if present) or null
      if (defaultOptionKey != null && newVisible.contains(defaultOptionKey)) {
        selectedValues[slot] = defaultOptionKey;
      } else {
        selectedValues[slot] = null;
      }
    }

    visibleOptionsBySlot.set(slot, newVisible);
  }

  private Set<String> computeVisibleOptions(int slot) {
    Predicate<String> filter = slotFilters.get(slot);
    Set<String> out = new LinkedHashSet<>();
    for (String k : options.keySet()) {
      if (filter == null || filter.test(k)) out.add(k);
    }
    return out;
  }

  private void checkSlot(int slot) {
    if (slot < 0 || slot >= numSlots)
      throw new IndexOutOfBoundsException("slot out of range: " + slot);
  }
}
