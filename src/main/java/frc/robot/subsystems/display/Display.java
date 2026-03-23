package frc.robot.subsystems.display;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import java.util.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem responsible for managing the robot's display output. It handles two main modes:
 * Failures (high priority) and Checklist (user-scrolled data).
 */
public class Display extends SubsystemBase {

  private final DisplayIO io;
  private final DisplayIOInputsAutoLogged inputs = new DisplayIOInputsAutoLogged();

  private final Deque<Runnable> pendingWrites = new ArrayDeque<>();
  private boolean displayDirty = true;

  public enum DisplayMode {
    IDLE,
    CHECKLIST,
    FAILURE
  }

  @AutoLogOutput(key = "Subsystems/Display/Mode")
  private DisplayMode currentMode = DisplayMode.IDLE;

  // Failures set (High Priority)
  private final LinkedHashSet<String> failures = new LinkedHashSet<>();

  // Checklist data map (Label -> Value) using LinkedHashMap to preserve insertion order
  private final Map<String, String> checklistData = new LinkedHashMap<>();
  // Compiled list for scrolling
  private final List<String> checklist = new ArrayList<>();

  private boolean inChecklistMode = false;
  private int scrollerIndex = -1;

  private String line1 = "";
  private String line2 = "";

  /**
   * Creates a new Display subsystem.
   *
   * @param io The IO implementation for the display hardware.
   */
  public Display(DisplayIO io) {
    this.io = io;
    rebuildChecklist();
    resetToIdle();
    setName("Subsystems/Display");
    ConstantsLogger.logConstants(DisplayConstants.class, getName());
  }

  @Override
  public void periodic() {
    PeriodicTimer.tick(getName());
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    if (inputs.buttonPressed) {
      handleButtonPress();
    }

    if (displayDirty) {
      rebuildMessages();
    }

    if (!pendingWrites.isEmpty()) {
      pendingWrites.poll().run();
    }

    logOutputs();
  }

  /**
   * updates or adds a data point to the checklist. The checklist maintains the insertion order.
   *
   * @param label The name of the data (e.g., "Battery").
   * @param value The value to display (e.g., 12.5 or "High"). Objects are converted to Strings.
   */
  public void setChecklistData(String label, Object value) {
    String strValue = value == null ? "N/A" : String.valueOf(value);
    String previous = checklistData.put(label, strValue);

    // Only rebuild if the value actually changed to optimize cycles
    if (!Objects.equals(previous, strValue)) {
      rebuildChecklist();
    }
  }

  private void rebuildChecklist() {
    checklist.clear();
    for (Map.Entry<String, String> entry : checklistData.entrySet()) {
      checklist.add(entry.getKey() + ": " + entry.getValue());
    }
    // If the data updates while user is looking at it, mark dirty to refresh if needed
    if (inChecklistMode) {
      displayDirty = true;
    }
  }

  /**
   * Sets or clears a failure message. Failures take priority over the checklist.
   *
   * @param key The unique identifier for the failure message.
   * @param active True to show the failure, False to remove it.
   */
  public void setFailure(String key, boolean active) {
    boolean changed = active ? failures.add(key) : failures.remove(key);

    if (changed) {
      if (!failures.isEmpty()) {
        inChecklistMode = false;
        scrollerIndex = -1;
      }
      displayDirty = true;
    }
  }

  private void handleButtonPress() {
    // If there are failures, the button does nothing (or could acknowledge, but logic here is
    // blocking)
    if (!failures.isEmpty()) return;

    if (!inChecklistMode) {
      // Enter checklist mode
      inChecklistMode = true;
      scrollerIndex = 0;
      line1 = "Check Item:";
      line2 = checklist.isEmpty() ? "No Data" : checklist.get(0);
      displayDirty = true;
      return;
    }

    // Scroll through checklist
    if (scrollerIndex < checklist.size() - 1) {
      line1 = line2;
      scrollerIndex++;
      line2 = checklist.get(scrollerIndex);
    } else {
      // End of list, go back to Idle
      resetToIdle();
    }

    displayDirty = true;
  }

  private void rebuildMessages() {
    pendingWrites.clear();

    if (!failures.isEmpty()) {
      currentMode = DisplayMode.FAILURE;
      buildFailureMessage();
    } else if (inChecklistMode) {
      currentMode = DisplayMode.CHECKLIST;
      enqueue(() -> io.writeLine1(line1));
      enqueue(() -> io.writeLine2(line2));
    } else {
      resetToIdle();
    }

    displayDirty = false;
  }

  private void buildFailureMessage() {
    if (failures.size() == 1) {
      String f = failures.iterator().next();
      enqueue(() -> io.writeLine1(f));
      enqueue(() -> io.writeLine2(""));
    } else if (failures.size() == 2) {
      Iterator<String> it = failures.iterator();
      enqueue(() -> io.writeLine1(it.next()));
      enqueue(() -> io.writeLine2(it.next()));
    } else {
      enqueue(() -> io.writeContinuous(String.join(" || ", failures)));
    }
  }

  private void enqueue(Runnable r) {
    pendingWrites.add(r);
  }

  private void resetToIdle() {
    inChecklistMode = false;
    scrollerIndex = -1;
    currentMode = DisplayMode.IDLE;
    enqueue(() -> io.writeLine1("Ready"));
    enqueue(() -> io.writeLine2(""));
    displayDirty = false;
  }

  private void logOutputs() {
    Logger.recordOutput("Subsystems/Display/Real/Line1 Real", line1);
    Logger.recordOutput("Subsystems/Display/Real/Line2 Real", line2);

    // State
    Logger.recordOutput("Subsystems/Display/State/CurrentMode", currentMode.toString());
    Logger.recordOutput("Subsystems/Display/State/InChecklistMode", inChecklistMode);

    // Data structures
    Logger.recordOutput("Subsystems/Display/Failures/List", failures.toArray(new String[0]));
    Logger.recordOutput("Subsystems/Display/Checklist/Items", checklist.toArray(new String[0]));
  }
}
