package org.ciprix.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * A message that stays on the dashboard while a condition is true (hardware disconnected, low
 * battery, ...). It is the persistent counterpart of {@link Report}, which prints a line once.
 *
 * <p>By default it is shown with the WPILib {@code Alert} widget, so it appears in Elastic,
 * AdvantageScope and Shuffleboard and is logged by AdvantageKit. Keeping every persistent alert of
 * the library behind this class has two benefits: unit tests can capture the alerts without
 * NetworkTables ({@link #setBackendFactory(BackendFactory)}), and a future WPILib that changes the
 * alert API only needs this file to change.
 *
 * <p>The widget is created the first time the alert becomes active, and calls that do not change
 * anything cost nothing, so it is safe to call {@link #set(boolean)} every cycle.
 */
public final class PersistentAlert {

  /** How serious the alert is. */
  public enum Level {
    /** Something is broken and needs attention now (red). */
    ERROR,
    /** Something is degraded but the robot still works (yellow). */
    WARNING,
    /** Information only (green). */
    INFO
  }

  /** What actually shows one alert. */
  public interface Backend {
    void set(boolean active);

    void setText(String text);
  }

  /** Creates the object that shows an alert. */
  @FunctionalInterface
  public interface BackendFactory {
    Backend create(String group, String text, Level level);
  }

  /** Default group: the same one WPILib uses when no group is given. */
  public static final String DEFAULT_GROUP = "Alerts";

  private static final BackendFactory DEFAULT_FACTORY =
      (group, text, level) -> {
        AlertType type =
            switch (level) {
              case ERROR -> AlertType.kError;
              case WARNING -> AlertType.kWarning;
              case INFO -> AlertType.kInfo;
            };
        Alert alert = new Alert(group, text, type);
        return new Backend() {
          @Override
          public void set(boolean active) {
            alert.set(active);
          }

          @Override
          public void setText(String newText) {
            alert.setText(newText);
          }
        };
      };

  private static volatile BackendFactory factory = DEFAULT_FACTORY;

  private final String group;
  private final Level level;
  private String text;
  private boolean active = false;
  private Backend backend = null;

  /** Creates an alert in the default group. */
  public PersistentAlert(String text, Level level) {
    this(DEFAULT_GROUP, text, level);
  }

  /**
   * @param group dashboard group (one widget per group)
   * @param text message shown while the alert is active
   * @param level how serious it is
   */
  public PersistentAlert(String group, String text, Level level) {
    this.group = group;
    this.text = text == null ? "" : text;
    this.level = level;
  }

  /** Shows or hides the alert. Does nothing when the state did not change. */
  public void set(boolean newActive) {
    if (newActive == active) {
      return;
    }
    active = newActive;
    if (backend == null) {
      if (!newActive) {
        return;
      }
      backend = factory.create(group, text, level);
    }
    backend.set(newActive);
  }

  /** Changes the message. Does nothing when the text did not change. */
  public void setText(String newText) {
    String value = newText == null ? "" : newText;
    if (value.equals(text)) {
      return;
    }
    text = value;
    if (backend != null) {
      backend.setText(value);
    }
  }

  /** Convenience: sets the text and shows the alert. */
  public void show(String newText) {
    setText(newText);
    set(true);
  }

  /** True while the alert is being shown. */
  public boolean isActive() {
    return active;
  }

  /** Current message. */
  public String getText() {
    return text;
  }

  /** How serious the alert is. */
  public Level getLevel() {
    return level;
  }

  /** Replaces the way alerts are shown (intended for tests). */
  public static void setBackendFactory(BackendFactory newFactory) {
    factory = newFactory == null ? DEFAULT_FACTORY : newFactory;
  }

  /** Restores the WPILib alert widget. */
  public static void resetBackendFactory() {
    factory = DEFAULT_FACTORY;
  }
}
