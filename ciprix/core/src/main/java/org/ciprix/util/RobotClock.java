package org.ciprix.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

/**
 * Single source of time for the library, in seconds.
 *
 * <p>By default it reads {@link Timer#getFPGATimestamp()}, which AdvantageKit replaces with the
 * logged timestamp, so everything that uses this clock stays deterministic in replay. Unit tests
 * can install a manual source with {@link #setSource(DoubleSupplier)} and step time without the
 * HAL.
 */
public final class RobotClock {
  private static final DoubleSupplier DEFAULT_SOURCE = Timer::getFPGATimestamp;
  private static volatile DoubleSupplier source = DEFAULT_SOURCE;

  private RobotClock() {}

  /** Current time in seconds. */
  public static double now() {
    return source.getAsDouble();
  }

  /** Replaces the time source (intended for tests). */
  public static void setSource(DoubleSupplier newSource) {
    source = newSource == null ? DEFAULT_SOURCE : newSource;
  }

  /** Restores the default FPGA time source. */
  public static void resetSource() {
    source = DEFAULT_SOURCE;
  }
}
