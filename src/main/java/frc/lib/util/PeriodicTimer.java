package frc.lib.util;

import frc.robot.Constants;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.junction.Logger;

/**
 * Utility to measure the duration of a block (typically the start/end of a subsystem's {@code
 * periodic()} or a command's {@code execute()}) and publish statistics to AdvantageKit in real
 * time.
 *
 * <p>Usage:
 *
 * <pre>
 * // at the very beginning of periodic/execute
 * PeriodicTimer.start("Intake");
 *
 * // ... your periodic work ...
 *
 * // at the very end of periodic/execute
 * PeriodicTimer.stop("Intake");
 * </pre>
 *
 * The utility records the following AdvantageKit keys (per name): - Timing/&lt;name&gt;/last_ms ->
 * last measured duration (ms) - Timing/&lt;name&gt;/avg_ms -> rolling average duration (ms) -
 * Timing/&lt;name&gt;/min_ms -> observed minimum (ms) - Timing/&lt;name&gt;/max_ms -> observed
 * maximum (ms) - Timing/&lt;name&gt;/count -> number of samples recorded
 *
 * <p>Overrun reporting (organized under "Timing/&lt;name&gt;/overruns/"): -
 * Timing/&lt;name&gt;/overruns/count -> total number of overruns observed -
 * Timing/&lt;name&gt;/overruns/last_ms -> last overrun duration (ms) -
 * Timing/&lt;name&gt;/overruns/last_timestamp -> ISO-8601 timestamp of last overrun -
 * Timing/&lt;name&gt;/overruns/entries/&lt;i&gt;/ms -> recent overrun i (0 = most recent) -
 * Timing/&lt;name&gt;/overruns/entries/&lt;i&gt;/timestamp -> timestamp for that overrun
 *
 * <p>Notes: - Designed for main-thread usage (AdvantageKit's Logger.recordOutput is not
 * thread-safe). - Lightweight and safe to call every loop (adds a small amount of work at stop()).
 */
public final class PeriodicTimer {

  // --- configuration ---
  private static final double OVERRUN_THRESHOLD_MS = 20.0;
  private static final int OVERRUN_BUFFER_CAPACITY = 20;

  // Stores start time (ns) for an active timing block
  private static final Map<String, Long> starts = new ConcurrentHashMap<>();

  // Stores last tick time (ns) for tick-based measurement
  private static final Map<String, Long> lastTicks = new ConcurrentHashMap<>();

  // Stores rolling stats per name
  private static final Map<String, Stats> stats = new ConcurrentHashMap<>();

  // Stores overrun tracking per name
  private static final Map<String, OverrunStats> overruns = new ConcurrentHashMap<>();

  // ISO formatter for timestamps (UTC)
  private static final DateTimeFormatter ISO_FMT =
      DateTimeFormatter.ISO_INSTANT.withZone(ZoneOffset.UTC);

  // Prevent instantiation
  private PeriodicTimer() {}

  /**
   * Start timing a named block. Typically put this at the top of periodic() or execute().
   *
   * @param name A short name for the subsystem or command (e.g., "Intake" or "Drive.execute")
   */
  public static void start(String name) {
    if (!Constants.periodicTimer) return;

    if (name == null) {
      throw new IllegalArgumentException("name must not be null");
    }
    starts.put(name, System.nanoTime());
  }

  /**
   * Stop timing the named block. Computes elapsed time and records metrics to AdvantageKit. If
   * start(name) was not called prior, this prints a warning entry to AdvantageKit and returns.
   *
   * @param name The same name passed to {@link #start(String)}
   */
  public static void stop(String name) {
    if (!Constants.periodicTimer) return;

    if (name == null) {
      throw new IllegalArgumentException("name must not be null");
    }

    Long startNs = starts.remove(name);
    if (startNs == null) {
      // No matching start - record a diagnostic field so you can spot misuse in logs
      Logger.recordOutput("Timing/" + name + "/warning", "stop_without_start");
      return;
    }

    long nowNs = System.nanoTime();
    double elapsedMs = (nowNs - startNs) / 1_000_000.0;

    // Update rolling stats (thread-safe updates via computeIfAbsent + synchronized Stats)
    Stats s = stats.computeIfAbsent(name, k -> new Stats());
    s.addSample(elapsedMs);

    // Publish latest basic metrics to AdvantageKit (grouped under "Timing/<name>/")
    Logger.recordOutput("Timing/" + name + "/last_ms", elapsedMs, "ms");
    Logger.recordOutput("Timing/" + name + "/avg_ms", s.getAverage(), "ms");
    Logger.recordOutput("Timing/" + name + "/min_ms", s.getMin(), "ms");
    Logger.recordOutput("Timing/" + name + "/max_ms", s.getMax(), "ms");
    Logger.recordOutput("Timing/" + name + "/count", s.getCount());

    // Overrun handling
    if (elapsedMs > OVERRUN_THRESHOLD_MS) {
      OverrunStats o =
          overruns.computeIfAbsent(name, k -> new OverrunStats(OVERRUN_BUFFER_CAPACITY));
      long tsMillis = System.currentTimeMillis();
      o.addOverrun(elapsedMs, tsMillis);

      // publish overrun summary
      Logger.recordOutput("Timing/" + name + "/overruns/count", o.getTotalCount());
      Logger.recordOutput("Timing/" + name + "/overruns/last_ms", elapsedMs, "ms");
      Logger.recordOutput(
          "Timing/" + name + "/overruns/last_timestamp",
          ISO_FMT.format(Instant.ofEpochMilli(tsMillis)));

      // publish recent entries (0 = most recent). Cap to buffer size to avoid excessive keys.
      OverrunEntry[] recent = o.getRecentEntries();
      for (int i = 0; i < recent.length; i++) {
        // keys: Timing/<name>/overruns/entries/<i>/ms and /timestamp
        String base = "Timing/" + name + "/overruns/entries/" + i + "/";
        Logger.recordOutput(base + "ms", recent[i].ms, "ms");
        Logger.recordOutput(
            base + "timestamp", ISO_FMT.format(Instant.ofEpochMilli(recent[i].timestampMs)));
      }
    }
  }

  /**
   * Tick-based measurement: call this once per loop (e.g., at the start of a subsystem's
   * periodic()). The first call for a given name initializes the timer. Subsequent calls record the
   * elapsed time since the previous tick and publish the same metrics and overrun information as
   * {@link #stop(String)}.
   *
   * @param name the short name for the subsystem or loop
   */
  public static void tick(String name) {
    if (!Constants.periodicTimer) return;

    if (name == null) {
      throw new IllegalArgumentException("name must not be null");
    }

    long nowNs = System.nanoTime();
    Long lastNs = lastTicks.put(name, nowNs);

    if (lastNs == null) {
      // First tick: initialize and return
      return;
    }

    double elapsedMs = (nowNs - lastNs) / 1_000_000.0;

    // Update rolling stats (reuse same Stats container)
    Stats s = stats.computeIfAbsent(name, k -> new Stats());
    s.addSample(elapsedMs);

    // Publish latest basic metrics to AdvantageKit (grouped under "Timing/<name>/")
    Logger.recordOutput("Timing/" + name + "/last_ms", elapsedMs, "ms");
    Logger.recordOutput("Timing/" + name + "/avg_ms", s.getAverage(), "ms");
    Logger.recordOutput("Timing/" + name + "/min_ms", s.getMin(), "ms");
    Logger.recordOutput("Timing/" + name + "/max_ms", s.getMax(), "ms");
    Logger.recordOutput("Timing/" + name + "/count", s.getCount());

    // Overrun handling (same as stop)
    if (elapsedMs > OVERRUN_THRESHOLD_MS) {
      OverrunStats o =
          overruns.computeIfAbsent(name, k -> new OverrunStats(OVERRUN_BUFFER_CAPACITY));
      long tsMillis = System.currentTimeMillis();
      o.addOverrun(elapsedMs, tsMillis);

      // publish overrun summary
      Logger.recordOutput("Timing/" + name + "/overruns/count", o.getTotalCount());
      Logger.recordOutput("Timing/" + name + "/overruns/last_ms", elapsedMs, "ms");
      Logger.recordOutput(
          "Timing/" + name + "/overruns/last_timestamp",
          ISO_FMT.format(Instant.ofEpochMilli(tsMillis)));

      // publish recent entries (0 = most recent). Cap to buffer size to avoid excessive keys.
      OverrunEntry[] recent = o.getRecentEntries();
      for (int i = 0; i < recent.length; i++) {
        // keys: Timing/<name>/overruns/entries/<i>/ms and /timestamp
        String base = "Timing/" + name + "/overruns/entries/" + i + "/";
        Logger.recordOutput(base + "ms", recent[i].ms, "ms");
        Logger.recordOutput(
            base + "timestamp", ISO_FMT.format(Instant.ofEpochMilli(recent[i].timestampMs)));
      }
    }
  }

  /**
   * Reset the aggregated stats and overrun history for a given name.
   *
   * @param name name to reset
   */
  public static void reset(String name) {
    if (!Constants.periodicTimer) return;

    if (name == null) {
      throw new IllegalArgumentException("name must not be null");
    }
    stats.remove(name);
    starts.remove(name);
    lastTicks.remove(name);
    overruns.remove(name);
    Logger.recordOutput("Timing/" + name + "/reset", "true");
  }

  /* --- internal rolling stats container --- */
  private static final class Stats {
    private double totalMs = 0.0;
    private double minMs = Double.POSITIVE_INFINITY;
    private double maxMs = Double.NEGATIVE_INFINITY;
    private long count = 0L;

    synchronized void addSample(double ms) {
      totalMs += ms;
      if (ms < minMs) minMs = ms;
      if (ms > maxMs) maxMs = ms;
      count++;
    }

    synchronized double getAverage() {
      return (count == 0) ? 0.0 : (totalMs / count);
    }

    synchronized double getMin() {
      return (count == 0) ? 0.0 : minMs;
    }

    synchronized double getMax() {
      return (count == 0) ? 0.0 : maxMs;
    }

    synchronized long getCount() {
      return count;
    }
  }

  /* --- Overrun tracking --- */
  private static final class OverrunEntry {
    final double ms;
    final long timestampMs;

    OverrunEntry(double ms, long timestampMs) {
      this.ms = ms;
      this.timestampMs = timestampMs;
    }
  }

  /** Circular buffer that stores the last N overrun events and a total overrun counter. */
  private static final class OverrunStats {
    private final OverrunEntry[] buffer;
    private int head = 0; // next write index
    private boolean bufferFilled = false;
    private long totalCount = 0L;

    OverrunStats(int capacity) {
      if (capacity <= 0) throw new IllegalArgumentException("capacity must be > 0");
      this.buffer = new OverrunEntry[capacity];
    }

    synchronized void addOverrun(double ms, long timestampMs) {
      buffer[head] = new OverrunEntry(ms, timestampMs);
      head = (head + 1) % buffer.length;
      if (head == 0) bufferFilled = true;
      totalCount++;
    }

    /**
     * Returns the most-recent-first array of OverrunEntry. Length equals buffer capacity but unused
     * slots (before the buffer is filled) are trimmed.
     */
    synchronized OverrunEntry[] getRecentEntries() {
      int size = bufferFilled ? buffer.length : head;
      OverrunEntry[] out = new OverrunEntry[size];
      for (int i = 0; i < size; i++) {
        // most recent first: index = head - 1 - i (wrap)
        int idx = head - 1 - i;
        if (idx < 0) idx += buffer.length;
        out[i] = buffer[idx];
      }
      return out;
    }

    synchronized long getTotalCount() {
      return totalCount;
    }
  }
}
