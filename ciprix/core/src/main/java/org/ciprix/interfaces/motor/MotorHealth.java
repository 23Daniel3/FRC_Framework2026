package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ciprix.interfaces.motor.advanced.MotorConfig;

/**
 * Decides, every cycle, whether a motor is healthy: average current, stall, automatic un-jam, free
 * spin and temperature.
 *
 * <p>It is pure logic: it receives the measurements and the current time and keeps no reference to
 * hardware, so the same code runs on the robot, in simulation and in unit tests. {@code MotorBase}
 * feeds it and copies the result to the motor inputs, which is what subsystems read:
 *
 * <pre>{@code
 * fsm.state(IntakeState.COLLECT)
 *     .transitionToSecurityState(() -> inputs.rollerMotorInputs.hasProblem);
 * }</pre>
 *
 * <p>What it reports:
 *
 * <ul>
 *   <li><b>Average current</b>: moving average over {@code currentAverageSamples} cycles. Above
 *       {@code currentWarningThreshold} it becomes the problem "High current".
 *   <li><b>Stalled</b>: the motor is being driven, is not moving and draws more than {@code
 *       stallCurrentThreshold} for {@code stallTimeSeconds}. It stays true until the motor runs
 *       normally again or the command is released, so it does not flicker between un-jam attempts.
 *   <li><b>Un-jamming</b>: when the stall reversal is enabled, the motor is driven backwards for
 *       {@code reversalTimeSeconds} and then goes back to what it was doing.
 *   <li><b>Spinning free</b>: the motor is being driven and turns fast while drawing almost no
 *       current for {@code freeSpinTimeSeconds} (broken belt, loose coupler).
 *   <li><b>Over temperature</b>: above {@code temperatureWarning}.
 * </ul>
 */
public final class MotorHealth {

  /** Applied voltage below which the motor is not considered to be driven. */
  public static final double DRIVEN_VOLTS = 0.1;

  private static final int DISCONNECTED = 1;
  private static final int STALLED = 1 << 1;
  private static final int FREE_SPIN = 1 << 2;
  private static final int OVER_TEMPERATURE = 1 << 3;
  private static final int HIGH_CURRENT = 1 << 4;

  // Thresholds, converted once.
  private final double currentWarningAmps;
  private final boolean stallDetection;
  private final boolean unjamEnabled;
  private final double stallCurrentAmps;
  private final double stallVelocityRps;
  private final double stallTimeSeconds;
  private final double unjamSeconds;
  private final double unjamPercent;
  private final boolean freeSpinDetection;
  private final double freeSpinMaxCurrentAmps;
  private final double freeSpinMinVelocityRps;
  private final double freeSpinTimeSeconds;
  private final double temperatureWarningCelsius;

  // Moving average of the current.
  private final double[] samples;
  private int sampleIndex = 0;
  private int sampleCount = 0;
  private double sampleSum = 0.0;

  // State.
  private double averageCurrentAmps = 0.0;
  private boolean highCurrent = false;
  private boolean stalled = false;
  private boolean unjamming = false;
  private double unjamDirection = 1.0;
  private double unjamEndTime = 0.0;
  private int unjamCount = 0;
  private double stallStartTime = Double.NaN;
  private double recoverStartTime = Double.NaN;
  private boolean freeSpinning = false;
  private double freeSpinStartTime = Double.NaN;
  private boolean overTemperature = false;

  private int problemMask = 0;
  private String problem = "";

  public MotorHealth(MotorConfig config) {
    this.currentWarningAmps = config.currentWarningThreshold.in(Amps);
    this.unjamEnabled = config.stallReversalEnabled;
    this.stallDetection = config.stallReversalEnabled || config.stallDetectionEnabled;
    this.stallCurrentAmps = config.stallCurrentThreshold.in(Amps);
    this.stallVelocityRps = Math.abs(config.stallVelocityThreshold.in(RotationsPerSecond));
    this.stallTimeSeconds = Math.max(0.0, config.stallTimeSeconds);
    this.unjamSeconds = Math.max(0.0, config.reversalTimeSeconds);
    this.unjamPercent = Math.abs(config.reversalPercentOutput);
    this.freeSpinDetection = config.freeSpinDetectionEnabled;
    this.freeSpinMaxCurrentAmps = config.freeSpinMaxCurrent.in(Amps);
    this.freeSpinMinVelocityRps = Math.abs(config.freeSpinMinVelocity.in(RotationsPerSecond));
    this.freeSpinTimeSeconds = Math.max(0.0, config.freeSpinTimeSeconds);
    this.temperatureWarningCelsius = config.temperatureWarning.in(Celsius);
    this.samples = new double[Math.max(1, config.currentAverageSamples)];
  }

  /**
   * Evaluates one cycle.
   *
   * @param now current time in seconds
   * @param enabled true while the robot is enabled
   * @param commanded true while the motor has a command other than "stop"
   * @param holdingAtSetpoint true while a position loop is already at its setpoint (holding a
   *     mechanism against gravity is not a stall)
   * @param appliedVolts voltage applied to the motor
   * @param currentAmps measured current
   * @param velocityRps measured velocity, in rotations per second
   * @param temperatureCelsius measured temperature
   * @param connected true while the motor controller is answering
   */
  public void update(
      double now,
      boolean enabled,
      boolean commanded,
      boolean holdingAtSetpoint,
      double appliedVolts,
      double currentAmps,
      double velocityRps,
      double temperatureCelsius,
      boolean connected) {

    updateTemperature(temperatureCelsius);

    if (!enabled) {
      // A disabled robot cannot stall, and old samples must not leak into the next match.
      resetMotionState();
    } else {
      updateAverage(Math.abs(currentAmps));
      boolean driven = commanded && Math.abs(appliedVolts) > DRIVEN_VOLTS;
      updateStall(
          now,
          commanded,
          driven,
          holdingAtSetpoint,
          appliedVolts,
          Math.abs(currentAmps),
          velocityRps);
      updateFreeSpin(now, driven, Math.abs(velocityRps));
    }

    int mask = 0;
    if (!connected) mask |= DISCONNECTED;
    if (stalled) mask |= STALLED;
    if (freeSpinning) mask |= FREE_SPIN;
    if (overTemperature) mask |= OVER_TEMPERATURE;
    if (highCurrent) mask |= HIGH_CURRENT;
    if (mask != problemMask) {
      problemMask = mask;
      problem = describe(mask);
    }
  }

  private void updateTemperature(double temperatureCelsius) {
    if (temperatureWarningCelsius <= 0.0) {
      overTemperature = false;
    } else if (temperatureCelsius > temperatureWarningCelsius) {
      overTemperature = true;
    } else if (temperatureCelsius < temperatureWarningCelsius - 5.0) {
      // 5 degrees of hysteresis so the alert does not flicker around the threshold.
      overTemperature = false;
    }
  }

  private void updateAverage(double currentAmps) {
    if (sampleCount == samples.length) {
      sampleSum -= samples[sampleIndex];
    } else {
      sampleCount++;
    }
    samples[sampleIndex] = currentAmps;
    sampleSum += currentAmps;
    sampleIndex = (sampleIndex + 1) % samples.length;
    averageCurrentAmps = Math.max(0.0, sampleSum / sampleCount);

    if (currentWarningAmps <= 0.0) {
      highCurrent = false;
    } else if (averageCurrentAmps > currentWarningAmps) {
      highCurrent = true;
    } else if (averageCurrentAmps < 0.9 * currentWarningAmps) {
      highCurrent = false;
    }
  }

  private void updateStall(
      double now,
      boolean commanded,
      boolean driven,
      boolean holdingAtSetpoint,
      double appliedVolts,
      double currentAmps,
      double velocityRps) {
    if (!stallDetection) {
      return;
    }

    if (unjamming) {
      // The reversal ends when its time is up, or at once when the command is released.
      if (now >= unjamEndTime || !commanded) {
        unjamming = false;
        stallStartTime = Double.NaN;
        recoverStartTime = Double.NaN;
      } else {
        return;
      }
    }

    boolean trying = driven && !holdingAtSetpoint;
    boolean stuck = Math.abs(velocityRps) < stallVelocityRps;

    if (!trying) {
      // The command was released (or the mechanism arrived): nothing is stalled any more.
      stalled = false;
      unjamCount = 0;
      stallStartTime = Double.NaN;
      recoverStartTime = Double.NaN;
      return;
    }

    if (stuck && currentAmps > stallCurrentAmps) {
      recoverStartTime = Double.NaN;
      if (Double.isNaN(stallStartTime)) {
        stallStartTime = now;
      }
      if (now - stallStartTime >= stallTimeSeconds) {
        stalled = true;
        if (unjamEnabled) {
          unjamming = true;
          unjamCount++;
          unjamEndTime = now + unjamSeconds;
          unjamDirection = appliedVolts > 0.0 ? -1.0 : 1.0;
        }
      }
      return;
    }

    stallStartTime = Double.NaN;
    if (stalled) {
      if (stuck) {
        // Still not moving (the current limit may be hiding the stall current): not recovered.
        recoverStartTime = Double.NaN;
      } else {
        if (Double.isNaN(recoverStartTime)) {
          recoverStartTime = now;
        }
        if (now - recoverStartTime >= stallTimeSeconds) {
          stalled = false;
          unjamCount = 0;
          recoverStartTime = Double.NaN;
        }
      }
    }
  }

  private void updateFreeSpin(double now, boolean driven, double speedRps) {
    if (!freeSpinDetection) {
      return;
    }
    boolean suspicious =
        driven
            && !unjamming
            && speedRps > freeSpinMinVelocityRps
            && averageCurrentAmps < freeSpinMaxCurrentAmps;
    if (!suspicious) {
      freeSpinStartTime = Double.NaN;
      freeSpinning = false;
      return;
    }
    if (Double.isNaN(freeSpinStartTime)) {
      freeSpinStartTime = now;
    }
    if (now - freeSpinStartTime >= freeSpinTimeSeconds) {
      freeSpinning = true;
    }
  }

  private void resetMotionState() {
    sampleIndex = 0;
    sampleCount = 0;
    sampleSum = 0.0;
    averageCurrentAmps = 0.0;
    highCurrent = false;
    stalled = false;
    unjamming = false;
    unjamCount = 0;
    stallStartTime = Double.NaN;
    recoverStartTime = Double.NaN;
    freeSpinning = false;
    freeSpinStartTime = Double.NaN;
  }

  private static String describe(int mask) {
    if (mask == 0) {
      return "";
    }
    StringBuilder text = new StringBuilder();
    append(text, mask, DISCONNECTED, "Disconnected");
    append(text, mask, STALLED, "Stalled");
    append(text, mask, FREE_SPIN, "Spinning free");
    append(text, mask, OVER_TEMPERATURE, "Over temperature");
    append(text, mask, HIGH_CURRENT, "High current");
    return text.toString();
  }

  private static void append(StringBuilder text, int mask, int bit, String label) {
    if ((mask & bit) != 0) {
      if (text.length() > 0) {
        text.append(", ");
      }
      text.append(label);
    }
  }

  /** Moving average of the current, in amps. */
  public double getAverageCurrentAmps() {
    return averageCurrentAmps;
  }

  /** True while the average current is above the configured warning threshold. */
  public boolean isHighCurrent() {
    return highCurrent;
  }

  /** True from the moment a stall is detected until the motor runs normally again. */
  public boolean isStalled() {
    return stalled;
  }

  /** True while the motor must be driven backwards to free the mechanism. */
  public boolean isUnjamming() {
    return unjamming;
  }

  /**
   * Percent output to apply while {@link #isUnjamming()} is true (already signed); zero otherwise.
   */
  public double getUnjamOutput() {
    return unjamming ? unjamDirection * unjamPercent : 0.0;
  }

  /** Un-jam attempts since the motor last ran normally. */
  public int getUnjamCount() {
    return unjamCount;
  }

  /** True while the motor turns fast with almost no load. */
  public boolean isFreeSpinning() {
    return freeSpinning;
  }

  /** True while the temperature is above the configured warning threshold. */
  public boolean isOverTemperature() {
    return overTemperature;
  }

  /** True when something is wrong with the motor. */
  public boolean hasProblem() {
    return problemMask != 0;
  }

  /** What is wrong, for example {@code "Stalled, High current"}; empty when all is well. */
  public String getProblem() {
    return problem;
  }
}
