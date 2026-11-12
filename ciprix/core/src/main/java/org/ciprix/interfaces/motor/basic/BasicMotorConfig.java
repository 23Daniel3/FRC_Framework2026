package org.ciprix.interfaces.motor.basic;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

/**
 * Configuration for a {@link BasicMotorIO} — everything needed to drive a simple, open-loop motor
 * (CIM, RedLine 775pro, BAG, Mini-CIM, etc.), regardless of which speed controller is behind it
 * (PWM, Talon SRX, SparkMax in brushed mode...).
 *
 * <p>{@link org.ciprix.interfaces.motor.advanced.MotorConfig} extends this with everything only a
 * closed-loop-capable motor needs (PID/FF gains, encoder settings, etc). Fluent setters here are
 * re-declared (covariant return type) in {@code MotorConfig} so chaining keeps working across the
 * hierarchy, e.g. {@code new MotorConfig().currentLimit(...).pid(...)}.
 */
public class BasicMotorConfig {

  /** Motor (stator) current limit. It is what protects the motor and limits the torque. */
  public Current currentLimit = Amps.of(40);

  /**
   * Battery (supply) current limit, for controllers that have one. {@code null} (default) keeps the
   * behavior of each controller: the supply limit follows {@link #currentLimit}.
   */
  public Current supplyCurrentLimit = null;

  public boolean inverted = false;
  public boolean brakeMode = false;

  /**
   * Voltage that a percent output of 1.0 means. The controllers compensate the battery voltage to
   * it, so {@code runPercentOutput(0.5)} applies half of this voltage whether the battery is full
   * or not. Use 10 V to get the same behavior for the whole match.
   */
  public Voltage nominalVoltage = Volts.of(12.0);

  public double minOutput = -1.0;
  public double maxOutput = 1.0;
  public double openLoopRampSeconds = 0.0;

  /**
   * Temperature above which the motor reports the problem "Over temperature". Zero disables the
   * check. Controllers that do not measure temperature never trigger it.
   */
  public Temperature temperatureWarning = Celsius.of(85.0);

  public int leaderMotorID = 0;
  public boolean followerInverted = false;

  public BasicMotorConfig currentLimit(Current amps) {
    this.currentLimit = amps;
    return this;
  }

  /** Sets the battery (supply) current limit, for controllers that have one (Talon FX/FXS). */
  public BasicMotorConfig supplyCurrentLimit(Current amps) {
    this.supplyCurrentLimit = amps;
    return this;
  }

  /** Sets the temperature above which the motor reports a problem. Zero disables the check. */
  public BasicMotorConfig temperatureWarning(Temperature temperature) {
    this.temperatureWarning = temperature;
    return this;
  }

  public BasicMotorConfig inverted(boolean set) {
    this.inverted = set;
    return this;
  }

  /** Sets the motor to actively brake to a stop when neutral/idle. */
  public BasicMotorConfig brakeMode() {
    this.brakeMode = true;
    return this;
  }

  /** Sets the motor to coast freely when neutral/idle (default). */
  public BasicMotorConfig coastMode() {
    this.brakeMode = false;
    return this;
  }

  public BasicMotorConfig outputRange(double min, double max) {
    this.minOutput = min;
    this.maxOutput = max;
    return this;
  }

  public BasicMotorConfig nominalVoltage(Voltage v) {
    this.nominalVoltage = v;
    return this;
  }

  /** Seconds to go from 0 to full output. 0 (default) disables ramping. */
  public BasicMotorConfig openLoopRamp(double seconds) {
    this.openLoopRampSeconds = seconds;
    return this;
  }

  /** Marks this motor as a follower of another controller with the given CAN ID. */
  public BasicMotorConfig withMotorLeader(int leaderID) {
    this.leaderMotorID = leaderID;
    return this;
  }

  public BasicMotorConfig withFollowerInverted(boolean inverted) {
    this.followerInverted = inverted;
    return this;
  }

  /**
   * Returns a new config with only the settings of a simple motor (limits, inversion, neutral mode,
   * voltage, ramp, follower) and the full output range. Used when a motor is only the power stage
   * of a software closed loop, which applies the output range itself.
   */
  public BasicMotorConfig copyAsPowerStage() {
    BasicMotorConfig copy = new BasicMotorConfig();
    copy.currentLimit = currentLimit;
    copy.supplyCurrentLimit = supplyCurrentLimit;
    copy.inverted = inverted;
    copy.brakeMode = brakeMode;
    copy.nominalVoltage = nominalVoltage;
    copy.openLoopRampSeconds = openLoopRampSeconds;
    copy.temperatureWarning = temperatureWarning;
    copy.leaderMotorID = leaderMotorID;
    copy.followerInverted = followerInverted;
    return copy;
  }

  public void toLog(String path) {
    Logger.recordOutput(path + "/currentLimitAmps", currentLimit.in(Amps));
    if (supplyCurrentLimit != null) {
      Logger.recordOutput(path + "/supplyCurrentLimitAmps", supplyCurrentLimit.in(Amps));
    }
    Logger.recordOutput(path + "/temperatureWarningCelsius", temperatureWarning.in(Celsius));
    Logger.recordOutput(path + "/inverted", inverted);
    Logger.recordOutput(path + "/brakeMode", brakeMode);
    Logger.recordOutput(path + "/nominalVoltage", nominalVoltage.in(Volts));
    Logger.recordOutput(path + "/outputRange/min", minOutput);
    Logger.recordOutput(path + "/outputRange/max", maxOutput);
    Logger.recordOutput(path + "/openLoopRampSeconds", openLoopRampSeconds);

    if (leaderMotorID != 0) {
      Logger.recordOutput(path + "/follower/leaderID", leaderMotorID);
      Logger.recordOutput(path + "/follower/inverted", followerInverted);
    }
  }
}
