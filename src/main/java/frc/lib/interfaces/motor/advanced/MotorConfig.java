package frc.lib.interfaces.motor.advanced;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;
import org.littletonrobotics.junction.Logger;

/**
 * Configuration for closed-loop-capable motors (SparkFlex, TalonFX, ...). Extends {@link
 * BasicMotorConfig} with encoder settings, soft limits, and PID/FF/MAXMotion gains across up to 4
 * slots.
 *
 * <p>The basic fluent setters are re-declared here with a covariant return type so calls chain
 * naturally regardless of order, e.g. {@code new MotorConfig().brakeMode().pid(0, 1, 0, 0)}.
 */
public class MotorConfig extends BasicMotorConfig {

  public double positionConversionFactor = 1.0;
  public double velocityConversionFactor = 1.0;

  public Angle positionTolerance = Rotations.of(0);
  public AngularVelocity velocityTolerance = RPM.of(0);

  public boolean softLimitEnabled = false;
  public Angle minPosition = Rotations.of(0.0);
  public Angle maxPosition = Rotations.of(0.0);
  public boolean positionWrap = false;

  public final double[] kP = new double[4];
  public final double[] kI = new double[4];
  public final double[] kD = new double[4];

  public final double[] kS = new double[4];
  public final double[] kV = new double[4];
  public final double[] kA = new double[4];
  public final double[] kG = new double[4];

  public final AngularVelocity[] maxMotionMaxVelocity = new AngularVelocity[4];
  public final AngularAcceleration[] maxMotionMaxAcceleration = new AngularAcceleration[4];
  public final Angle[] maxMotionAllowedClosedLoopError = new Angle[4];

  public enum FeedbackSensorType {
    INTERNAL,
    ALTERNATE,
    ABSOLUTE_DATAPORT
  }

  public Current currentWarningThreshold = Amps.of(0);
  public int currentAverageSamples = 25; // 0.5s at 50Hz

  public boolean stallReversalEnabled = false;
  public Current stallCurrentThreshold = Amps.of(40);
  public double stallTimeSeconds = 0.5;
  public double reversalTimeSeconds = 0.5;
  public double reversalPercentOutput = 0.2;

  public FeedbackSensorType feedbackType = FeedbackSensorType.INTERNAL;
  public int countsPerRevolution = 8192;

  public MotorConfig() {
    for (int i = 0; i < 4; i++) {
      maxMotionMaxVelocity[i] = RotationsPerSecond.of(0.0);
      maxMotionMaxAcceleration[i] = RotationsPerSecondPerSecond.of(0.0);
      maxMotionAllowedClosedLoopError[i] = Rotations.of(0.0);
    }
  }

  public static MotorConfig fromBasic(BasicMotorConfig basic) {
    if (basic instanceof MotorConfig) {
      return (MotorConfig) basic;
    }
    MotorConfig config = new MotorConfig();
    config.currentLimit = basic.currentLimit;
    config.inverted = basic.inverted;
    config.brakeMode = basic.brakeMode;
    config.nominalVoltage = basic.nominalVoltage;
    config.minOutput = basic.minOutput;
    config.maxOutput = basic.maxOutput;
    config.openLoopRampSeconds = basic.openLoopRampSeconds;
    config.leaderMotorID = basic.leaderMotorID;
    config.followerInverted = basic.followerInverted;
    return config;
  }

  // --- Covariant re-declarations of the basic fluent setters (keeps chaining working) ---

  @Override
  public MotorConfig currentLimit(Current amps) {
    super.currentLimit(amps);
    return this;
  }

  @Override
  public MotorConfig inverted(boolean set) {
    super.inverted(set);
    return this;
  }

  @Override
  public MotorConfig brakeMode() {
    super.brakeMode();
    return this;
  }

  @Override
  public MotorConfig coastMode() {
    super.coastMode();
    return this;
  }

  @Override
  public MotorConfig outputRange(double min, double max) {
    super.outputRange(min, max);
    return this;
  }

  @Override
  public MotorConfig nominalVoltage(Voltage v) {
    super.nominalVoltage(v);
    return this;
  }

  @Override
  public MotorConfig openLoopRamp(double seconds) {
    super.openLoopRamp(seconds);
    return this;
  }

  @Override
  public MotorConfig withMotorLeader(int leaderID) {
    super.withMotorLeader(leaderID);
    return this;
  }

  @Override
  public MotorConfig withFollowerInverted(boolean inverted) {
    super.withFollowerInverted(inverted);
    return this;
  }

  // --- Advanced-only fluent setters ---

  public MotorConfig softLimits(Angle min, Angle max) {
    this.softLimitEnabled = true;
    this.minPosition = min;
    this.maxPosition = max;
    return this;
  }

  public MotorConfig conversionFactors(double pos, double vel) {
    this.positionConversionFactor = pos;
    this.velocityConversionFactor = vel;
    return this;
  }

  public MotorConfig pid(int slot, double p, double i, double d) {
    this.kP[slot] = p;
    this.kI[slot] = i;
    this.kD[slot] = d;
    return this;
  }

  public MotorConfig svag(int slot, double s, double v, double a, double g) {
    this.kS[slot] = s;
    this.kV[slot] = v;
    this.kA[slot] = a;
    this.kG[slot] = g;
    return this;
  }

  public MotorConfig smartMotion(
      int slot, AngularVelocity maxVel, AngularAcceleration maxAccel, Angle allowedError) {
    this.maxMotionMaxVelocity[slot] = maxVel;
    this.maxMotionMaxAcceleration[slot] = maxAccel;
    this.maxMotionAllowedClosedLoopError[slot] = allowedError;
    return this;
  }

  public MotorConfig withAlternateEncoder(int cpr) {
    this.feedbackType = FeedbackSensorType.ALTERNATE;
    this.countsPerRevolution = cpr;
    return this;
  }

  public MotorConfig withAbsoluteEncoder() {
    this.feedbackType = FeedbackSensorType.ABSOLUTE_DATAPORT;
    return this;
  }

  public MotorConfig withPositionTolerance(Angle position) {
    this.positionTolerance = position;
    return this;
  }

  public MotorConfig withVelocityTolerance(AngularVelocity velocity) {
    this.velocityTolerance = velocity;
    return this;
  }

  public MotorConfig withCurrentWarning(Current threshold, int samples) {
    this.currentWarningThreshold = threshold;
    this.currentAverageSamples = samples;
    return this;
  }

  public MotorConfig withStallReversal(
      Current stallCurrent, double stallTime, double reverseTime, double reverseOutput) {
    this.stallReversalEnabled = true;
    this.stallCurrentThreshold = stallCurrent;
    this.stallTimeSeconds = stallTime;
    this.reversalTimeSeconds = reverseTime;
    this.reversalPercentOutput = reverseOutput;
    return this;
  }

  @Override
  public void toLog(String path) {
    super.toLog(path);

    Logger.recordOutput(path + "/positionConversionFactor", positionConversionFactor);
    Logger.recordOutput(path + "/velocityConversionFactor", velocityConversionFactor);
    Logger.recordOutput(path + "/positionToleranceRot", positionTolerance.in(Rotations));
    Logger.recordOutput(path + "/velocityToleranceRPM", velocityTolerance.in(RPM));
    Logger.recordOutput(path + "/softLimitEnabled", softLimitEnabled);

    if (softLimitEnabled) {
      Logger.recordOutput(path + "/softLimit/minRot", minPosition.in(Rotations));
      Logger.recordOutput(path + "/softLimit/maxRot", maxPosition.in(Rotations));
    }

    Logger.recordOutput(path + "/positionWrap", positionWrap);
    Logger.recordOutput(path + "/feedbackType", feedbackType.toString());
    Logger.recordOutput(path + "/countsPerRevolution", countsPerRevolution);

    if (currentWarningThreshold.in(Amps) > 0) {
      Logger.recordOutput(path + "/currentWarningThresholdAmps", currentWarningThreshold.in(Amps));
    }

    Logger.recordOutput(path + "/stallReversalEnabled", stallReversalEnabled);
    if (stallReversalEnabled) {
      Logger.recordOutput(
          path + "/stallReversal/stallThresholdAmps", stallCurrentThreshold.in(Amps));
      Logger.recordOutput(path + "/stallReversal/stallTimeSec", stallTimeSeconds);
      Logger.recordOutput(path + "/stallReversal/reverseTimeSec", reversalTimeSeconds);
      Logger.recordOutput(path + "/stallReversal/reverseOutput", reversalPercentOutput);
    }

    for (int i = 0; i < 4; i++) {
      boolean hasGains =
          kP[i] != 0
              || kI[i] != 0
              || kD[i] != 0
              || kS[i] != 0
              || kV[i] != 0
              || kA[i] != 0
              || kG[i] != 0;
      boolean hasMotion =
          maxMotionMaxVelocity[i] != null && maxMotionMaxVelocity[i].in(RotationsPerSecond) != 0;

      if (!hasGains && !hasMotion) continue;

      String slot = path + "/slot" + i;

      if (hasGains) {
        Logger.recordOutput(slot + "/kP", kP[i]);
        Logger.recordOutput(slot + "/kI", kI[i]);
        Logger.recordOutput(slot + "/kD", kD[i]);
        Logger.recordOutput(slot + "/kS", kS[i]);
        Logger.recordOutput(slot + "/kV", kV[i]);
        Logger.recordOutput(slot + "/kA", kA[i]);
        Logger.recordOutput(slot + "/kG", kG[i]);
      }

      if (hasMotion && maxMotionMaxVelocity[i] != null) {
        Logger.recordOutput(slot + "/maxVelRPS", maxMotionMaxVelocity[i].in(RotationsPerSecond));
        Logger.recordOutput(
            slot + "/maxAccelRPSS", maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond));
        Logger.recordOutput(
            slot + "/allowedErrorRot", maxMotionAllowedClosedLoopError[i].in(Rotations));
      }
    }
  }
}
