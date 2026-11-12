package org.ciprix.interfaces.motor.advanced;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.ciprix.interfaces.encoder.EncoderIO;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;
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
    ABSOLUTE_DATAPORT,
    REMOTE_CANCODER,
    FUSED_CANCODER,
    SYNC_CANCODER
  }

  /**
   * Threshold for triggering a DriverStation warning about high current consumption. 0 disables.
   */
  public Current currentWarningThreshold = Amps.of(0);

  /** Number of loop iterations (usually 20ms each) to use for the moving average calculation. */
  public int currentAverageSamples = 25; // 0.5s at 50Hz

  /** Whether the automatic stall reversal (un-jam) system is enabled. */
  public boolean stallReversalEnabled = false;

  /**
   * Current threshold that defines a stall condition (alongside low velocity and active command).
   */
  public Current stallCurrentThreshold = Amps.of(40);

  /** Time the motor must remain in a stall condition before triggering the reversal. */
  public double stallTimeSeconds = 0.5;

  /** Duration of the forced reverse movement when un-jamming. */
  public double reversalTimeSeconds = 0.5;

  /** Percent output [-1.0, 1.0] applied in the opposite direction during the reversal period. */
  public double reversalPercentOutput = 0.2;

  /**
   * Whether a stall is detected and reported (problem "Stalled") without reversing the motor.
   * Enabling the stall reversal enables the detection too.
   */
  public boolean stallDetectionEnabled = false;

  /** Below this speed, a motor that is being driven is considered not moving. */
  public AngularVelocity stallVelocityThreshold = RPM.of(10);

  /** Whether the free-spin check is enabled (see {@link #withFreeSpinDetection}). */
  public boolean freeSpinDetectionEnabled = false;

  /** Average current below which a fast-turning motor is considered to have no load. */
  public Current freeSpinMaxCurrent = Amps.of(0);

  /** Speed above which the free-spin check applies. */
  public AngularVelocity freeSpinMinVelocity = RPM.of(0);

  /** Time the motor must turn without load before the problem is reported. */
  public double freeSpinTimeSeconds = 0.5;

  /** Seconds to go from 0 to full output in closed loop. 0 (default) disables the ramp. */
  public double closedLoopRampSeconds = 0.0;

  /** How the gravity gain (kG) is applied. */
  public enum GravityType {
    /** Constant: elevators and anything that always pulls the same way. */
    ELEVATOR,
    /** Proportional to the cosine of the position: arms, with zero at the horizontal. */
    ARM
  }

  public GravityType gravityType = GravityType.ELEVATOR;

  public FeedbackSensorType feedbackType = FeedbackSensorType.INTERNAL;
  public int countsPerRevolution = 8192;

  /** CAN id of the remote sensor (CANcoder) used as feedback. -1 when there is none. */
  public int remoteSensorId = -1;

  /** Rotor turns per turn of the remote sensor (used when the sensor is fused with the rotor). */
  public double rotorToSensorRatio = 1.0;

  public EncoderIO externalEncoder = null;
  public FeedbackSensorType externalFusionType = null;

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
    config.supplyCurrentLimit = basic.supplyCurrentLimit;
    config.temperatureWarning = basic.temperatureWarning;
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
  public MotorConfig supplyCurrentLimit(Current amps) {
    super.supplyCurrentLimit(amps);
    return this;
  }

  @Override
  public MotorConfig temperatureWarning(Temperature temperature) {
    super.temperatureWarning(temperature);
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

  public MotorConfig withExternalEncoder(EncoderIO encoder, FeedbackSensorType fusionType) {
    this.externalEncoder = encoder;
    this.externalFusionType = fusionType;
    return this;
  }

  public boolean hasExternalEncoder() {
    return externalEncoder != null;
  }

  /**
   * Uses a CANcoder as the feedback sensor of a Talon FX, read directly by the motor controller (1
   * kHz, no roboRIO in the loop).
   *
   * @param canId CAN id of the CANcoder (same bus as the motor)
   * @param type {@link FeedbackSensorType#REMOTE_CANCODER}, {@link
   *     FeedbackSensorType#FUSED_CANCODER} or {@link FeedbackSensorType#SYNC_CANCODER}
   * @param rotorToSensorRatio rotor turns per turn of the CANcoder (the gear ratio between them)
   */
  public MotorConfig withRemoteCANcoder(
      int canId, FeedbackSensorType type, double rotorToSensorRatio) {
    this.feedbackType = type;
    this.remoteSensorId = canId;
    this.rotorToSensorRatio = rotorToSensorRatio;
    return this;
  }

  /**
   * Treats the position as continuous between {@code min} and {@code max} (a turret that turns
   * freely, a swerve azimuth): the closed loop and {@code atSetpoint} take the shortest way around.
   * Talon FX supports it in hardware only for a range of exactly one rotation.
   */
  public MotorConfig withPositionWrap(Angle min, Angle max) {
    this.positionWrap = true;
    this.minPosition = min;
    this.maxPosition = max;
    return this;
  }

  /** Seconds to go from 0 to full output in closed loop. 0 disables the ramp. */
  public MotorConfig closedLoopRamp(double seconds) {
    this.closedLoopRampSeconds = seconds;
    return this;
  }

  /** Applies kG as an arm (cosine of the position, zero at the horizontal). */
  public MotorConfig armGravity() {
    this.gravityType = GravityType.ARM;
    return this;
  }

  /**
   * Reports the problem "Stalled" when the motor is driven, does not move and draws more than
   * {@code stallCurrent} for {@code stallTime} seconds, without reversing it. Use {@link
   * #withStallReversal} to also free the mechanism automatically.
   */
  public MotorConfig withStallDetection(Current stallCurrent, double stallTime) {
    this.stallDetectionEnabled = true;
    this.stallCurrentThreshold = stallCurrent;
    this.stallTimeSeconds = stallTime;
    return this;
  }

  /**
   * Reports the problem "Spinning free" when the motor is driven and turns faster than {@code
   * minVelocity} while its average current stays below {@code maxCurrent} for {@code seconds}: the
   * signature of a broken belt or a loose coupler. Only use it on mechanisms that always have load
   * while they move (a flywheel at speed draws little current and would trigger it).
   */
  public MotorConfig withFreeSpinDetection(
      Current maxCurrent, AngularVelocity minVelocity, double seconds) {
    this.freeSpinDetectionEnabled = true;
    this.freeSpinMaxCurrent = maxCurrent;
    this.freeSpinMinVelocity = minVelocity;
    this.freeSpinTimeSeconds = seconds;
    return this;
  }

  /**
   * Configures a DriverStation warning when the average current exceeds the given threshold.
   *
   * @param threshold The current limit that triggers a warning.
   * @param samples Number of periodic iterations to average over (e.g. 25 = 0.5s at 50Hz).
   * @return This config instance for chaining.
   */
  public MotorConfig withCurrentWarning(Current threshold, int samples) {
    this.currentWarningThreshold = threshold;
    this.currentAverageSamples = samples;
    return this;
  }

  /**
   * Enables the autonomous Stall Reversal (un-jam) system. If the motor is commanded to move but
   * detects it is stalled (high current, near-zero velocity), it will ignore subsystem commands and
   * force a momentary reverse movement to un-jam itself.
   *
   * @param stallCurrent The current threshold that defines a physical stall.
   * @param stallTime How long the motor must be stalled before un-jamming triggers.
   * @param reverseTime How long to apply the reverse output.
   * @param reverseOutput The percent output to apply in the opposite direction.
   * @return This config instance for chaining.
   */
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

    Logger.recordOutput(path + "/closedLoopRampSeconds", closedLoopRampSeconds);
    Logger.recordOutput(path + "/gravityType", gravityType.toString());
    if (remoteSensorId >= 0) {
      Logger.recordOutput(path + "/remoteSensorId", remoteSensorId);
      Logger.recordOutput(path + "/rotorToSensorRatio", rotorToSensorRatio);
    }
    Logger.recordOutput(path + "/stallDetectionEnabled", stallDetectionEnabled);
    Logger.recordOutput(path + "/freeSpinDetectionEnabled", freeSpinDetectionEnabled);
    if (freeSpinDetectionEnabled) {
      Logger.recordOutput(path + "/freeSpin/maxCurrentAmps", freeSpinMaxCurrent.in(Amps));
      Logger.recordOutput(path + "/freeSpin/minVelocityRPM", freeSpinMinVelocity.in(RPM));
      Logger.recordOutput(path + "/freeSpin/timeSec", freeSpinTimeSeconds);
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
