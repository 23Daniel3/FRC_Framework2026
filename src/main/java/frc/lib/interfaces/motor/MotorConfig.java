package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.*;

public class MotorConfig {

  public Current currentLimit = Amps.of(40);
  public boolean inverted = false;
  public IdleMode idleMode = IdleMode.kCoast;
  public Voltage nominalVoltage = Volts.of(12.0);
  public double minOutput = -1.0;
  public double maxOutput = 1.0;

  public int leaderMotorID = 0;
  public boolean followerInverted = false;

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

  public FeedbackSensorType feedbackType = FeedbackSensorType.INTERNAL;
  public int countsPerRevolution = 8192;

  public MotorConfig() {
    for (int i = 0; i < 4; i++) {
      maxMotionMaxVelocity[i] = RotationsPerSecond.of(0.0);
      maxMotionMaxAcceleration[i] = RotationsPerSecondPerSecond.of(0.0);
      maxMotionAllowedClosedLoopError[i] = Rotations.of(0.0);
    }
  }

  public MotorConfig currentLimit(Current amps) {
    this.currentLimit = amps;
    return this;
  }

  public MotorConfig inverted(boolean set) {
    this.inverted = set;
    return this;
  }

  public MotorConfig brakeMode() {
    this.idleMode = IdleMode.kBrake;
    return this;
  }

  public MotorConfig coastMode() {
    this.idleMode = IdleMode.kCoast;
    return this;
  }

  public MotorConfig outputRange(double min, double max) {
    this.minOutput = min;
    this.maxOutput = max;
    return this;
  }

  public MotorConfig nominalVoltage(Voltage v) {
    this.nominalVoltage = v;
    return this;
  }

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

  public MotorConfig withMotorLeader(int leaderID) {
    this.leaderMotorID = leaderID;
    return this;
  }

  public MotorConfig withFollowerInverted(boolean inverted) {
    this.followerInverted = inverted;
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
}
