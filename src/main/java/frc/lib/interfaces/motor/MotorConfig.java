package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.*;

/**
 * Configuration builder class for Motors.
 *
 * <p>Uses WPILib Units for all physical quantities.
 */
public class MotorConfig {

  // --- Hardware Limits ---
  public Current currentLimit = Amps.of(40);
  public boolean inverted = false;
  public IdleMode idleMode = IdleMode.kCoast;
  public Voltage nominalVoltage = Volts.of(12.0);

  public int leaderMotorID = 0;
  public MotorAlignmentValue motorAlignment = MotorAlignmentValue.Aligned;

  // --- Conversion Factors (dimensionless) ---
  public double positionConversionFactor = 1.0;
  public double velocityConversionFactor = 1.0;

  // --- Soft Limits ---
  public boolean softLimitEnabled = false;
  public Angle minPosition = Rotations.of(0.0);
  public Angle maxPosition = Rotations.of(0.0);
  public boolean positionWrap = false;

  // --- PID (Slot 0 Defaults) ---
  public double kP = 0.0;
  public double kI = 0.0;
  public double kD = 0.0;
  public double kF = 0.0;
  public double kIZone = 0.0;

  // --- Smart Motion / MaxMotion (Slot 0 Defaults) ---
  public AngularVelocity maxMotionMaxVelocity = RotationsPerSecond.of(0.0);
  public AngularAcceleration maxMotionMaxAcceleration = RotationsPerSecondPerSecond.of(0.0);
  public Angle maxMotionAllowedClosedLoopError = Rotations.of(0.0);

  // --- Feedback Sensor Configuration ---
  public enum FeedbackSensorType {
    INTERNAL, // Built-in Hall/NEO encoder
    ALTERNATE, // External Quadrature on Data Port
    ABSOLUTE_DATAPORT // External Absolute/DutyCycle on Data Port
  }

  public FeedbackSensorType feedbackType = FeedbackSensorType.INTERNAL;
  public int countsPerRevolution = 8192; // For Alternate Encoder (Through Bore)

  // ---------------------------------------------------------------------------
  // Fluent Configuration API
  // ---------------------------------------------------------------------------

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

  public MotorConfig pid(double p, double i, double d, double f) {
    this.kP = p;
    this.kI = i;
    this.kD = d;
    this.kF = f;
    return this;
  }

  public MotorConfig smartMotion(
      AngularVelocity maxVel, AngularAcceleration maxAccel, Angle allowedError) {
    this.maxMotionMaxVelocity = maxVel;
    this.maxMotionMaxAcceleration = maxAccel;
    this.maxMotionAllowedClosedLoopError = allowedError;
    return this;
  }

  /**
   * Configures the motor to use an Alternate Encoder (Quadrature) on the Data Port.
   *
   * @param cpr Counts per revolution (e.g., 8192 for REV Through Bore).
   */
  public MotorConfig withAlternateEncoder(int cpr) {
    this.feedbackType = FeedbackSensorType.ALTERNATE;
    this.countsPerRevolution = cpr;
    return this;
  }

  /** Configures the motor to use an Absolute Encoder (DutyCycle) on the Data Port. */
  public MotorConfig withAbsoluteEncoder() {
    this.feedbackType = FeedbackSensorType.ABSOLUTE_DATAPORT;
    return this;
  }

  public MotorConfig withMotorLeader(int leaderID) {
    this.leaderMotorID = leaderID;
    return this;
  }

  public MotorConfig withMotorAlignment(MotorAlignmentValue alignmentValue) {
    this.motorAlignment = alignmentValue;
    return this;
  }
}
