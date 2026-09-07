package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

/**
 * Configuration builder class for Encoders.
 *
 * <p>Uses a fluent API to configure parameters like offsets, inversion, and conversion factors.
 *
 * <p>All physical quantities are represented using WPILib Units. Conversion factors are
 * dimensionless scalars applied to raw rotations.
 *
 * <p>Note: {@code inverted} is interpreted by each {@link EncoderIO} implementation in whatever way
 * is correct for that sensor (sign flip for a relative/CAN sensor, {@code 1 - raw} for a wrapped
 * absolute duty-cycle sensor, hardware sensor-direction config for a CANcoder, etc.) — {@link
 * EncoderBase} deliberately does not reinterpret it, to avoid double-applying it.
 */
public class EncoderConfig {

  /** Scale factor applied to raw rotations to obtain mechanism position. */
  public double positionConversionFactor = 1.0;

  /** Scale factor applied to raw velocity (rot/s) to obtain mechanism velocity. */
  public double velocityConversionFactor = 1.0;

  /** Whether the encoder direction is inverted. */
  public boolean inverted = false;

  /** Absolute offset applied to the raw encoder position. */
  public Angle offset = Rotations.of(0.0);

  /**
   * Number of samples to average for velocity calculation. Only used by sensors with no native
   * velocity signal — {@link EncoderBase} derives velocity numerically and smooths it with a moving
   * average of this width.
   */
  public int samplesToAverage = 1;

  /**
   * Sets the conversion factors.
   *
   * <p>Both factors are dimensionless scalars applied to raw encoder units.
   *
   * @param posRot Scale factor applied to raw rotations.
   * @param velRotPerSec Scale factor applied to raw velocity (rot/s).
   * @return The updated EncoderConfig.
   */
  public EncoderConfig conversionFactors(double posRot, double velRotPerSec) {
    this.positionConversionFactor = posRot;
    this.velocityConversionFactor = velRotPerSec;
    return this;
  }

  /**
   * Sets the encoder inversion.
   *
   * @param set True to invert.
   * @return The updated EncoderConfig.
   */
  public EncoderConfig inverted(boolean set) {
    this.inverted = set;
    return this;
  }

  /**
   * Sets the absolute offset (software zero) in mechanism units.
   *
   * @param offset The desired offset as an {@link Angle}.
   * @return The updated EncoderConfig.
   */
  public EncoderConfig offset(Angle offset) {
    this.offset = offset;
    return this;
  }

  /**
   * Convenience for {@link #offset(Angle)} when you already have the value in rotations.
   *
   * @param rotations Desired offset, in rotations.
   * @return The updated EncoderConfig.
   */
  public EncoderConfig offsetRotations(double rotations) {
    this.offset = Rotations.of(rotations);
    return this;
  }

  /**
   * Sets the number of samples to average (for stability).
   *
   * @param samples Number of samples (default 1).
   * @return The updated EncoderConfig.
   */
  public EncoderConfig averageDepth(int samples) {
    this.samplesToAverage = samples;
    return this;
  }
}
