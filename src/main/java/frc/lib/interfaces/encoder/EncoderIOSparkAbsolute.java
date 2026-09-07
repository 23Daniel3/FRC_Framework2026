package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

/**
 * Wraps a Spark Absolute Encoder dataport (on a SparkMax or SparkFlex) into a standalone {@link
 * EncoderIO} view. The parent motor controller object is passed in — no new CAN ID is allocated;
 * this class is read-only and never modifies the motor's configuration.
 *
 * <p>Velocity is converted from RPM to rad/s (multiplying by {@code π/30}) so that the unit
 * delivered to {@link EncoderBase} matches the convention used by the motor's own internal encoder
 * in every Spark implementation.
 */
public class EncoderIOSparkAbsolute extends EncoderBase {

  private final AbsoluteEncoder encoder;

  public EncoderIOSparkAbsolute(String name, EncoderConfig config, SparkBase motorController) {
    super(name, config);
    this.encoder = motorController.getAbsoluteEncoder();
  }

  @Override
  protected RawSample readRaw() {
    double sign = config.inverted ? -1.0 : 1.0;
    double posRot = encoder.getPosition() * sign;
    double velRadPerSec = encoder.getVelocity() * sign * (Math.PI / 30.0); // RPM → rad/s

    return new RawSample(
        Rotations.of(posRot),
        RadiansPerSecond.of(velRadPerSec),
        true,
        true, // Duty-cycle absolute encoders don't expose a disconnected signal via REV API
        new String[] {});
  }
}
