package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;

/**
 * Wraps the alternate encoder port of a {@link SparkMax} into a standalone read-only {@link
 * EncoderIO} view. The parent motor controller object is passed in — no new CAN ID is allocated.
 *
 * <p>Velocity is converted from RPM to rad/s (multiplying by {@code π/30}) so that the unit
 * delivered to {@link EncoderBase} matches the convention used by the motor's own internal encoder
 * in every Spark implementation.
 *
 * <p>Note: only SparkMax has a physical alternate-encoder port. SparkFlex does not expose one.
 */
public class EncoderIOSparkAlternate extends EncoderBase {

  private final RelativeEncoder encoder;

  public EncoderIOSparkAlternate(String name, EncoderConfig config, SparkMax motorController) {
    super(name, config);
    this.encoder = motorController.getAlternateEncoder();
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
        true, // REV doesn't expose alternate-encoder disconnected state
        new String[] {});
  }

  @Override
  public void setPosition(Angle position) {
    double sign = config.inverted ? -1.0 : 1.0;
    encoder.setPosition(position.in(Rotations) * sign);
  }
}
