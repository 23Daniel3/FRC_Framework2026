package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

/**
 * Wraps a Spark Absolute Encoder dataport (on a SparkMax or SparkFlex) into a standalone {@link
 * EncoderIO} view. The parent motor controller's vendor object is passed in so there is exactly one
 * CAN ID on the bus; this class is read-only and never modifies the motor's configuration.
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
    double velRPS = encoder.getVelocity() * sign / 60.0; // RPM → RPS

    return new RawSample(
        Rotations.of(posRot),
        RotationsPerSecond.of(velRPS),
        true,
        true, // Duty-cycle absolute encoders don't expose a disconnected signal via REV API
        new String[] {});
  }
}
