package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Implementation of {@link EncoderIO} for the REV Through Bore Encoder.
 *
 * <p>Assumes connection via DIO (PWM Absolute Mode).
 */
public class EncoderIOThroughBore implements EncoderIO {

  private final DutyCycleEncoder encoder;
  private final EncoderConfig config;

  /**
   * @param dioChannel The DIO port on the RoboRIO.
   * @param config Configuration object.
   */
  public EncoderIOThroughBore(int dioChannel, EncoderConfig config) {
    this.encoder = new DutyCycleEncoder(dioChannel);
    this.config = config;
    this.encoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0); // REV Spec

    // Prime inputs (absolute encoder has no velocity)
    updateInputs(new EncoderIOInputs());
  }

  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    inputs.isConnected = encoder.isConnected();

    // Raw absolute position (0.0 - 1.0 rotations)
    double rawRotations = encoder.get();
    if (config.inverted) {
      rawRotations = 1.0 - rawRotations;
    }

    Angle rawAngle = Rotations.of(rawRotations);
    inputs.absolutePosition = rawAngle;

    // Mechanism position:
    // (raw - offset) * conversion factor
    Angle mechanismPosition = rawAngle.minus(config.offset).times(config.positionConversionFactor);

    inputs.position = mechanismPosition;

    // DutyCycleEncoder does not provide native velocity
    inputs.velocity = RotationsPerSecond.of(0.0);

    inputs.activeFaults = EncoderFaults.getDutyCycleFaults(encoder);
  }

  @Override
  public void setPosition(Angle position) {
    // For absolute encoders, setting position updates the software offset
    double rawRotations = encoder.get();
    if (config.inverted) {
      rawRotations = 1.0 - rawRotations;
    }

    Angle rawAngle = Rotations.of(rawRotations);

    // offset = raw - (target / factor)
    Angle newOffset = rawAngle.minus(position.div(config.positionConversionFactor));

    this.config.offset = newOffset;
  }
}
