package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * {@link frc.lib.interfaces.encoder.EncoderIO} for a REV Through Bore Encoder wired to DIO in PWM
 * absolute mode.
 *
 * <p>Has no native velocity signal — {@link EncoderBase} derives it numerically. If you need clean,
 * rollover-free velocity, wire the Through Bore's A/B quadrature outputs instead and use {@link
 * EncoderIOQuadrature}.
 */
public class EncoderIOThroughBore extends EncoderBase {

  private final DutyCycleEncoder encoder;

  public EncoderIOThroughBore(String name, int dioChannel, EncoderConfig config) {
    super(name, config);
    this.encoder = new DutyCycleEncoder(dioChannel);
    this.encoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0); // REV spec
  }

  @Override
  protected RawSample readRaw() {
    double rawRotations = encoder.get();
    if (config.inverted) {
      rawRotations = 1.0 - rawRotations;
    }

    return RawSample.withoutVelocity(
        Rotations.of(rawRotations),
        encoder.isConnected(),
        EncoderFaults.getDutyCycleFaults(encoder));
  }
}
