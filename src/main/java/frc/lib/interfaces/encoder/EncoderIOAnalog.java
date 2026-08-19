package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.AnalogEncoder;

/**
 * {@link frc.lib.interfaces.encoder.EncoderIO} for an analog absolute encoder (US Digital MA3,
 * Thrifty Absolute Encoder, ...) wired to an Analog Input.
 *
 * <p>No native velocity — derived numerically by {@link EncoderBase} — and no connection detection
 * is possible on a plain analog voltage signal, so {@code isConnected} always reports {@code true}.
 */
public class EncoderIOAnalog extends EncoderBase {

  private final AnalogEncoder encoder;

  public EncoderIOAnalog(String name, int analogChannel, EncoderConfig config) {
    super(name, config);
    this.encoder = new AnalogEncoder(analogChannel);
    this.encoder.setInverted(config.inverted);
  }

  @Override
  protected RawSample readRaw() {
    return RawSample.withoutVelocity(Rotations.of(encoder.get()), true, new String[] {});
  }
}
