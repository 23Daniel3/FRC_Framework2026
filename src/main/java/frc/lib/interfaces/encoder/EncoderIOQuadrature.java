package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;

/**
 * {@link frc.lib.interfaces.encoder.EncoderIO} for a relative quadrature encoder wired to two DIO
 * ports — a Through Bore in relative mode, a US Digital E4T, or any encoder mounted on a
 * CIM/RedLine shaft to give closed-loop feedback to an otherwise sensorless motor (see {@code
 * frc.lib.interfaces.motor.impl.MotorIOTalonSRX}, {@code MotorIOSparkMaxBrushed}, or {@code
 * MotorIOPWM}).
 *
 * <p>Reports native velocity straight from the RoboRIO's hardware counter — no numeric
 * differentiation, and no rollover artifacts, unlike a wrapped absolute sensor.
 */
public class EncoderIOQuadrature extends EncoderBase {

  private final Encoder encoder;

  /**
   * @param channelA DIO channel for the A phase.
   * @param channelB DIO channel for the B phase.
   * @param countsPerRevolution Encoder resolution (counts per shaft revolution) — used to convert
   *     raw pulses to rotations before {@link EncoderConfig#positionConversionFactor} is applied
   *     for any further gearing.
   */
  public EncoderIOQuadrature(
      String name, int channelA, int channelB, int countsPerRevolution, EncoderConfig config) {
    super(name, config);
    this.encoder = new Encoder(channelA, channelB);
    this.encoder.setDistancePerPulse(1.0 / countsPerRevolution);
    this.encoder.setReverseDirection(config.inverted);
  }

  @Override
  protected RawSample readRaw() {
    return new RawSample(
        Rotations.of(encoder.getDistance()),
        RotationsPerSecond.of(encoder.getRate()),
        true,
        true, // no disconnect detection on a raw DIO quadrature encoder
        new String[] {});
  }

  @Override
  public void setPosition(Angle position) {
    encoder.reset();
    super.setPosition(position);
  }
}