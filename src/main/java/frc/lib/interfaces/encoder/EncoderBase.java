package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.logger.LoggedTunableNumber;
import frc.robot.Constants;

/**
 * Shared base for every {@link EncoderIO} implementation — absolute, relative, CAN-based, or
 * bridged from a motor controller's own feedback.
 *
 * <p>Written once here, and free for every sensor type:
 *
 * <ul>
 *   <li>Software zero offset, live-tunable from the dashboard in tuning mode ({@code
 *       Name/Config/OffsetRot}), exactly like {@code MotorBase}'s tunable output range;
 *   <li>Position/velocity conversion factors;
 *   <li>Numeric velocity derivation with a moving-average filter for sensors that report no
 *       native velocity (e.g. an absolute duty-cycle or analog encoder);
 *   <li>Connection-state debouncing, so one missed CAN/DIO frame doesn't flicker a fault.
 * </ul>
 *
 * <p>Concrete subclasses implement only {@link #readRaw()}: read the sensor, correct for
 * inversion in whatever way is correct for that sensor type, and return a {@link RawSample}. If
 * the sensor supports a native hardware zero (CANcoder, a motor's own feedback), override {@link
 * #setPosition(Angle)} instead of relying on the default software-offset behavior.
 */
public abstract class EncoderBase implements EncoderIO {

  protected final String name;
  protected final EncoderConfig config;
  protected final boolean tuningMode = Constants.tuningMode;

  private final LoggedTunableNumber offsetTunable;
  private final LinearFilter velocityFilter;
  private final Debouncer connectionDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private Angle softwareOffset;
  private Angle lastRawPosition = null;
  private double lastTimestampSeconds = 0.0;

  protected EncoderBase(String name, EncoderConfig config) {
    this.name = name;
    this.config = config;
    this.softwareOffset = config.offset;
    this.offsetTunable =
        new LoggedTunableNumber(name + "/Config/OffsetRot", config.offset.in(Rotations));
    this.velocityFilter = LinearFilter.movingAverage(Math.max(1, config.samplesToAverage));
  }

  /** Result of a single hardware read, before offset/conversion is applied. */
  protected record RawSample(
      Angle position,
      AngularVelocity velocity,
      boolean nativeVelocity,
      boolean isConnected,
      String[] faults) {

    /** For sensors with no native velocity signal — {@link EncoderBase} derives it numerically. */
    public static RawSample withoutVelocity(Angle position, boolean isConnected, String[] faults) {
      return new RawSample(position, RotationsPerSecond.of(0.0), false, isConnected, faults);
    }
  }

  /** Reads the sensor and returns the raw (inversion-corrected, pre-offset) sample. */
  protected abstract RawSample readRaw();

  @Override
  public final void updateInputs(EncoderIOInputs inputs) {
    if (tuningMode && offsetTunable.hasChanged(hashCode())) {
      softwareOffset = Rotations.of(offsetTunable.get());
    }

    RawSample raw = readRaw();

    inputs.absolutePosition = raw.position();
    inputs.position =
        raw.position().minus(softwareOffset).times(config.positionConversionFactor);

    inputs.velocity =
        raw.nativeVelocity()
            ? RotationsPerSecond.of(raw.velocity().in(RotationsPerSecond) * config.velocityConversionFactor)
            : RotationsPerSecond.of(deriveVelocity(raw.position()) * config.velocityConversionFactor);

    inputs.isConnected = connectionDebouncer.calculate(raw.isConnected());
    inputs.activeFaults = raw.faults();
  }

  /**
   * Numerically differentiates raw position for sensors without a native velocity signal, then
   * smooths it with a moving average of {@link EncoderConfig#samplesToAverage} taps.
   *
   * <p>Caveat: a wrapped absolute sensor (e.g. Through Bore in duty-cycle mode) will spike this
   * for one sample when crossing its 0/1 rollover point. If you need clean continuous velocity,
   * prefer {@code EncoderIOQuadrature} (native velocity, no rollover) for that mechanism instead.
   */
  private double deriveVelocity(Angle rawPosition) {
    double now = Timer.getFPGATimestamp();
    double instantaneous = 0.0;

    if (lastRawPosition != null) {
      double dt = now - lastTimestampSeconds;
      if (dt > 1.0e-6) {
        instantaneous = (rawPosition.in(Rotations) - lastRawPosition.in(Rotations)) / dt;
      }
    }

    lastRawPosition = rawPosition;
    lastTimestampSeconds = now;
    return velocityFilter.calculate(instantaneous);
  }

  /**
   * Zeros this encoder by shifting the software offset so the current raw position reports as
   * {@code position}. Sensors with a native hardware zero override this instead — more precise,
   * and it survives brownouts/reboots where a software offset would not.
   */
  @Override
  public void setPosition(Angle position) {
    softwareOffset = readRaw().position().minus(position.div(config.positionConversionFactor));
  }
}