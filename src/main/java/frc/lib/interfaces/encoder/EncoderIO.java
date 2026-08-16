package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for Encoders.
 *
 * <p>Provides a standard way to read position and velocity from any sensor (ThroughBore, CANCoder,
 * quadrature, analog, or even a motor controller's own built-in feedback via {@code
 * frc.lib.interfaces.encoder.impl.EncoderIOMotor}).
 *
 * <p>Every real implementation extends {@link EncoderBase} rather than implementing this interface
 * directly — that's where offset handling, conversion factors, numeric velocity derivation, and
 * connection debouncing live, written once and shared by every sensor type.
 */
public interface EncoderIO {

  @AutoLog
  public static class EncoderIOInputs {
    /** The absolute position in mechanism units (accounted for offset). */
    public Angle position = Rotations.of(0.0);

    /** The absolute position without offset. */
    public Angle absolutePosition = Rotations.of(0.0);

    /** The velocity in mechanism units per second. */
    public AngularVelocity velocity = RotationsPerSecond.of(0.0);

    /** Whether the encoder is connected. */
    public boolean isConnected = false;

    /** List of active faults. */
    public String[] activeFaults = new String[] {};
  }

  /**
   * Refreshes {@code inputs} from the sensor hardware.
   *
   * <p>The default empty implementation exists because {@link EncoderIO} is optional: a motor
   * without an external sensor can use {@link EncoderIONone}, which relies on this no-op. Every
   * real sensor extends {@link EncoderBase} whose {@code updateInputs} is {@code final} and
   * overrides this automatically — direct implementors of this interface must override explicitly.
   */
  public default void updateInputs(EncoderIOInputs inputs) {}

  /**
   * Zeros the encoder by treating the current physical position as {@code position}.
   *
   * <p>The base implementation in {@link EncoderBase} applies a software offset. Sensors that
   * support a native hardware zero (e.g. CANcoder, Spark relative encoder) should override this to
   * push the zero to the hardware instead — more precise and survives brownouts.
   *
   * <p>The default empty implementation exists for read-only sensors (e.g. duty-cycle absolute
   * encoders) whose position cannot be programmatically zeroed.
   *
   * @param position The current known mechanism position to treat as the new zero reference.
   */
  public default void setPosition(Angle position) {}
}
