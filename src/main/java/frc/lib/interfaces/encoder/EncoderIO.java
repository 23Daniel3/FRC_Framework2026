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
 * etc).
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

  /** Updates the inputs. */
  public default void updateInputs(EncoderIOInputs inputs) {}

  /**
   * Sets the current position as a new offset (zeroing).
   *
   * @param position The current known position.
   */
  public default void setPosition(Angle position) {}
}
