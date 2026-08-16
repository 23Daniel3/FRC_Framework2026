package frc.lib.interfaces.encoder;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

/**
 * A no-{@code Angle}/{@code AngularVelocity} facade over any {@link EncoderIO}, for subsystems
 * (or students) who just want plain numbers.
 *
 * <p>Wraps the same {@link EncoderIO} you'd otherwise call directly — call {@link #update()} once
 * per loop, then read whichever plain-double getter is convenient:
 *
 * <pre>{@code
 * SimpleEncoder armEncoder = new SimpleEncoder(new EncoderIOThroughBore("Arm", 3, config));
 *
 * // in periodic():
 * armEncoder.update();
 * double degrees = armEncoder.getDegrees();
 * }</pre>
 */
public class SimpleEncoder {

  private final EncoderIO io;
  private final EncoderIO.EncoderIOInputs inputs = new EncoderIO.EncoderIOInputs();

  public SimpleEncoder(EncoderIO io) {
    this.io = io;
  }

  /** Reads the underlying sensor. Call once per loop before using the getters below. */
  public void update() {
    io.updateInputs(inputs);
  }

  public double getRotations() {
    return inputs.position.in(Rotations);
  }

  public double getDegrees() {
    return inputs.position.in(Degrees);
  }

  public double getVelocityRPM() {
    return inputs.velocity.in(RPM);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public String[] getActiveFaults() {
    return inputs.activeFaults;
  }

  /** Zeros the encoder at the current position. */
  public void zero() {
    io.setPosition(Rotations.of(0.0));
  }

  public void setRotations(double rotations) {
    io.setPosition(Rotations.of(rotations));
  }

  public void setDegrees(double degrees) {
    io.setPosition(Degrees.of(degrees));
  }
}