package frc.lib.interfaces.encoder;

import edu.wpi.first.units.measure.Angle;
import frc.lib.interfaces.motor.advanced.MotorController;
import frc.lib.interfaces.motor.advanced.MotorIO.MotorIOInputs;
import java.util.function.Supplier;

/**
 * Bridges a closed-loop motor's own built-in feedback (a SparkFlex's encoder, a TalonFX's
 * integrated encoder, ...) into the standalone {@link frc.lib.interfaces.encoder.EncoderIO}
 * world, so a subsystem that just wants "an encoder" can treat an internal and an external sensor
 * identically — same {@code EncoderIOInputsAutoLogged}, same {@code setPosition(Angle)}.
 *
 * <p>This does <b>not</b> own or poll hardware itself: it reads whichever {@link MotorIOInputs}
 * object your subsystem already updates each cycle (avoiding a duplicate CAN read), and forwards
 * zeroing to the motor's own {@link MotorController#setOffset(Angle)} — a real hardware zero, not
 * a stacked software offset.
 *
 * <pre>{@code
 * MotorIOSparkFlex armMotor = new MotorIOSparkFlex("Arm", 9, MotorType.kBrushless, armConfig);
 *
 * EncoderIOMotor armEncoder = new EncoderIOMotor(
 *     "ArmEncoder",
 *     armMotor::getMotorIOInputs,
 *     armMotor.getMotorController(),
 *     new EncoderConfig().conversionFactors(1.0 / 60.0, 1.0 / 60.0)); // e.g. 60:1 gearbox
 *
 * // In periodic(), AFTER armMotor.updateInputs(armMotorInputs):
 * armEncoder.updateInputs(armEncoderInputs);
 * }</pre>
 */
public class EncoderIOMotor extends EncoderBase {

  private final Supplier<MotorIOInputs> motorInputs;
  private final MotorController controller;

  public EncoderIOMotor(
      String name,
      Supplier<MotorIOInputs> motorInputs,
      MotorController controller,
      EncoderConfig config) {
    super(name, config);
    this.motorInputs = motorInputs;
    this.controller = controller;
  }

  @Override
  protected RawSample readRaw() {
    MotorIOInputs m = motorInputs.get();
    return new RawSample(m.position, m.velocity, true, m.isConnected, m.activeFaults);
  }

  @Override
  public void setPosition(Angle position) {
    controller.setOffset(position);
  }
}