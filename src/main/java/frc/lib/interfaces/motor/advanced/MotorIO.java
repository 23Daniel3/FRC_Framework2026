package frc.lib.interfaces.motor.advanced;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.interfaces.motor.basic.BasicMotorIO;
import org.littletonrobotics.junction.LogTable;

/**
 * Full IO for motors with closed-loop control (SparkFlex, TalonFX, ...). Extends {@link
 * BasicMotorIO}: a concrete implementation like {@code MotorIOSparkFlex} can be handed to code that
 * only asks for a {@link BasicMotorIO} — that code will only see percent/voltage control — or to
 * code that asks for the full {@link MotorIO} — same object, two views, one implementation.
 */
public interface MotorIO extends BasicMotorIO {

  class MotorIOInputs extends BasicMotorIOInputs {
    public Angle position = Rotations.of(0.0);
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public boolean atSetpoint = false;

    @Override
    public void toLog(LogTable table) {
      super.toLog(table);
      table.put("Position", position);
      table.put("Velocity", velocity);
      table.put("AtSetpoint", atSetpoint);
    }

    @Override
    public void fromLog(LogTable table) {
      super.fromLog(table);
      position = table.get("Position", position);
      velocity = table.get("Velocity", velocity);
      atSetpoint = table.get("AtSetpoint", atSetpoint);
    }
  }

  void updateInputs(MotorIOInputs inputs);

  /**
   * Zeros the motor's feedback sensor by treating the current physical position as {@code offset}.
   * Delegates to the underlying encoder's zeroing mechanism (software offset or hardware zero,
   * depending on the sensor type). Equivalent to {@link
   * frc.lib.interfaces.encoder.EncoderIO#setPosition} from the motor's perspective.
   */
  void setOffset(Angle offset);

  void runVelocity(AngularVelocity velocity);

  void runPosition(Angle position);

  void runSmartPosition(Angle position);

  void runVelocity(AngularVelocity velocity, int slot);

  void runPosition(Angle position, int slot);

  void runSmartPosition(Angle position, int slot);

  void applyHardwarePID(int slot, double p, double i, double d);

  void applyHardwareSVAG(int slot, double s, double v, double a, double g);

  void applyHardwareSmartMotion(int slot, double maxVel, double maxAccel, double allowedErr);

  void applyHardwareOutputRange(int slot, double min, double max);

  @Override
  MotorIOInputs getMotorIOInputs();

  @Override
  MotorController getMotorController();
}
