package frc.lib.interfaces.motor.basic;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Minimal, vendor-agnostic IO layer for simple/brushed motors driven open-loop (CIM, RedLine
 * 775pro, BAG, etc.), through any speed controller — PWM (VictorSPX, TalonSRX, Spark, Jaguar,
 * Victor, Talon) or CAN (Talon SRX, Victor SPX, SparkMax in brushed mode).
 *
 * <p>A concrete implementation of {@link frc.lib.interfaces.motor.advanced.MotorIO} (which extends
 * this interface) can be handed to code that only knows about {@code BasicMotorIO} — that code will
 * only ever see percent/voltage control, current limiting, and basic telemetry, even though the
 * underlying object also supports closed-loop control.
 */
public interface BasicMotorIO {

  class BasicMotorIOInputs implements LoggableInputs {
    public double percentOutput = 0.0;
    public Voltage appliedVolts = Volts.of(0.0);
    public Current current = Amps.of(0.0);
    public Temperature temperature = Celsius.of(0.0);
    public boolean isConnected = false;
    public String[] activeFaults = new String[] {};

    @Override
    public void toLog(LogTable table) {
      table.put("PercentOutput", percentOutput);
      table.put("AppliedVolts", appliedVolts);
      table.put("Current", current);
      table.put("Temperature", temperature);
      table.put("IsConnected", isConnected);
      table.put("ActiveFaults", activeFaults);
    }

    @Override
    public void fromLog(LogTable table) {
      percentOutput = table.get("PercentOutput", percentOutput);
      appliedVolts = table.get("AppliedVolts", appliedVolts);
      current = table.get("Current", current);
      temperature = table.get("Temperature", temperature);
      isConnected = table.get("IsConnected", isConnected);
      activeFaults = table.get("ActiveFaults", activeFaults);
    }
  }

  void updateInputs(BasicMotorIOInputs inputs);

  void setBrakeMode(boolean enabled);

  void runVoltage(Voltage volts);

  void runPercentOutput(double percent);

  void stop();

  void setCurrentLimit(Current current);

  BasicMotorIOInputs getMotorIOInputs();

  BasicMotorController getMotorController();
}
