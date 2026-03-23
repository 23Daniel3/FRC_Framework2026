package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface MotorIO {

  public static class MotorIOInputs implements LoggableInputs {
    public Angle position = Rotations.of(0.0);
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public Voltage appliedVolts = Volts.of(0.0);
    public Current current = Amps.of(0.0);
    public Temperature temperature = Celsius.of(0.0);
    public boolean isConnected = false;
    public String[] activeFaults = new String[] {};

    @Override
    public void toLog(LogTable table) {
      table.put("Position", position);
      table.put("Velocity", velocity);
      table.put("AppliedVolts", appliedVolts);
      table.put("Current", current);
      table.put("Temperature", temperature);
      table.put("IsConnected", isConnected);
      table.put("ActiveFaults", activeFaults);
    }

    @Override
    public void fromLog(LogTable table) {
      position = table.get("Position", position);
      velocity = table.get("Velocity", velocity);
      appliedVolts = table.get("AppliedVolts", appliedVolts);
      current = table.get("Current", current);
      temperature = table.get("Temperature", temperature);
      isConnected = table.get("IsConnected", isConnected);
      activeFaults = table.get("ActiveFaults", activeFaults);
    }
  }

  void updateInputs(MotorIOInputs inputs);

  void setBrakeMode(boolean enabled);

  void setOffset(Angle offset);

  void runVoltage(Voltage volts);

  void runPercentOutput(Voltage volts);

  void runVelocity(AngularVelocity velocity);

  void runPosition(Angle position);

  void runSmartPosition(Angle position);

  void stop();

  void applyHardwarePID(int slot, double p, double i, double d);

  void applyHardwareSVAG(int slot, double s, double v, double a, double g);

  void applyHardwareSmartMotion(int slot, double maxVel, double maxAccel, double allowedErr);

  void applyHardwareOutputRange(int slot, double min, double max);

  void setCurrentLimit(Current current);

  MotorIOInputs getMotorIOInputs();
}