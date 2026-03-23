package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware abstraction interface for motor controllers (IO Layer).
 *
 * <p>This interface defines standard interactions including PID slots, SmartMotion, and FeedForward
 * control.
 */
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

  /** Updates the input object with the latest hardware data. */
  public default void updateInputs(MotorIOInputs inputs) {}

  // --- Basic Control ---

  /** Sets the motor output percentage (-1.0 to 1.0). */
  public default void setPercentOutput(double percentOutput) {}

  /** Sets the motor output voltage. */
  public default void setVoltage(Voltage volts) {}

  /** Stops the motor immediately. */
  public default void stop() {}

  /** Sets the idle mode (Brake/Coast). */
  public default void setBrakeMode(boolean enabled) {}

  /** Sets the encoder position/offset. */
  public default void setOffset(Angle offset) {}

  /** Gets the absolute encoder position directly (use sparingly). */
  public default Angle getAbsEncoderPosition() {
    return Rotations.of(0.0);
  }

  // --- Closed Loop Control (Simplified) ---

  /** Runs velocity control on Slot 0. */
  public default void runVelocity(AngularVelocity velocity) {
    runVelocity(velocity, 0, Volts.of(0.0));
  }

  /** Runs position control on Slot 0. */
  public default void runPosition(Angle position) {
    runPosition(position, 0, Volts.of(0.0));
  }

  /** Runs SmartMotion (MaxMotion) to a position on Slot 0. */
  public default void runSmartPosition(Angle position) {
    runSmartPosition(position, 0, Volts.of(0.0));
  }

  // --- Closed Loop Control (Advanced with Slots & FeedForward) ---

  /**
   * Runs velocity control.
   *
   * @param velocity Target velocity.
   * @param slotId PID Slot index (0-3).
   * @param arbFF Arbitrary FeedForward voltage.
   */
  public default void runVelocity(AngularVelocity velocity, int slotId, Voltage arbFF) {}

  /**
   * Runs position control.
   *
   * @param position Target position.
   * @param slotId PID Slot index (0-3).
   * @param arbFF Arbitrary FeedForward voltage.
   */
  public default void runPosition(Angle position, int slotId, Voltage arbFF) {}

  /**
   * Runs SmartMotion (MaxMotion) control to a target position.
   *
   * @param position Target position.
   * @param slotId PID Slot index (0-3).
   * @param arbFF Arbitrary FeedForward voltage.
   */
  public default void runSmartPosition(Angle position, int slotId, Voltage arbFF) {}

  // --- Configuration Methods ---

  /**
   * Configures PIDF constants for a specific slot.
   *
   * @param slotId The slot index (0-3).
   * @param kP Proportional gain.
   * @param kI Integral gain.
   * @param kD Derivative gain.
   * @param kF Feed-forward gain.
   */
  public default void configurePIDF(int slotId, double kP, double kI, double kD, double kF) {}

  public default void configureKSVA(int slotId, double kS, double kV, double kA) {}

  public default void setMaxOutputSlot(double maxOutput, int slot) {}

  public default void setCurrentLimit(Current current) {}
  ;

  public default void setVoltageCompensation(Voltage voltage) {}
  ;

  /**
   * Configures SmartMotion (MaxMotion) parameters for a specific slot.
   *
   * @param slotId The slot index (0-3).
   * @param maxVel Maximum velocity.
   * @param maxAccel Maximum acceleration.
   * @param allowedError Allowed closed-loop error.
   */
  public default void configureSmartMotion(
      int slotId, AngularVelocity maxVel, AngularAcceleration maxAccel, Angle allowedError) {}

  /** Helper to get populated inputs. */
  public default MotorIOInputs getMotorIOInputs() {
    MotorIOInputs inputs = new MotorIOInputs();
    updateInputs(inputs);
    return inputs;
  }
}
