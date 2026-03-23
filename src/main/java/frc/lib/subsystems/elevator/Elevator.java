package frc.lib.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Elevator", inputs);
  }

  public void runVolts(Voltage volts) {
    io.setVoltage(volts);
  }

  public void setPercentOutput(double percentOutput) {
    io.setPercentOutput(percentOutput);
    Logger.recordOutput("Subsystems/Elevator/SetpointPercentOutput", percentOutput);
  }

  public void stop() {
    io.stop();
  }

  public void reset() {
    io.reset();
  }

  public void runPosition(Angle position) {
    if (position.in(Rotations) > ElevatorConstants.ELEVATOR_MAX_POSITION)
      position = Rotations.of(ElevatorConstants.ELEVATOR_MAX_POSITION);
    if (position.in(Rotations) < ElevatorConstants.ELEVATOR_MIN_POSITION)
      position = Rotations.of(ElevatorConstants.ELEVATOR_MIN_POSITION);

    io.runPosition(position);
  }

  @AutoLogOutput(key = "Subsystems/Elevator/AtSetpoint")
  public boolean atSetpoint() {
    return (Math.abs(inputs.motorLeftInputs.velocity.in(RadiansPerSecond)) < 0.1)
        && (Math.abs(inputs.motorRightInputs.velocity.in(RadiansPerSecond)) < 0.1);
  }

  public boolean atZero() {
    return (Math.abs(inputs.motorLeftInputs.position.in(Rotations)) < 15)
        && (Math.abs(inputs.motorRightInputs.position.in(Rotations)) < 15);
  }

  public boolean atLimit() {
    return inputs.motorLeftInputs.current.in(Amps) > 40
        || inputs.motorRightInputs.current.in(Amps) > 40;
  }

  public Angle getPosition() {
    return inputs.motorLeftInputs.position;
  }

  public boolean isConnected() {
    return inputs.motorRightInputs.isConnected && inputs.motorRightInputs.isConnected;
  }

  public MotorIOInputs getMotorLeftInputs() {
    return inputs.motorLeftInputs;
  }

  public MotorIOInputs getMotorRightInputs() {
    return inputs.motorRightInputs;
  }
}
