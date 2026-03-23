package frc.lib.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Arm", inputs);
  }

  public void runVolts(Voltage volts) {
    io.setVoltage(volts);
  }

  public void runVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
    Logger.recordOutput("Subsystems/Arm/Setpoint", velocity);
  }

  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.motorInputs.velocity;
  }

  public void setPercentOutput(double percentOutput) {
    io.setPercentOutput(percentOutput);
  }

  public void stop() {
    io.stop();
  }

  public void setPosition(Angle position) {
    if (position.in(Rotations) > ArmConstants.ARM_MAX_POSITION) {
      position = Rotations.of(ArmConstants.ARM_MAX_POSITION);
    }
    if (position.in(Rotations) < ArmConstants.ARM_MIN_POSITION) {
      position = Rotations.of(ArmConstants.ARM_MIN_POSITION);
    }
    io.runPosition(position);
  }

  public boolean atZero() {
    return (Math.abs(inputs.motorInputs.position.in(Rotations)) < 0.01);
  }

  public Angle getPosition() {
    return inputs.motorInputs.position;
  }

  public boolean atLimit() {
    return inputs.motorInputs.current.in(Amps) > 40;
  }

  public void zero() {
    io.setOffset(Rotations.of(0));
  }

  public boolean isMotorConected() {
    return inputs.motorInputs.isConnected;
  }

  public MotorIOInputs getMotorInputs() {
    return inputs.motorInputs;
  }
}
