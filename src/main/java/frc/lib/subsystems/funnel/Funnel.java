package frc.lib.subsystems.funnel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {

  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  public Funnel(FunnelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Funnel", inputs);
  }

  public void runVolts(Voltage volts) {
    io.setVoltage(volts);
  }

  public void runVelocity(AngularVelocity velocity) {
    io.runVelocity(velocity);
  }

  public void stop() {
    io.stop();
  }

  public void setPercentOutput(double percentOutput) {
    io.setPercentOutput(percentOutput);
  }

  public boolean isConnected() {
    return inputs.motorInputs.isConnected;
  }

  public MotorIOInputs getMotorInputs() {
    return inputs.motorInputs;
  }
}
