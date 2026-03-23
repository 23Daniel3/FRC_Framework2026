package frc.lib.subsystems.cradle;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.util.SetpointTracker;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Cradle extends SubsystemBase {

  private final CradleIO io;
  private final CradleIOInputsAutoLogged inputs = new CradleIOInputsAutoLogged();

  private final Trigger sensorTrue;
  private double setpointVelocity;

  public Cradle(CradleIO io) {
    this.io = io;
    sensorTrue = new Trigger(() -> inputs.sensorIsTrue);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Cradle/inputs", inputs);
    Logger.recordOutput("Subsystems/Cradle/SetpointRadPerSec", setpointVelocity);
  }

  public MotorIOInputs getMotorLeftInputs() {
    return inputs.motorLeftInputs;
  }

  public MotorIOInputs getMotorRightInputs() {
    return inputs.motorRightInputs;
  }

  public void setVolts(Voltage volts) {
    io.setVoltage(volts);
  }

  public void setVelocity(AngularVelocity velocity) {
    setpointVelocity = velocity.in(RadiansPerSecond);
    io.setVelocity(RadiansPerSecond.of(setpointVelocity));
  }

  public void setPercentOutput(double percentOutput) {
    io.setPercentOutput(percentOutput);
  }

  public void setInvertedPercentOutput(double percentOutput) {
    io.setInvertPercentOutput(percentOutput);
  }

  public void setStop() {
    io.setStop();
  }

  public Trigger sensorIsTrue() {
    return sensorTrue;
  }

  @AutoLogOutput(key = "Subsystems/Cradle/AtSetpointVelocityRadPerSec")
  public boolean atSetpointVelocity() {
    return SetpointTracker.atSetpoint(
        setpointVelocity,
        CradleConstants.VELOCITY_TOLERANCE_RAD_PER_SEC,
        inputs.motorLeftInputs.velocity.in(RadiansPerSecond));
  }
}
