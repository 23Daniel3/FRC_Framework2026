package frc.lib.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Wrist", inputs);
  }

  public void setPosition(Angle position) {
    if (position.in(Rotations) > WristConstants.WRIST_MAX_POSITION)
      position = Rotations.of(WristConstants.WRIST_MAX_POSITION);
    if (position.in(Rotations) < WristConstants.WRIST_MIN_POSITION)
      position = Rotations.of(WristConstants.WRIST_MIN_POSITION);
  }

  public void stopWrist() {
    io.stop();
  }

  public void setPercentOutput(double percentOutput) {
    io.setPercentOutput(percentOutput);
  }

  public void stop() {
    io.stop();
  }

  public boolean isConnected() {
    return inputs.motorInputs.isConnected;
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public MotorIOInputs getMotorInputs() {
    return inputs.motorInputs;
  }
}
