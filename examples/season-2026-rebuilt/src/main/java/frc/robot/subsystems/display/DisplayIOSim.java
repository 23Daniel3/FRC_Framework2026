package frc.robot.subsystems.display;

import org.littletonrobotics.junction.Logger;

public class DisplayIOSim implements DisplayIO {

  @Override
  public void updateInputs(DisplayIOInputs inputs) {
    inputs.isConnected = true;
    inputs.buttonPressed = false;
  }

  @Override
  public void writeLine1(String text) {
    Logger.recordOutput("Subsystems/Display/Line1", text);
  }

  @Override
  public void writeLine2(String text) {
    Logger.recordOutput("Subsystems/Display/Line2", text);
  }

  @Override
  public void writeContinuous(String text) {
    Logger.recordOutput("Subsystems/DisplaySim/Continuous", text);
  }

  @Override
  public void clear() {
    Logger.recordOutput("Subsystems/DisplaySim/Clear", true);
  }
}
