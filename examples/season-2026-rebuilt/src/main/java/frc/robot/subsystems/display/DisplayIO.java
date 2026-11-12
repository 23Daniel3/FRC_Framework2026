package frc.robot.subsystems.display;

import org.littletonrobotics.junction.AutoLog;

public interface DisplayIO {

  @AutoLog
  class DisplayIOInputs {
    public boolean buttonPressed = false;

    public boolean isConnected = false;
  }

  default void updateInputs(DisplayIOInputs inputs) {}

  default void clear() {}

  default void writeLine1(String text) {}

  default void writeLine2(String text) {}

  default void writeContinuous(String text) {}
}
