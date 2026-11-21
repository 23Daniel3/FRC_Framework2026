package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.littletonrobotics.junction.AutoLog;

public interface LedIO {

  @AutoLog
  class LedIOInputs {
    public int length;
    public int[] red;
    public int[] green;
    public int[] blue;
  }

  default void updateInputs(LedIOInputs inputs) {}

  default void apply(AddressableLEDBuffer buffer) {}

  default void clear() {}
}
