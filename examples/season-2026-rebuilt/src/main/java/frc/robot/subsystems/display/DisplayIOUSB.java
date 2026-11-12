package frc.robot.subsystems.display;

import edu.wpi.first.wpilibj.SerialPort;
import org.littletonrobotics.junction.Logger;

public class DisplayIOUSB implements DisplayIO {

  private final SerialPort port;

  public DisplayIOUSB() {
    port = new SerialPort(DisplayConstants.BAUD_RATE, DisplayConstants.PORT_USB);
  }

  @Override
  public void updateInputs(DisplayIOInputs inputs) {
    inputs.isConnected = port != null;

    if (port.getBytesReceived() > 0) {
      String msg = port.readString().trim();
      Logger.recordOutput("Subsystems/Display/ArduinoInput", msg);
      inputs.buttonPressed = "BTN".equals(msg);
    } else {
      inputs.buttonPressed = false;
    }
  }

  @Override
  public void writeLine1(String text) {
    send("L1:" + text);
  }

  @Override
  public void writeLine2(String text) {
    send("L2:" + text);
  }

  @Override
  public void writeContinuous(String text) {
    send("Continuous:" + text);
  }

  @Override
  public void clear() {
    send("CLR");
  }

  private void send(String msg) {
    port.writeString(msg + "\n");
    Logger.recordOutput("Display/Sent", msg);
  }
}
