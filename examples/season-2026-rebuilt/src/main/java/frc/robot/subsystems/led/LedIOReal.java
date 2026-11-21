package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedIOReal implements LedIO {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LedIOReal() {
    led = new AddressableLED(LedConstants.PWM_PORT);
    buffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);

    led.setLength(buffer.getLength());
    clear();
    led.start();
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.length = buffer.getLength();
    inputs.red = new int[inputs.length];
    inputs.green = new int[inputs.length];
    inputs.blue = new int[inputs.length];

    for (int i = 0; i < inputs.length; i++) {
      inputs.red[i] = buffer.getRed(i);
      inputs.green[i] = buffer.getGreen(i);
      inputs.blue[i] = buffer.getBlue(i);
    }
  }

  @Override
  public void apply(AddressableLEDBuffer newBuffer) {
    led.setData(newBuffer);
  }

  @Override
  public void clear() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
    led.setData(buffer);
  }

  public AddressableLEDBuffer getBuffer() {
    return buffer;
  }
}
