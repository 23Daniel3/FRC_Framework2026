package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedIOSim implements LedIO {

  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);

  @Override
  public void updateInputs(LedIOInputs inputs) {
    int length = buffer.getLength();
    inputs.length = length;

    inputs.red = new int[length];
    inputs.green = new int[length];
    inputs.blue = new int[length];

    for (int i = 0; i < length; i++) {
      inputs.red[i] = buffer.getRed(i);
      inputs.green[i] = buffer.getGreen(i);
      inputs.blue[i] = buffer.getBlue(i);
    }
  }

  @Override
  public void apply(AddressableLEDBuffer newBuffer) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, newBuffer.getRed(i), newBuffer.getGreen(i), newBuffer.getBlue(i));
    }
  }

  @Override
  public void clear() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
  }
}
