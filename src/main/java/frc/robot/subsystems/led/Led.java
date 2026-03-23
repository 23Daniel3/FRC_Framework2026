package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {

  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
  private final AddressableLEDBuffer buffer;

  // Limite de potência (20%)
  private static final double kBrightness = 0.2;
  private static final int kHsvValue = (int) (255 * kBrightness);

  // ================= MODOS =================
  private enum Mode {
    OFF,
    SOLID,
    RAINBOW,
    RAINBOW_CONTINUOUS,
    BLINK,
    BREATHING,
    CHASE
  }

  private Mode mode = Mode.OFF;
  private Color primaryColor = Color.kGreen;

  // ================= ESTADOS INTERNOS =================
  private double lastToggleTime = 0.0;
  private boolean blinkOn = false;
  private int chaseIndex = 0;

  // Rainbow contínuo
  private int rainbowHueOffset = 0;
  private double rainbowSpeed = 0.0; // 0–10
  private double lastRainbowUpdate = 0.0;

  // ================= CONSTRUTOR =================
  public Led(LedIO io) {
    this.io = io;
    this.buffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);
    setName("Subsystems/Led");
    ConstantsLogger.logConstants(LedConstants.class, getName());
  }

  // ================= PERIODIC =================
  @Override
  public void periodic() {
    PeriodicTimer.tick(getName());
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    applyEffect();
    io.apply(buffer);
  }

  // ================= ENGINE DE EFEITOS =================
  private void applyEffect() {
    switch (mode) {
      case OFF -> fill(Color.kBlack);
      case SOLID -> fill(primaryColor);
      case RAINBOW -> LEDPattern.rainbow(255, kHsvValue).applyTo(buffer);
      case RAINBOW_CONTINUOUS -> rainbowContinuousEffect();
      case BLINK -> blinkEffect();
      case BREATHING -> breathingEffect();
      case CHASE -> chaseEffect();
    }
  }

  // ================= UTIL =================
  private void fill(Color color) {
    // Aplica o limite de 20% nos canais RGB
    Color dimmedColor =
        new Color(color.red * kBrightness, color.green * kBrightness, color.blue * kBrightness);

    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, dimmedColor);
    }
  }

  // ================= EFEITOS MANUAIS =================

  private void blinkEffect() {
    double now = Timer.getFPGATimestamp();
    if (now - lastToggleTime > 0.3) {
      blinkOn = !blinkOn;
      lastToggleTime = now;
    }
    fill(blinkOn ? primaryColor : Color.kBlack);
  }

  private void breathingEffect() {
    double t = Timer.getFPGATimestamp();
    // O brilho oscila, mas o pico é limitado pelo kBrightness
    double intensity = (0.5 + 0.5 * Math.sin(2 * Math.PI * t / 2.0)) * kBrightness;

    Color c =
        new Color(
            primaryColor.red * intensity,
            primaryColor.green * intensity,
            primaryColor.blue * intensity);

    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, c);
    }
  }

  private void chaseEffect() {
    fill(Color.kBlack);
    // Aplica brilho reduzido no pixel do chase
    Color dimmedColor =
        new Color(
            primaryColor.red * kBrightness,
            primaryColor.green * kBrightness,
            primaryColor.blue * kBrightness);

    buffer.setLED(chaseIndex, dimmedColor);
    chaseIndex = (chaseIndex + 1) % buffer.getLength();
  }

  private void rainbowContinuousEffect() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastRainbowUpdate;
    lastRainbowUpdate = now;

    rainbowHueOffset += (int) (rainbowSpeed * 20 * dt);
    rainbowHueOffset %= 180;

    int length = buffer.getLength();
    for (int i = 0; i < length; i++) {
      int hue = (rainbowHueOffset + (i * 180 / length)) % 180;
      buffer.setHSV(i, hue, 255, kHsvValue);
    }
  }

  // ================= API PÚBLICA =================

  public void off() {
    mode = Mode.OFF;
  }

  public void solid(Color color) {
    primaryColor = color;
    mode = Mode.SOLID;
  }

  public void rainbow() {
    mode = Mode.RAINBOW;
  }

  public void rainbowContinuous(double speed) {
    rainbowSpeed = Math.max(0.0, Math.min(10.0, speed));
    rainbowHueOffset = 0;
    lastRainbowUpdate = Timer.getFPGATimestamp();
    mode = Mode.RAINBOW_CONTINUOUS;
  }

  public void blink(Color color) {
    primaryColor = color;
    mode = Mode.BLINK;
  }

  public void breathe(Color color) {
    primaryColor = color;
    mode = Mode.BREATHING;
  }

  public void chase(Color color) {
    primaryColor = color;
    chaseIndex = 0;
    mode = Mode.CHASE;
  }
}
