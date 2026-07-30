package frc.lib.controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Command factory for Xbox Controller vibration. Centralizes Rumble patterns for tactile feedback
 * to the driver.
 */
public class VibrateXboxController {

  private final CommandXboxController controller;

  public VibrateXboxController(CommandXboxController controller) {
    this.controller = controller;
  }

  /** Sets raw vibration to the motors. */
  private void setRumble(double heavy, double light) {
    controller.getHID().setRumble(RumbleType.kLeftRumble, heavy);
    controller.getHID().setRumble(RumbleType.kRightRumble, light);
  }

  /** Stops all controller vibration. */
  private void stopRumble() {
    setRumble(0, 0);
  }

  /**
   * Basic continuous vibration.
   *
   * @param heavy Heavy vibration level (0.0 to 1.0)
   * @param light Light vibration level (0.0 to 1.0)
   * @param seconds Duration (if <= 0 and keepAlive is true, runs indefinitely)
   * @param keepAlive If false, the command finishes after the duration. If true, ignores duration.
   */
  public Command continuous(double heavy, double light, double seconds, boolean keepAlive) {
    Command cmd = Commands.startEnd(() -> setRumble(heavy, light), this::stopRumble);

    return keepAlive ? cmd : cmd.withTimeout(seconds);
  }

  /**
   * Pulsed vibration (On/Off).
   *
   * @param intensity Vibration strength (0.0 to 1.0)
   * @param pulseCount Number of pulses within the total duration
   * @param totalDuration Total command duration
   */
  public Command pulses(double intensity, int pulseCount, double totalDuration) {
    Timer timer = new Timer();
    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.run(
                () -> {
                  double time = timer.get();
                  // Logic: divides the total duration into segments and checks if currently in an
                  // "on" segment
                  boolean isOn = ((int) (time * (pulseCount * 2) / totalDuration) % 2 == 0);
                  setRumble(isOn ? intensity : 0, isOn ? intensity : 0);
                }))
        .withTimeout(totalDuration)
        .finallyDo(this::stopRumble);
  }

  /**
   * "Zig-Zag" vibration that quickly alternates between the heavy and light motors. Gives a
   * sensation of instability or lateral movement.
   */
  public Command zigZag(double intensity, double seconds) {
    Timer timer = new Timer();
    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.run(
                () -> {
                  if ((int) (timer.get() * 15) % 2 == 0) { // Alternates every ~0.06s
                    setRumble(intensity, 0);
                  } else {
                    setRumble(0, intensity);
                  }
                }))
        .withTimeout(seconds)
        .finallyDo(this::stopRumble);
  }

  /** Vibration that gradually increases in intensity (Ramp). */
  public Command rampUp(double targetIntensity, double seconds) {
    Timer timer = new Timer();
    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.run(
                () -> {
                  double currentProgress = Math.min(timer.get() / seconds, 1.0);
                  setRumble(currentProgress * targetIntensity, currentProgress * targetIntensity);
                }))
        .withTimeout(seconds)
        .finallyDo(this::stopRumble);
  }

  // --- CONVENIENT SHORTCUT METHODS ---

  /** Quick alert (0.2s) of high light intensity for action confirmation. */
  public Command lightConfirm() {
    return continuous(0, 0.8, 0.2, false);
  }

  /** Heavy alert to warn of collisions or end of match. */
  public Command heavyWarning() {
    return pulses(1.0, 3, 1.0);
  }
}
