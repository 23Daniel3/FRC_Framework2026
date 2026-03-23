package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.led.Led;

/**
 * Utility class that provides factory methods for creating LED-related commands.
 *
 * <p>This class centralizes common LED behaviors (colors, animations, and timed effects) as {@link
 * Command} objects, allowing easy reuse and clean composition inside command-based robot code.
 *
 * <p>All methods return fully-configured commands that declare the {@link Led} subsystem as a
 * requirement.
 */
public final class LedCommands {

  /**
   * Creates a command that turns all LEDs off.
   *
   * @param leds the LED subsystem
   * @return a command that disables all LEDs
   */
  public static Command off(Led leds) {
    return Commands.runOnce(leds::off, leds);
  }

  /**
   * Creates a command that sets all LEDs to a solid color.
   *
   * @param leds the LED subsystem
   * @param color the color to apply
   * @return a command that sets a solid LED color
   */
  public static Command solid(Led leds, Color color) {
    return Commands.runOnce(() -> leds.solid(color), leds);
  }

  /**
   * Creates a command that sets the LEDs to solid red.
   *
   * @param leds the LED subsystem
   * @return a command that sets the LEDs to red
   */
  public static Command red(Led leds) {
    return solid(leds, Color.kRed);
  }

  /**
   * Creates a command that sets the LEDs to solid blue.
   *
   * @param leds the LED subsystem
   * @return a command that sets the LEDs to blue
   */
  public static Command blue(Led leds) {
    return solid(leds, Color.kBlue);
  }

  /**
   * Creates a command that sets the LEDs to solid green.
   *
   * @param leds the LED subsystem
   * @return a command that sets the LEDs to green
   */
  public static Command green(Led leds) {
    return solid(leds, Color.kGreen);
  }

  /**
   * Creates a command that sets the LEDs to solid yellow.
   *
   * @param leds the LED subsystem
   * @return a command that sets the LEDs to yellow
   */
  public static Command yellow(Led leds) {
    return solid(leds, Color.kYellow);
  }

  /**
   * Creates a command that sets the LEDs to solid purple.
   *
   * @param leds the LED subsystem
   * @return a command that sets the LEDs to purple
   */
  public static Command purple(Led leds) {
    return solid(leds, Color.kPurple);
  }

  /**
   * Creates a command that sets the LEDs to solid orange.
   *
   * @param leds the LED subsystem
   * @return a command that sets the LEDs to orange
   */
  public static Command orange(Led leds) {
    return solid(leds, Color.kOrange);
  }

  /**
   * Creates a command that sets the LEDs to solid white.
   *
   * @param leds the LED subsystem
   * @return a command that sets the LEDs to white
   */
  public static Command white(Led leds) {
    return solid(leds, Color.kWhite);
  }

  /**
   * Creates a command that enables the default rainbow LED effect.
   *
   * @param leds the LED subsystem
   * @return a command that activates the rainbow effect
   */
  public static Command rainbow(Led leds) {
    return Commands.runOnce(leds::rainbow, leds);
  }

  /**
   * Creates a command that enables a continuous rainbow effect with a configurable speed.
   *
   * @param leds the LED subsystem
   * @param speed the speed of the rainbow animation
   * @return a command that activates the continuous rainbow effect
   */
  public static Command rainbowContinuous(Led leds, double speed) {
    return Commands.runOnce(() -> leds.rainbowContinuous(speed), leds);
  }

  /**
   * Creates a command that makes the LEDs blink in a given color.
   *
   * @param leds the LED subsystem
   * @param color the blink color
   * @return a command that activates the blink effect
   */
  public static Command blink(Led leds, Color color) {
    return Commands.runOnce(() -> leds.blink(color), leds);
  }

  /**
   * Creates a command that makes the LEDs perform a breathing effect in a given color.
   *
   * @param leds the LED subsystem
   * @param color the breathing effect color
   * @return a command that activates the breathing effect
   */
  public static Command breathe(Led leds, Color color) {
    return Commands.runOnce(() -> leds.breathe(color), leds);
  }

  /**
   * Creates a command that makes the LEDs perform a chase effect in a given color.
   *
   * @param leds the LED subsystem
   * @param color the chase effect color
   * @return a command that activates the chase effect
   */
  public static Command chase(Led leds, Color color) {
    return Commands.runOnce(() -> leds.chase(color), leds);
  }

  /**
   * Creates a command that blinks the LEDs in a given color for a fixed duration.
   *
   * @param leds the LED subsystem
   * @param color the blink color
   * @param seconds the duration in seconds
   * @return a timed blink command
   */
  public static Command blinkForTime(Led leds, Color color, double seconds) {
    return blink(leds, color).withTimeout(seconds);
  }

  /**
   * Creates a command that sets the LEDs to a solid color for a fixed duration.
   *
   * @param leds the LED subsystem
   * @param color the solid color
   * @param seconds the duration in seconds
   * @return a timed solid color command
   */
  public static Command solidForTime(Led leds, Color color, double seconds) {
    return solid(leds, color).withTimeout(seconds);
  }

  /**
   * Creates a command that enables the rainbow effect for a fixed duration.
   *
   * @param leds the LED subsystem
   * @param seconds the duration in seconds
   * @return a timed rainbow command
   */
  public static Command rainbowForTime(Led leds, double seconds) {
    return rainbow(leds).withTimeout(seconds);
  }

  /**
   * Creates a short success animation using green LEDs.
   *
   * @param leds the LED subsystem
   * @return a command sequence representing a success indication
   */
  public static Command success(Led leds) {
    return Commands.sequence(
        solid(leds, Color.kGreen).withTimeout(0.3),
        off(leds).withTimeout(0.1),
        solid(leds, Color.kGreen).withTimeout(0.3));
  }

  /**
   * Creates an error animation using red blinking LEDs.
   *
   * @param leds the LED subsystem
   * @return a command sequence representing an error indication
   */
  public static Command error(Led leds) {
    return Commands.sequence(blink(leds, Color.kRed).withTimeout(0.6), off(leds));
  }

  /**
   * Creates a repeating loading animation using a blue chase effect.
   *
   * @param leds the LED subsystem
   * @return a repeating command representing a loading state
   */
  public static Command loading(Led leds, Color color) {
    return Commands.runOnce(() -> leds.chase(color), leds);
  }
}
