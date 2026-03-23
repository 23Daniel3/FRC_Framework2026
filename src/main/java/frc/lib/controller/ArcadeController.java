package frc.lib.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * {@code ArcadeController} is a wrapper for a generic arcade-style joystick with up to 16 buttons.
 *
 * <p>It provides access to each button as a {@link Trigger}, similar to {@link
 * edu.wpi.first.wpilibj2.command.button.CommandXboxController}, allowing for event-driven command
 * binding in the WPILib command framework.
 */
public class ArcadeController {
  private static final int MAX_BUTTONS = 16;
  private final Joystick joystick;
  private final Trigger[] buttons;

  /**
   * Creates an {@code ArcadeController} instance on the given port.
   *
   * @param port the driver station port of the joystick
   */
  public ArcadeController(int port) {
    this.joystick = new Joystick(port);
    this.buttons = new Trigger[MAX_BUTTONS];

    for (int i = 0; i < MAX_BUTTONS; i++) {
      buttons[i] = new JoystickButton(joystick, i + 1);
    }
  }

  /**
   * Returns the underlying WPILib {@link Joystick} instance.
   *
   * @return the joystick object backing this controller
   */
  public Joystick getJoystick() {
    return joystick;
  }

  /**
   * Returns the {@link Trigger} for a specific button index.
   *
   * @param buttonIndex the 1-based index of the button (valid range 1–16)
   * @return the {@link Trigger} representing the given button
   * @throws IllegalArgumentException if the button index is outside the valid range
   */
  public Trigger button(int buttonIndex) {
    if (buttonIndex < 1 || buttonIndex > MAX_BUTTONS) {
      throw new IllegalArgumentException("Button index must be between 1 and " + MAX_BUTTONS);
    }
    return buttons[buttonIndex - 1];
  }

  /**
   * Gets the {@link Trigger} for button 1.
   *
   * @return the Trigger for button 1
   */
  public Trigger button1() {
    return button(1);
  }

  /**
   * Gets the {@link Trigger} for button 2.
   *
   * @return the Trigger for button 2
   */
  public Trigger button2() {
    return button(2);
  }

  /**
   * Gets the {@link Trigger} for button 3.
   *
   * @return the Trigger for button 3
   */
  public Trigger button3() {
    return button(3);
  }

  /**
   * Gets the {@link Trigger} for button 4.
   *
   * @return the Trigger for button 4
   */
  public Trigger button4() {
    return button(4);
  }

  /**
   * Gets the {@link Trigger} for button 5.
   *
   * @return the Trigger for button 5
   */
  public Trigger button5() {
    return button(5);
  }

  /**
   * Gets the {@link Trigger} for button 6.
   *
   * @return the Trigger for button 6
   */
  public Trigger button6() {
    return button(6);
  }

  /**
   * Gets the {@link Trigger} for button 7.
   *
   * @return the Trigger for button 7
   */
  public Trigger button7() {
    return button(7);
  }

  /**
   * Gets the {@link Trigger} for button 8.
   *
   * @return the Trigger for button 8
   */
  public Trigger button8() {
    return button(8);
  }

  /**
   * Gets the {@link Trigger} for button 9.
   *
   * @return the Trigger for button 9
   */
  public Trigger button9() {
    return button(9);
  }

  /**
   * Gets the {@link Trigger} for button 10.
   *
   * @return the Trigger for button 10
   */
  public Trigger button10() {
    return button(10);
  }

  /**
   * Gets the {@link Trigger} for button 11.
   *
   * @return the Trigger for button 11
   */
  public Trigger button11() {
    return button(11);
  }

  /**
   * Gets the {@link Trigger} for button 12.
   *
   * @return the Trigger for button 12
   */
  public Trigger button12() {
    return button(12);
  }

  /**
   * Gets the {@link Trigger} for button 13.
   *
   * @return the Trigger for button 13
   */
  public Trigger button13() {
    return button(13);
  }

  /**
   * Gets the {@link Trigger} for button 14.
   *
   * @return the Trigger for button 14
   */
  public Trigger button14() {
    return button(14);
  }

  /**
   * Gets the {@link Trigger} for button 15.
   *
   * @return the Trigger for button 15
   */
  public Trigger button15() {
    return button(15);
  }

  /**
   * Gets the {@link Trigger} for button 16.
   *
   * @return the Trigger for button 16
   */
  public Trigger button16() {
    return button(16);
  }
}
