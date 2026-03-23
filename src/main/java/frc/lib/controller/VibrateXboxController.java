package frc.lib.controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Fábrica de comandos para vibração do Xbox Controller. Centraliza padrões de Rumble para feedback
 * tátil ao piloto.
 */
public class VibrateXboxController {

  private final CommandXboxController controller;

  public VibrateXboxController(CommandXboxController controller) {
    this.controller = controller;
  }

  /** Define a vibração bruta nos motores. */
  private void setRumble(double heavy, double light) {
    controller.getHID().setRumble(RumbleType.kLeftRumble, heavy);
    controller.getHID().setRumble(RumbleType.kRightRumble, light);
  }

  /** Para toda a vibração do controle. */
  private void stopRumble() {
    setRumble(0, 0);
  }

  /**
   * Vibração contínua básica. *
   *
   * @param heavy Nível de vibração pesada (0.0 a 1.0)
   * @param light Nível de vibração leve (0.0 a 1.0)
   * @param seconds Duração (se <= 0 e keepAlive for true, roda indefinidamente)
   * @param keepAlive Se false, o comando termina após o tempo. Se true, ignora o tempo.
   */
  public Command continuous(double heavy, double light, double seconds, boolean keepAlive) {
    Command cmd = Commands.startEnd(() -> setRumble(heavy, light), this::stopRumble);

    return keepAlive ? cmd : cmd.withTimeout(seconds);
  }

  /**
   * Vibração em pulsos (Liga/Desliga). * @param intensity Força da vibração (0.0 a 1.0)
   *
   * @param pulseCount Quantidade de pulsos dentro do tempo total
   * @param totalDuration Duração total do comando
   */
  public Command pulses(double intensity, int pulseCount, double totalDuration) {
    Timer timer = new Timer();
    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.run(
                () -> {
                  double time = timer.get();
                  // Lógica: divide o tempo total em fatias e verifica se está na fatia de "ligado"
                  boolean isOn = ((int) (time * (pulseCount * 2) / totalDuration) % 2 == 0);
                  setRumble(isOn ? intensity : 0, isOn ? intensity : 0);
                }))
        .withTimeout(totalDuration)
        .finallyDo(this::stopRumble);
  }

  /**
   * Vibração "Zig-Zag" que alterna rapidamente entre o motor pesado e o leve. Dá uma sensação de
   * instabilidade ou movimento lateral.
   */
  public Command zigZag(double intensity, double seconds) {
    Timer timer = new Timer();
    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.run(
                () -> {
                  if ((int) (timer.get() * 15) % 2 == 0) { // Alterna a cada ~0.06s
                    setRumble(intensity, 0);
                  } else {
                    setRumble(0, intensity);
                  }
                }))
        .withTimeout(seconds)
        .finallyDo(this::stopRumble);
  }

  /** Vibração que aumenta de intensidade gradualmente (Rampa). */
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

  // --- MÉTODOS DE ATALHO CONVENIENTES ---

  /** Alerta rápido (0.2s) de alta intensidade leve para confirmação de ação. */
  public Command lightConfirm() {
    return continuous(0, 0.8, 0.2, false);
  }

  /** Alerta pesado para avisar de colisões ou fim de jogo. */
  public Command heavyWarning() {
    return pulses(1.0, 3, 1.0);
  }
}
