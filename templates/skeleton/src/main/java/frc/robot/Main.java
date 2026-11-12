package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Ponto de entrada. Nao coloque nada aqui: a inicializacao fica em {@link Robot}. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
