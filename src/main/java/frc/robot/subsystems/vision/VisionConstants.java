package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.lib.calculus.LinearInterpolation.Point;
import frc.lib.calculus.ThrottleMap;

public class VisionConstants {
  public enum VisionCamera {
    FRONT,
    LEFT,
    RIGHT
  }

  public static final boolean ALL_LOG_ACTIVE = false;

  public static final double MAX_AMBIGUITY = 0.6;
  public static final double MAX_DISTANCE = 4.0;

  public static final AprilTagFieldLayout APRILTAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // --- Configurações Físicas ---
  public static final double ROBOT_RADIUS =
      0.43; // Metros (usado para normalizar velocidade angular)

  // --- Mapas de Desvio Padrão (Base: 2 Tags) ---
  // Cubic Spline para XY baseado na distância
  public static final ThrottleMap XY_STD_MAP =
      new ThrottleMap(
          new Point(0.0, 0.005),
          new Point(1.38, 0.005),
          new Point(1.65, 0.025),
          new Point(2.0, 0.03),
          new Point(3.0, 0.07),
          new Point(4.0, 0.10),
          new Point(5.0, 0.13));

  // Cubic Spline para Rotação baseado na distância
  public static final ThrottleMap THETA_STD_MAP =
      new ThrottleMap(
          new Point(0.0, Math.toRadians(0.5)),
          new Point(3.0, Math.toRadians(5.0)),
          new Point(7.0, Math.toRadians(40.0)));

  // --- Penalidades Suaves ---
  // Em vez de "if (ambiguity > 0.4)", usamos uma curva que cresce exponencialmente
  public static final ThrottleMap AMBIGUITY_PENALTY_MAP =
      new ThrottleMap(
          new Point(0.0, 1.0), // Perfeito
          new Point(0.2, 1.2), // Aceitável
          new Point(0.4, 4.0), // Crítico
          new Point(0.6, 15.0) // Lixo (quase ignora)
          );

  // Escalonador dinâmico baseado no "Movimento Total" (m/s)
  public static final ThrottleMap DYNAMIC_SCALER =
      new ThrottleMap(
          new Point(0.0, 1.0), // Parado
          new Point(2.0, 2.5), // Movendo
          new Point(5.0, 8.0) // Sprint/Giro agressivo
          );

  public static final double SINGLE_TAG_PENALTY = 2.5;
  public static final double MULTI_TAG_REWARD = 0.8;
}
