package frc.lib.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class LogPolygon2d {

  public static void logPolygon(String name, Polygon2d polygon) {
    var vertices = polygon.getVertices();
    Pose2d[] poses = new Pose2d[vertices.length];

    for (int i = 0; i < vertices.length; i++) {
      poses[i] = new Pose2d(vertices[i], new Rotation2d());
    }

    Logger.recordOutput("Zones/" + name, poses);
  }
}
