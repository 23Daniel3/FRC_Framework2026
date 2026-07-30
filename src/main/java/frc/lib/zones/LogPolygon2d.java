package frc.lib.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class LogPolygon2d {

  public static void logPolygon(String nameOrPath, Polygon2d polygon) {
    if (polygon == null) return;
    var vertices = polygon.getVertices();
    Pose2d[] poses = new Pose2d[vertices.length];

    for (int i = 0; i < vertices.length; i++) {
      poses[i] = new Pose2d(vertices[i], new Rotation2d());
    }

    String path = nameOrPath.contains("/") ? nameOrPath : "Zones/" + nameOrPath;
    Logger.recordOutput(path, poses);
  }
}
