package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.calculus.ThrottleMap;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.zones.LogPolygon2d;
import frc.lib.zones.Polygon2d;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import org.littletonrobotics.junction.Logger;

public final class ConstantsLogger {

  private ConstantsLogger() {}

  public static void logConstants(Class<?> clazz) {
    if (clazz == null) return;
    logConstants(clazz, clazz.getSimpleName());
  }

  public static void logConstants(Class<?> clazz, String rootPath) {
    if (clazz == null || rootPath == null) return;

    for (Field field : clazz.getDeclaredFields()) {
      if (shouldSkipField(field)) continue;

      try {
        field.setAccessible(true);
        String path = rootPath + "/Constants/" + field.getName();
        Object value = field.get(null);

        if (value != null) {
          logValue(path, value);
        }

      } catch (Throwable e) {
        // Catch Throwable to prevent ANY crash (Exception or Error) from killing the robot loop
        System.err.println(
            "Telemetry Error: Failed to log field '"
                + field.getName()
                + "' in "
                + clazz.getSimpleName());
        e.printStackTrace();
      }
    }
  }

  private static boolean shouldSkipField(Field field) {
    int modifiers = field.getModifiers();
    return !Modifier.isStatic(modifiers) || !Modifier.isPublic(modifiers);
  }

  private static void logValue(String path, Object value) {
    // Safety check: specific logic for specific types
    if (value instanceof Number) {
      if (value instanceof Double || value instanceof Float) {
        Logger.recordOutput(path, ((Number) value).doubleValue());
      } else {
        Logger.recordOutput(path, ((Number) value).longValue());
      }

    } else if (value instanceof Boolean) {
      Logger.recordOutput(path, (Boolean) value);

    } else if (value instanceof String) {
      Logger.recordOutput(path, (String) value);

    } else if (value instanceof Enum<?>) {
      Logger.recordOutput(path, value.toString());

    } else if (value instanceof double[]) {
      Logger.recordOutput(path, (double[]) value);

    } else if (value instanceof ThrottleMap) {
      logThrottleMap(path, (ThrottleMap) value);

    } else if (value instanceof Polygon2d) {
      LogPolygon2d.logPolygon(path, (Polygon2d) value);

    } else if (value instanceof Polygon2d[]) {
      Polygon2d[] polygons = (Polygon2d[]) value;
      for (int i = 0; i < polygons.length; i++) {
        if (polygons[i] != null) {
          LogPolygon2d.logPolygon(path + "/" + i, polygons[i]);
        }
      }

    } else if (value instanceof Pose2d) {
      Logger.recordOutput(path, (Pose2d) value);

    } else if (value instanceof Pose2d[]) {
      Logger.recordOutput(path, (Pose2d[]) value);

    } else if (value instanceof Pose3d) {
      Logger.recordOutput(path, (Pose3d) value);

    } else if (value instanceof Translation2d) {
      Logger.recordOutput(path, (Translation2d) value);

    } else if (value instanceof Translation2d[]) {
      Translation2d[] points = (Translation2d[]) value;
      Pose2d[] poses = new Pose2d[points.length];
      for (int i = 0; i < points.length; i++) {
        poses[i] = new Pose2d(points[i], new Rotation2d());
      }
      Logger.recordOutput(path, poses);

    } else if (value instanceof Translation3d) {
      Logger.recordOutput(path, (Translation3d) value);

    } else if (value instanceof MotorConfig) {
      ((MotorConfig) value).toLog(path);

    } else {
      try {
        Logger.recordOutput(path, String.valueOf(value));
      } catch (Exception ignored) {
        Logger.recordOutput(path, "ERROR: toString() failed");
      }
    }
  }

  private static void logThrottleMap(String root, ThrottleMap map) {
    try {
      double[][] data = map.getCurveData();

      // Ensure data is valid before accessing indices
      if (data != null && data.length >= 2 && data[0] != null && data[1] != null) {
        Logger.recordOutput(root + "/CurveX", data[0]);
        Logger.recordOutput(root + "/CurveY", data[1]);
      }

      if (map.getMode() != null) {
        Logger.recordOutput(root + "/Mode", map.getMode().toString());
      }
    } catch (Exception e) {
      Logger.recordOutput(root + "/Error", "Failed to parse ThrottleMap");
    }
  }
}
