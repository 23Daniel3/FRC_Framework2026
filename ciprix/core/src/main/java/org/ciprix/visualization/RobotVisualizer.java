package org.ciprix.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;
import java.util.TreeMap;
import java.util.function.Supplier;
import org.ciprix.util.Report;
import org.ciprix.util.RobotLoop;
import org.littletonrobotics.junction.Logger;

/**
 * Shows the mechanisms of the robot in 3D, in AdvantageScope, with the team's own CAD.
 *
 * <p>AdvantageScope draws a custom robot from a folder with {@code model.glb} (the frame) and one
 * {@code model_N.glb} per moving part, plus a {@code config.json}. The robot code logs where each
 * part is; this class does that, once per cycle, under {@value #COMPONENTS_KEY}:
 *
 * <pre>{@code
 * // Once, after the subsystems exist (index N = model_N.glb)
 * RobotVisualizer.component(0, MechanismPoses.pivot(
 *     IntakeConstants.PIVOT_AXIS, Axis.PITCH, intake::getPivotAngle), IntakeConstants.PIVOT_AXIS);
 * }</pre>
 *
 * <p>The poses come from the inputs of the subsystems, so the same view works in simulation, on the
 * real robot and when replaying a log. In AdvantageScope: App &gt; Use Custom Assets Folder (the
 * folder that holds the {@code Robot_<name>} folders), then a 3D Field tab with the robot pose and
 * the custom robot, then drag {@value #COMPONENTS_KEY} onto it as "Components".
 *
 * <p>To add a mechanism from the CAD: export the moving part (in robot coordinates) as {@code
 * model_N.glb}, register it here with index N and the point it turns around, and paste the output
 * of {@link #advantageScopeConfig} into {@code config.json}: it already has the offsets that put
 * each pivot axis at the origin, which is how AdvantageScope expects a part to be zeroed.
 */
public final class RobotVisualizer {

  /** Key of the component poses (relative to the robot). */
  public static final String COMPONENTS_KEY = "Visualizer/Components";

  /** Rotations that turn a glTF exported from Onshape (Y up) into the robot frame (Z up). */
  public static final String ONSHAPE_ROTATIONS =
      "[{\"axis\": \"x\", \"degrees\": 90}, {\"axis\": \"z\", \"degrees\": 90}]";

  private static final class Component {
    final Supplier<Pose3d> pose;
    final Translation3d zeroPoint;
    boolean failureReported = false;

    Component(Supplier<Pose3d> pose, Translation3d zeroPoint) {
      this.pose = pose;
      this.zeroPoint = zeroPoint;
    }
  }

  private static final TreeMap<Integer, Component> components = new TreeMap<>();
  private static boolean started = false;
  private static Pose3d[] poses = new Pose3d[0];

  private RobotVisualizer() {}

  /**
   * Registers a moving part.
   *
   * @param index number of its model in AdvantageScope ({@code model_0.glb} is 0)
   * @param pose where the part is, relative to the robot (see {@link MechanismPoses})
   */
  public static void component(int index, Supplier<Pose3d> pose) {
    component(index, pose, Translation3d.kZero);
  }

  /**
   * Registers a moving part.
   *
   * @param index number of its model in AdvantageScope ({@code model_0.glb} is 0)
   * @param pose where the part is, relative to the robot (see {@link MechanismPoses})
   * @param zeroPoint the point of the CAD model that must be at the origin when the part is zeroed:
   *     the pivot axis of something that turns, the start of something that slides. Used by {@link
   *     #advantageScopeConfig}
   */
  public static synchronized void component(
      int index, Supplier<Pose3d> pose, Translation3d zeroPoint) {
    if (index < 0 || pose == null) {
      Report.error("RobotVisualizer: component(" + index + ") ignored: invalid index or pose.");
      return;
    }
    if (components.containsKey(index)) {
      Report.warning("RobotVisualizer: component " + index + " registered twice, the last wins.");
    }
    components.put(index, new Component(pose, zeroPoint == null ? Translation3d.kZero : zeroPoint));
    poses = new Pose3d[components.lastKey() + 1];
    if (!started) {
      started = true;
      RobotLoop.addPeriodic("RobotVisualizer", RobotVisualizer::update);
    }
  }

  /** Number of registered parts. */
  public static synchronized int size() {
    return components.size();
  }

  /** Poses published in the last cycle (index = model number; unused numbers stay at zero). */
  public static synchronized Pose3d[] getPoses() {
    return poses.clone();
  }

  /** Runs once per cycle: computes and publishes every pose. */
  static synchronized void update() {
    for (int i = 0; i < poses.length; i++) {
      Component component = components.get(i);
      Pose3d pose = Pose3d.kZero;
      if (component != null) {
        try {
          Pose3d value = component.pose.get();
          pose = value == null ? Pose3d.kZero : value;
        } catch (RuntimeException e) {
          // A bad supplier must not take the visualizer (or the robot) down.
          if (!component.failureReported) {
            component.failureReported = true;
            Report.error("RobotVisualizer: component " + i + " failed: " + e);
          }
        }
      }
      poses[i] = pose;
    }
    Logger.recordOutput(COMPONENTS_KEY, poses);
  }

  /**
   * Writes a complete {@code config.json} for an AdvantageScope custom robot, with one component
   * per registered part, already zeroed at its pivot point.
   *
   * @param robotName name shown in AdvantageScope
   * @param cadRotations rotations that put the exported model in the robot frame, as JSON (for
   *     example {@link #ONSHAPE_ROTATIONS}); applied to the frame and to every part
   */
  public static synchronized String advantageScopeConfig(String robotName, String cadRotations) {
    String rotations = cadRotations == null || cadRotations.isBlank() ? "[]" : cadRotations;
    StringBuilder json = new StringBuilder();
    json.append("{\n");
    json.append("  \"name\": \"").append(escape(robotName)).append("\",\n");
    json.append("  \"isFTC\": false,\n");
    json.append("  \"disableSimplification\": false,\n");
    json.append("  \"rotations\": ").append(rotations).append(",\n");
    json.append("  \"position\": [0, 0, 0],\n");
    json.append("  \"cameras\": [],\n");
    json.append("  \"components\": [");
    int count = components.isEmpty() ? 0 : components.lastKey() + 1;
    for (int i = 0; i < count; i++) {
      Component component = components.get(i);
      Translation3d zero = component == null ? Translation3d.kZero : component.zeroPoint;
      json.append(i == 0 ? "\n" : ",\n");
      json.append("    {\"zeroedRotations\": ").append(rotations);
      json.append(
          String.format(
              Locale.ROOT,
              ", \"zeroedPosition\": [%.4f, %.4f, %.4f]}",
              negate(zero.getX()),
              negate(zero.getY()),
              negate(zero.getZ())));
    }
    json.append(count == 0 ? "]\n" : "\n  ]\n");
    json.append("}\n");
    return json.toString();
  }

  /**
   * Writes {@link #advantageScopeConfig} to a file, creating the folders. Meant for simulation:
   * write it next to the hand-made {@code config.json} (for example {@code config.generated.json})
   * and copy the components from it, so nothing tuned by hand is lost.
   *
   * @return true when the file was written
   */
  public static boolean writeAdvantageScopeConfig(
      Path file, String robotName, String cadRotations) {
    try {
      Path folder = file.toAbsolutePath().getParent();
      if (folder != null) {
        Files.createDirectories(folder);
      }
      Files.writeString(
          file, advantageScopeConfig(robotName, cadRotations), StandardCharsets.UTF_8);
      return true;
    } catch (IOException | RuntimeException e) {
      Report.warning("RobotVisualizer: could not write " + file + ": " + e);
      return false;
    }
  }

  private static double negate(double value) {
    return value == 0.0 ? 0.0 : -value;
  }

  private static String escape(String text) {
    return text == null ? "" : text.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  /** Forgets every part (intended for tests). */
  public static synchronized void resetForTesting() {
    components.clear();
    poses = new Pose3d[0];
    started = false;
  }
}
