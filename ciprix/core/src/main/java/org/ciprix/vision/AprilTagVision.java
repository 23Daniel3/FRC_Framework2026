package org.ciprix.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.ciprix.interfaces.HardwareHealthMonitor;
import org.littletonrobotics.junction.Logger;

/**
 * AprilTag vision ready to use: reads the cameras, filters the poses and corrects the odometry of
 * the drivetrain. Everything is logged under {@code Vision/<camera>}, accepted and rejected poses
 * included, and a camera that stops answering shows up in the hardware health alerts.
 *
 * <pre>{@code
 * vision =
 *     new AprilTagVision(drivetrain::addVisionMeasurement, drivetrain::getPose,
 *             () -> drivetrain.getState().Speeds)
 *         .withCamera("Front", new VisionIOLimelight("limelight-front", drivetrain::getRotation))
 *         .withCamera("Back", new VisionIOLimelight("limelight-back", drivetrain::getRotation), 1.5)
 *         .withFilter(new VisionFilter()
 *             .field(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)));
 * }</pre>
 *
 * <p>In replay, give each camera an empty IO ({@code new VisionIO() {}}): the inputs come from the
 * log and the same poses are accepted again.
 */
public class AprilTagVision extends SubsystemBase {

  /** Half the diagonal of a typical frame, to turn rotation into speed at the bumpers. */
  private static final double ROBOT_RADIUS = 0.4;

  private static final String[] NO_FAULTS = new String[0];

  private static final class Camera {
    final String name;
    final VisionIO io;
    final double stdDevFactor;
    final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    Camera(String name, VisionIO io, double stdDevFactor) {
      this.name = name;
      this.io = io;
      this.stdDevFactor = stdDevFactor;
    }
  }

  private final Supplier<Pose2d> estimatedPose;
  private final Supplier<ChassisSpeeds> robotSpeeds;
  private final VisionProcessor processor;
  private final List<Camera> cameras = new ArrayList<>();
  private boolean enabled = true;

  /**
   * @param consumer receives the accepted poses, usually {@code drivetrain::addVisionMeasurement}
   * @param estimatedPose where the robot is now, usually {@code drivetrain::getPose}
   * @param robotSpeeds robot-relative speeds, for example {@code () ->
   *     drivetrain.getState().Speeds}
   */
  public AprilTagVision(
      VisionProcessor.MeasurementConsumer consumer,
      Supplier<Pose2d> estimatedPose,
      Supplier<ChassisSpeeds> robotSpeeds) {
    this.estimatedPose = estimatedPose;
    this.robotSpeeds = robotSpeeds;
    this.processor = new VisionProcessor(new VisionFilter(), new VisionStdDevs(), consumer);
    setName("Vision");
  }

  /** Adds a camera trusted like the others. */
  public AprilTagVision withCamera(String name, VisionIO io) {
    return withCamera(name, io, 1.0);
  }

  /**
   * Adds a camera.
   *
   * @param name short name used in the logs and alerts, for example {@code "Front"}
   * @param io the camera
   * @param stdDevFactor 1 for a normal camera, more for one trusted less (2 doubles its standard
   *     deviations)
   */
  public AprilTagVision withCamera(String name, VisionIO io, double stdDevFactor) {
    Camera camera = new Camera(name, io == null ? new VisionIO() {} : io, stdDevFactor);
    cameras.add(camera);
    HardwareHealthMonitor.register(
        "Vision/" + name, () -> camera.inputs.connected, () -> NO_FAULTS);
    return this;
  }

  /** Replaces the rules that decide which poses are used. */
  public AprilTagVision withFilter(VisionFilter filter) {
    processor.setFilter(filter);
    return this;
  }

  /** Replaces how much each pose is trusted. */
  public AprilTagVision withStdDevs(VisionStdDevs stdDevs) {
    processor.setStdDevs(stdDevs);
    return this;
  }

  /** Turns the odometry correction on or off (the cameras are still read and logged). */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  /** True while the odometry is being corrected. */
  public boolean isEnabled() {
    return enabled;
  }

  /** True when a pose was accepted in the last second: the robot knows where it is. */
  public boolean hasRecentPose() {
    return processor.hasRecentPose();
  }

  /** True while the camera is sending images. */
  public boolean isConnected(String camera) {
    return find(camera).map(c -> c.inputs.connected).orElse(false);
  }

  /** True when the camera saw at least one tag in this cycle. */
  public boolean seesTags(String camera) {
    return find(camera).map(c -> c.inputs.tagIds.length > 0).orElse(false);
  }

  /** Horizontal angle to the best target of the camera (zero when it sees none). */
  public Rotation2d getTargetX(String camera) {
    return find(camera).map(c -> c.inputs.latestTargetObservation.tx()).orElse(Rotation2d.kZero);
  }

  /** Vertical angle to the best target of the camera (zero when it sees none). */
  public Rotation2d getTargetY(String camera) {
    return find(camera).map(c -> c.inputs.latestTargetObservation.ty()).orElse(Rotation2d.kZero);
  }

  private Optional<Camera> find(String name) {
    for (Camera camera : cameras) {
      if (camera.name.equals(name)) {
        return Optional.of(camera);
      }
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {
    ChassisSpeeds speeds = robotSpeeds == null ? null : robotSpeeds.get();
    double yawRate = speeds == null ? 0.0 : speeds.omegaRadiansPerSecond;
    double speed =
        speeds == null
            ? 0.0
            : Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                + Math.abs(yawRate) * ROBOT_RADIUS;
    Pose2d estimate = estimatedPose == null ? null : estimatedPose.get();
    Optional<AprilTagFieldLayout> layout = processor.getFilter().getLayout();

    for (Camera camera : cameras) {
      camera.io.updateInputs(camera.inputs);
      Logger.processInputs("Vision/" + camera.name, camera.inputs);
      if (enabled) {
        processor.process(
            camera.name, camera.inputs, camera.stdDevFactor, yawRate, speed, estimate);
      }
      if (layout.isPresent()) {
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int id : camera.inputs.tagIds) {
          layout.get().getTagPose(id).ifPresent(tagPoses::add);
        }
        Logger.recordOutput("Vision/" + camera.name + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      }
    }
    Logger.recordOutput("Vision/Enabled", enabled);
    Logger.recordOutput("Vision/HasRecentPose", processor.hasRecentPose());
  }
}
