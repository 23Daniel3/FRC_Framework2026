package org.ciprix.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Supplier;
import org.ciprix.vision.PoseObservation.Type;

/**
 * A Limelight, read straight from NetworkTables (no LimelightHelpers needed). Every image since the
 * last cycle comes from the NetworkTables queue, so no image is lost and none is used twice. Both
 * the MegaTag 1 and the MegaTag 2 pose of each image are reported; the filter decides which to use.
 *
 * <pre>{@code
 * new VisionIOLimelight("limelight-front", drivetrain::getRotation)
 * }</pre>
 */
public class VisionIOLimelight implements VisionIO {

  /** A camera that sent nothing for this long is disconnected. */
  private static final double DISCONNECTED_AFTER_SECONDS = 0.25;

  /** In a botpose array, the data of the first tag starts here, with 7 values per tag. */
  private static final int FIRST_TAG_INDEX = 11;

  private static final int VALUES_PER_TAG = 7;

  private final Supplier<Rotation2d> robotHeading;
  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * @param name name of the Limelight, as set in its web page (for example {@code
   *     "limelight-front"})
   * @param robotHeading heading of the robot from the gyro, sent every cycle for MegaTag 2
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> robotHeading) {
    this.robotHeading = robotHeading;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
    megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double sinceLastImage =
        (RobotController.getFPGATime() - latencySubscriber.getLastChange()) * 1.0e-6;
    inputs.connected = sinceLastImage < DISCONNECTED_AFTER_SECONDS;
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // MegaTag 2 needs the heading of the robot in every image; flushing sends it at once.
    Rotation2d heading = robotHeading == null ? null : robotHeading.get();
    if (heading != null) {
      orientationPublisher.accept(new double[] {heading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
      NetworkTableInstance.getDefault().flush();
    }

    List<PoseObservation> observations = new ArrayList<>();
    Set<Integer> tagIds = new TreeSet<>();
    read(megatag1Subscriber, Type.MEGATAG_1, observations, tagIds);
    read(megatag2Subscriber, Type.MEGATAG_2, observations, tagIds);
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }

  private static void read(
      DoubleArraySubscriber subscriber,
      Type type,
      List<PoseObservation> observations,
      Set<Integer> tagIds) {
    for (TimestampedDoubleArray sample : subscriber.readQueue()) {
      PoseObservation observation = parse(sample.value, sample.timestamp, type);
      if (observation != null) {
        observations.add(observation);
        tagIds.addAll(tagIdsOf(sample.value));
      }
    }
  }

  /**
   * Turns one Limelight {@code botpose} array into an observation. The array holds x, y, z (m),
   * roll, pitch, yaw (degrees), total latency (ms), tag count, tag span, average distance, average
   * area, then 7 values per tag: id, txnc, tync, ta, distance to the camera, distance to the robot
   * and ambiguity.
   *
   * @param botpose the array, as published by the Limelight
   * @param receivedMicros when NetworkTables received it, in microseconds
   * @param type MegaTag 1 or MegaTag 2
   * @return the observation, or null when no tag was seen
   */
  public static PoseObservation parse(double[] botpose, long receivedMicros, Type type) {
    if (botpose == null || botpose.length < FIRST_TAG_INDEX || botpose[7] < 1.0) {
      return null;
    }
    double timestamp = receivedMicros * 1.0e-6 - botpose[6] * 1.0e-3;
    Pose3d pose =
        new Pose3d(
            botpose[0],
            botpose[1],
            botpose[2],
            new Rotation3d(
                Math.toRadians(botpose[3]),
                Math.toRadians(botpose[4]),
                Math.toRadians(botpose[5])));
    double ambiguity = 0.0;
    for (int i = FIRST_TAG_INDEX; i + VALUES_PER_TAG <= botpose.length; i += VALUES_PER_TAG) {
      ambiguity = Math.max(ambiguity, botpose[i + 6]);
    }
    return new PoseObservation(timestamp, pose, ambiguity, (int) botpose[7], botpose[9], type);
  }

  /** Ids of the tags listed in a {@code botpose} array. */
  public static List<Integer> tagIdsOf(double[] botpose) {
    List<Integer> ids = new ArrayList<>();
    if (botpose != null) {
      for (int i = FIRST_TAG_INDEX; i + VALUES_PER_TAG <= botpose.length; i += VALUES_PER_TAG) {
        ids.add((int) botpose[i]);
      }
    }
    return ids;
  }
}
