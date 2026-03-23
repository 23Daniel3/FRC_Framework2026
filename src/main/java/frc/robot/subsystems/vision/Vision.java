package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.robot.subsystems.vision.VisionConstants.VisionCamera;
import java.util.EnumMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Vision extends SubsystemBase {

  private final VisionConsumer consumer;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;
  private final DoubleSupplier rotation;

  enum StopVision {
    STOP,
    RUN
  }

  private final LoggedDashboardChooser<StopVision> stopVision;

  private final EnumMap<VisionCamera, VisionIO> ioMap = new EnumMap<>(VisionCamera.class);
  private final EnumMap<VisionCamera, VisionIOInputsAutoLogged> inputsMap =
      new EnumMap<>(VisionCamera.class);

  private final EnumMap<VisionCamera, Double> cameraImuOffsets = new EnumMap<>(VisionCamera.class);

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public Vision(
      VisionConsumer consumer,
      Supplier<ChassisSpeeds> robotVelocitySupplier,
      DoubleSupplier rotation,
      VisionIO... ios) {

    this.consumer = consumer;
    this.robotVelocitySupplier = robotVelocitySupplier;
    this.rotation = rotation;
    this.stopVision = new LoggedDashboardChooser<>("Desligar Visão");

    stopVision.addDefaultOption("LIGADA", StopVision.RUN);
    stopVision.addOption("PARADA", StopVision.STOP);

    for (VisionIO io : ios) {
      ioMap.put(io.getCamera(), io);
      inputsMap.put(io.getCamera(), new VisionIOInputsAutoLogged());
      cameraImuOffsets.put(io.getCamera(), 0.0);
    }

    ConstantsLogger.logConstants(VisionConstants.class, "Subsystems/Vision");

    setName("Subsystems/Vision");
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());
    ChassisSpeeds robotSpeeds = robotVelocitySupplier.get();

    for (var camera : ioMap.keySet()) {
      var io = ioMap.get(camera);
      var inputs = inputsMap.get(camera);

      io.updateRobotOrientation(rotation);
      io.updateInputs(inputs);
      Logger.processInputs(getName() + "/" + camera.name(), inputs);

      if (camera != VisionCamera.FRONT) {
        if (shouldAcceptPose(camera, inputs)) {
          var stdDevs = calculateVisionStdDevs(camera, inputs, robotSpeeds);
          if (inputs.tagCount == 2) {
            Pose2d pose =
                new Pose2d(
                    inputs.pose.getX(),
                    inputs.pose.getY(),
                    Rotation2d.fromDegrees(inputs.angleMegatag1));
            consumer.accept(pose, inputs.timestamp, stdDevs);
          } else {
            consumer.accept(inputs.pose, inputs.timestamp, stdDevs);
          }
          Logger.recordOutput(
              "Subsystems/Vision/" + io.getCamera().name() + "/isValidatingPose", true);
        } else {
          Logger.recordOutput(
              "Subsystems/Vision/" + io.getCamera().name() + "/isValidatingPose", false);
        }
      }
    }
    PeriodicTimer.stop(getName());
  }

  private boolean shouldAcceptPose(VisionCamera camera, VisionIOInputsAutoLogged inputs) {
    if (!inputs.hasPose
        || camera == VisionCamera.FRONT
        || (inputs.tagCount == 1 && inputs.ambiguity > VisionConstants.MAX_AMBIGUITY)
        || inputs.avgTagDistance > VisionConstants.MAX_DISTANCE
        || stopVision.get() == StopVision.STOP) return false;

    return true;
  }

  private Matrix<N3, N1> calculateVisionStdDevs(
      VisionCamera camera, VisionIOInputsAutoLogged inputs, ChassisSpeeds speeds) {

    double distance = inputs.avgTagDistance;
    int tagCount = inputs.tagCount;

    // --- 1. Base pela Distância (Cubic Spline) ---
    double stdXY = XY_STD_MAP.applyThrottle(distance);
    double stdTheta = THETA_STD_MAP.applyThrottle(distance);

    // --- 2. Correção de Unidades na Velocidade ---
    // Multiplicamos omega pelo raio do robô para obter a "velocidade tangencial" nos cantos.
    double linearVel = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double tangentialVel = Math.abs(speeds.omegaRadiansPerSecond) * ROBOT_RADIUS;
    double combinedMotion = linearVel + tangentialVel;

    double motionScale = DYNAMIC_SCALER.applyThrottle(combinedMotion);

    // --- 3. Penalidade de Tags e Ambiguidade Suave ---
    double tagMultiplier = 1.0;
    if (tagCount == 1) {
      // Aplica penalidade base de 1 tag + penalidade progressiva por ambiguidade
      tagMultiplier = SINGLE_TAG_PENALTY * AMBIGUITY_PENALTY_MAP.applyThrottle(inputs.ambiguity);
    } else if (tagCount > 2) {
      tagMultiplier = MULTI_TAG_REWARD;
    }

    // --- 4. Composição Final ---
    double finalXY = stdXY * motionScale * tagMultiplier;
    double finalTheta = stdTheta * motionScale * tagMultiplier;

    String baseKey = "Subsystems/Vision/" + camera.name() + "/";

    if (VisionConstants.ALL_LOG_ACTIVE) {
      Logger.recordOutput(baseKey + "avgTagDistance", inputs.avgTagDistance);
      Logger.recordOutput(baseKey + "tagCount", inputs.tagCount);
      Logger.recordOutput(baseKey + "ambiguity", inputs.ambiguity);

      Logger.recordOutput(baseKey + "stdXY_fromDistance", stdXY);
      Logger.recordOutput(baseKey + "stdTheta_fromDistance", stdTheta);

      Logger.recordOutput(baseKey + "linearVelocity", linearVel);
      Logger.recordOutput(baseKey + "tangentialVelocity", tangentialVel);
      Logger.recordOutput(baseKey + "combinedMotion", combinedMotion);
      Logger.recordOutput(baseKey + "motionScale", motionScale);

      Logger.recordOutput(baseKey + "tagMultiplier", tagMultiplier);
      Logger.recordOutput(baseKey + "tagBranchSingle", tagCount == 1);
      Logger.recordOutput(baseKey + "tagBranchMulti", tagCount > 2);

      Logger.recordOutput(baseKey + "finalStdXY", finalXY);
      Logger.recordOutput(baseKey + "finalStdTheta", finalTheta);

      Logger.recordOutput(baseKey + "visionStdDevs/x", finalXY);
      Logger.recordOutput(baseKey + "visionStdDevs/y", finalXY);
      Logger.recordOutput(baseKey + "visionStdDevs/theta", finalTheta);
    }

    return VecBuilder.fill(finalXY, finalXY, finalTheta);
  }

  public double getAvgTagDistance(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).avgTagDistance;
    }
    return 0.0;
  }

  public double getAmbiguity(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).ambiguity;
    }
    return 0.0;
  }

  public double getTx(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).tx;
    }
    return 0.0;
  }

  public double getTy(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).ty;
    }
    return 0.0;
  }

  public double getTa(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).ta;
    }
    return 0.0;
  }

  public double getTimestamp(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).timestamp;
    }
    return 0.0;
  }

  public int getCurrentPipeline(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).currentPipeline;
    }
    return -1;
  }

  public int getFiducialID(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).fiducialID;
    }
    return -1;
  }

  public int getTagCount(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).tagCount;
    }
    return 0;
  }

  public boolean hasTarget(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).tv;
    }
    return false;
  }

  public boolean isConnected(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).connected;
    }
    return false;
  }

  public boolean hasPose(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).hasPose;
    }
    return false;
  }

  public Pose2d getPose(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      return inputsMap.get(camera).pose;
    }
    return new Pose2d();
  }

  public void setCameraPipeline(VisionCamera camera, int pipelineIndex) {
    if (ioMap.containsKey(camera)) {
      ioMap.get(camera).setPipeline(pipelineIndex);
    }
  }

  public void setCameraPriorityTagID(VisionCamera camera, int tagID) {
    if (ioMap.containsKey(camera)) {
      ioMap.get(camera).setPriorityTagID(tagID);
    }
  }

  public void setCameraLEDForceOn(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      ioMap.get(camera).setLEDMode_ForceOn();
    }
  }

  public void setCameraLEDForceOff(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      ioMap.get(camera).setLEDMode_ForceOff();
    }
  }

  public void setCameraLEDForceBlink(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      ioMap.get(camera).setLEDMode_ForceBlink();
    }
  }

  public void setCameraLEDModePipelineControl(VisionCamera camera) {
    if (ioMap.containsKey(camera)) {
      ioMap.get(camera).setLEDMode_PipelineControl();
    }
  }
}
