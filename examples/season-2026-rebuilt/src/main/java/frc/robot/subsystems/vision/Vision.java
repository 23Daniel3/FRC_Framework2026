package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.VisionCamera;
import java.util.EnumMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ciprix.util.ConstantsLogger;
import org.ciprix.util.PeriodicTimer;
import org.ciprix.util.RobotClock;
import org.ciprix.vision.PoseObservation;
import org.ciprix.vision.VisionFilter;
import org.ciprix.vision.VisionFilter.Rejection;
import org.ciprix.vision.VisionProcessor;
import org.ciprix.vision.VisionStdDevs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Vision extends SubsystemBase {

  private final VisionConsumer consumer;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;
  private final Supplier<Pose2d> estimatedPose;
  private final DoubleSupplier rotation;

  // Regras da biblioteca (limites do campo 2026, robo no chao, giro rapido, salto de tag unica).
  private final VisionFilter filter =
      new VisionFilter()
          .field(APRILTAG_LAYOUT)
          .maxDistance(MAX_DISTANCE)
          .maxAmbiguity(MAX_AMBIGUITY)
          .maxJump(MAX_SINGLE_TAG_JUMP);

  // Cada imagem e julgada uma vez so: a Limelight e lida como "ultimo valor" a cada ciclo, entao a
  // mesma imagem aparece em varios ciclos e nao pode entrar duas vezes no estimador.
  private final EnumMap<VisionCamera, Double> lastJudgedTimestamps =
      new EnumMap<>(VisionCamera.class);
  private double lastAcceptedTime = Double.NEGATIVE_INFINITY;

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
      Supplier<Pose2d> estimatedPose,
      DoubleSupplier rotation,
      VisionIO... ios) {

    this.consumer = consumer;
    this.robotVelocitySupplier = robotVelocitySupplier;
    this.estimatedPose = estimatedPose;
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
        String rejection = rejectionReason(camera, inputs, robotSpeeds);
        if (rejection.isEmpty()) {
          var stdDevs = calculateVisionStdDevs(camera, inputs, robotSpeeds);
          if (usesMegatag1Heading(inputs)) {
            // Com 2 ou mais tags o MegaTag 1 mede o angulo de verdade; o MegaTag 2 so devolve o
            // angulo do giroscopio, por isso nesse caso o desvio do angulo e ignorado.
            Pose2d pose =
                new Pose2d(
                    inputs.pose.getX(),
                    inputs.pose.getY(),
                    Rotation2d.fromDegrees(inputs.angleMegatag1));
            consumer.accept(pose, inputs.timestamp, stdDevs);
          } else {
            consumer.accept(inputs.pose, inputs.timestamp, stdDevs);
          }
          lastAcceptedTime = RobotClock.now();
        }
        Logger.recordOutput(
            "Subsystems/Vision/" + io.getCamera().name() + "/isValidatingPose",
            rejection.isEmpty());
        Logger.recordOutput(
            "Subsystems/Vision/" + io.getCamera().name() + "/RejectReason", rejection);
      }
    }
    PeriodicTimer.stop(getName());
  }

  /** Com 2 ou mais tags o angulo vem do MegaTag 1 (medido pelas tags, nao pelo giroscopio). */
  private static boolean usesMegatag1Heading(VisionIOInputsAutoLogged inputs) {
    return inputs.tagCount >= 2;
  }

  /**
   * Por que a pose desta camera nao entra no estimador, ou vazio quando ela entra. O motivo vai
   * para o log ({@code RejectReason}), assim uma camera com problema aparece no AdvantageScope.
   */
  private String rejectionReason(
      VisionCamera camera, VisionIOInputsAutoLogged inputs, ChassisSpeeds speeds) {
    if (camera == VisionCamera.FRONT) {
      return "FRONT_CAMERA";
    }
    if (stopVision.get() == StopVision.STOP) {
      return "DISABLED";
    }
    if (!inputs.hasPose) {
      return Rejection.NO_TAGS.name();
    }
    if (inputs.timestamp <= lastJudgedTimestamps.getOrDefault(camera, Double.NEGATIVE_INFINITY)) {
      return Rejection.DUPLICATE.name();
    }
    lastJudgedTimestamps.put(camera, inputs.timestamp);
    // A equipe tambem descarta tag unica ambigua no MegaTag 2 (a biblioteca so no MegaTag 1).
    if (inputs.tagCount == 1 && inputs.ambiguity > MAX_AMBIGUITY) {
      return Rejection.AMBIGUOUS.name();
    }
    PoseObservation observation =
        new PoseObservation(
            inputs.timestamp,
            new Pose3d(inputs.pose),
            inputs.ambiguity,
            inputs.tagCount,
            inputs.avgTagDistance,
            PoseObservation.Type.MEGATAG_2);
    boolean estimateTrusted = RobotClock.now() - lastAcceptedTime < VisionProcessor.TRUST_SECONDS;
    Rejection rejection =
        filter.check(
            observation,
            new VisionFilter.Context(
                speeds.omegaRadiansPerSecond, estimatedPose.get(), estimateTrusted));
    return rejection == Rejection.NONE ? "" : rejection.name();
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

    // Sem o angulo do MegaTag 1 a pose traz o proprio giroscopio: corrigir o giroscopio com ele
    // mesmo so injetaria o atraso da camera no angulo.
    double thetaStdDev = usesMegatag1Heading(inputs) ? finalTheta : VisionStdDevs.IGNORED;
    return VecBuilder.fill(finalXY, finalXY, thetaStdDev);
  }

  /** True quando alguma camera corrigiu a pose no ultimo segundo: o robo sabe onde esta. */
  public boolean hasRecentPose() {
    return RobotClock.now() - lastAcceptedTime < VisionProcessor.TRUST_SECONDS;
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
