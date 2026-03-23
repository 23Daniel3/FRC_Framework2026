package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.vision.VisionConstants.VisionCamera;
import java.util.function.DoubleSupplier;

public class VisionIOLimelight implements VisionIO {

  private final String limelightName;
  private final VisionCamera camera;

  public VisionIOLimelight(String limelightName, VisionCamera camera) {
    this.limelightName = limelightName;
    this.camera = camera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = LimelightHelpers.getLatency_Pipeline(limelightName) > 0.0;
    if (camera != VisionCamera.FRONT) {
      inputs.angleMegatag1 =
          LimelightHelpers.getBotPose2d_wpiBlue(limelightName).getRotation().getDegrees();

      var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
      RawFiducial[] tags = LimelightHelpers.getRawFiducials(limelightName);

      if (tags.length > 0) {
        double sum = 0.0;
        for (RawFiducial tag : tags) {
          sum += tag.ambiguity;
        }
        inputs.ambiguity = sum / tags.length;
      } else {
        inputs.ambiguity = 1.0;
      }

      if (estimate == null || estimate.tagCount == 0) {
        inputs.hasPose = false;
      } else {
        inputs.hasPose = true;
        inputs.pose = estimate.pose;
        inputs.timestamp = estimate.timestampSeconds;
        inputs.tagCount = estimate.tagCount;
        inputs.avgTagDistance = estimate.avgTagDist;
      }

      inputs.botPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace(limelightName);

      inputs.fiducialID = (int) LimelightHelpers.getFiducialID(limelightName);
      inputs.currentPipeline = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    inputs.ta = LimelightHelpers.getTA(limelightName);
    inputs.tx = LimelightHelpers.getTX(limelightName);
    inputs.ty = LimelightHelpers.getTY(limelightName);
    inputs.tv = LimelightHelpers.getTV(limelightName);
  }

  @Override
  public void updateRobotOrientation(DoubleSupplier yawDegress) {
    LimelightHelpers.SetRobotOrientation(limelightName, yawDegress.getAsDouble(), 0, 0, 0, 0, 0);
  }

  @Override
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(limelightName, pipeline);
  }

  @Override
  public void setPriorityTagID(int tagID) {
    LimelightHelpers.setPriorityTagID(limelightName, tagID);
  }

  @Override
  public void setLEDMode_ForceBlink() {
    LimelightHelpers.setLEDMode_ForceBlink(limelightName);
  }

  @Override
  public void setLEDMode_ForceOff() {
    LimelightHelpers.setLEDMode_ForceOff(limelightName);
  }

  @Override
  public void setLEDMode_ForceOn() {
    LimelightHelpers.setLEDMode_ForceOn(limelightName);
  }

  @Override
  public void setLEDMode_PipelineControl() {
    LimelightHelpers.setLEDMode_PipelineControl(limelightName);
  }

  @Override
  public VisionCamera getCamera() {
    return camera;
  }
}
