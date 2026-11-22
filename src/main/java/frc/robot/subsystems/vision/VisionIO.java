package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionConstants.VisionCamera;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;
    public boolean hasPose = false;
    public boolean tv = false;

    public Pose2d pose = new Pose2d();

    public int tagCount = 0;
    public int fiducialID = 0;
    public int currentPipeline = 0;

    public double ambiguity = 0.0;
    public double timestamp = 0.0;
    public double avgTagDistance = 0.0;

    public double angleMegatag1 = 0.0;

    public double ta = 0.0;
    public double tx = 0.0;
    public double ty = 0.0;

    public double[] botPoseTargetSpace = new double[0];
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void updateRobotOrientation(DoubleSupplier yawDegress) {}

  default void setPipeline(int pipeline) {}

  default void setPriorityTagID(int tagID) {}

  default void setLEDMode_ForceBlink() {}

  default void setLEDMode_ForceOff() {}

  default void setLEDMode_ForceOn() {}

  default void setLEDMode_PipelineControl() {}

  default VisionCamera getCamera() {
    return VisionCamera.LEFT;
  }
  ;
}
