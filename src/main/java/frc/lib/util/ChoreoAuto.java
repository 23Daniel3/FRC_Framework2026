package frc.lib.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/** Command to execute a Choreo trajectory using PathPlanner. */
public class ChoreoAuto extends Command {

  private final String traj;
  private Command pathfollower;

  /**
   * Constructs a ChoreoAuto command.
   *
   * @param traj The name of the Choreo trajectory to load.
   * @param timeOut Timeout for the command execution.
   */
  public ChoreoAuto(String traj) {
    this.traj = traj;
  }

  @Override
  public void initialize() {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(traj);
      pathfollower = AutoBuilder.followPath(path);
      pathfollower.initialize();
    } catch (IOException e) {
      e.printStackTrace();
    } catch (ParseException e) {
      e.printStackTrace();
    } catch (FileVersionException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void execute() {
    if (pathfollower != null) {
      pathfollower.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return pathfollower == null || pathfollower.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (pathfollower != null) {
      pathfollower.end(interrupted);
    }
  }
}
