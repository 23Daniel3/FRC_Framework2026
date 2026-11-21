package org.ciprix.util.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Read-only view of a drivetrain, used by the driving helpers of the library (rotation strategies,
 * translation modifiers, alliance utilities).
 *
 * <p>The library never depends on a concrete drivetrain class. Make your drivetrain subsystem
 * implement this interface and every helper works with it, whatever swerve or tank implementation
 * is behind it.
 */
public interface DriveContext {

  /** Estimated robot pose on the field. */
  Pose2d getPose();

  /** Estimated robot heading. */
  default Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Measured robot-relative chassis speeds. */
  ChassisSpeeds getRobotVelocity();

  /** Maximum linear speed currently allowed. */
  LinearVelocity getMaxSpeed();

  /** Maximum angular speed currently allowed. */
  AngularVelocity getMaxAngularSpeed();
}
