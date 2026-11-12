package org.ciprix.util.drivetrain;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

/**
 * Acceleration limits of a drivetrain that can be adjusted at runtime (anti-tipping, power
 * management).
 */
public interface DriveAccelerationLimits {

  /** Maximum linear acceleration currently allowed. */
  LinearAcceleration getMaxAcceleration();

  /** Sets the maximum linear acceleration. */
  void setMaxAcceleration(LinearAcceleration maxAcceleration);

  /** Maximum angular acceleration currently allowed. */
  AngularAcceleration getMaxAngularAcceleration();

  /** Sets the maximum angular acceleration. */
  void setMaxAngularAcceleration(AngularAcceleration maxAngularAcceleration);
}
