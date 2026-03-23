package frc.lib.zones;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a 2D geometric zone on the field.
 *
 * <p>Implementations must be lightweight and allocation-free during runtime.
 */
public interface ZoneIO {

  /**
   * Checks whether the given point lies inside the zone.
   *
   * @param point point in field coordinates
   * @return true if inside the zone
   */
  boolean contains(Translation2d point);

  /**
   * Computes the minimum distance from the zone boundary to the given point.
   *
   * <p>Returns 0 if the point is inside the zone.
   *
   * @param point reference point
   * @return distance in meters
   */
  double distanceTo(Translation2d point);

  /**
   * Returns the geometric center of the zone.
   *
   * @return center point
   */
  Translation2d getCenter();
}
