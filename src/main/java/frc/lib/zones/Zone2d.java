package frc.lib.zones;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Associates a {@link ZoneIO} with a logical identifier.
 *
 * @param <Z> zone enum type
 */
public final class Zone2d<Z extends Enum<Z>> {

  private final ZoneIO zone;
  private final Z id;

  public Zone2d(Z id, ZoneIO zone) {
    this.id = id;
    this.zone = zone;
  }

  public boolean contains(Translation2d point) {
    return zone.contains(point);
  }

  public double distanceTo(Translation2d point) {
    return zone.distanceTo(point);
  }

  public Z getId() {
    return id;
  }

  public ZoneIO getZone() {
    return zone;
  }
}
