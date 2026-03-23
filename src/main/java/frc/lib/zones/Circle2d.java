package frc.lib.zones;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;

/** Circular 2D zone defined by a center supplier and a radius. */
public final class Circle2d implements ZoneIO {

  private final Supplier<Translation2d> centerSupplier;
  private final double radius;
  private final double radiusSq;

  public Circle2d(Supplier<Translation2d> centerSupplier, double radius) {
    this.centerSupplier = centerSupplier;
    this.radius = radius;
    this.radiusSq = radius * radius;
  }

  public Circle2d(Translation2d center, double radius) {
    this(() -> center, radius);
  }

  @Override
  public boolean contains(Translation2d point) {
    Translation2d c = centerSupplier.get();
    double dx = point.getX() - c.getX();
    double dy = point.getY() - c.getY();
    return (dx * dx + dy * dy) <= radiusSq;
  }

  @Override
  public double distanceTo(Translation2d point) {
    Translation2d c = centerSupplier.get();
    double dx = point.getX() - c.getX();
    double dy = point.getY() - c.getY();
    double dist = Math.sqrt(dx * dx + dy * dy) - radius;
    return Math.max(0.0, dist);
  }

  @Override
  public Translation2d getCenter() {
    return centerSupplier.get();
  }

  public double getRadius() {
    return radius;
  }
}
