package frc.lib.zones;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Generic 2D polygon zone.
 *
 * <p>Supports clockwise, counter-clockwise, and self-intersecting polygons using the even-odd rule.
 */
public final class Polygon2d implements ZoneIO {

  private final Translation2d[] vertices;
  private final Translation2d center;

  public Polygon2d(Translation2d... vertices) {
    if (vertices.length < 3) {
      throw new IllegalArgumentException("Polygon must have at least 3 vertices");
    }
    this.vertices = vertices;
    this.center = computeCenter(vertices);
  }

  @Override
  public boolean contains(Translation2d point) {
    boolean inside = false;
    int j = vertices.length - 1;

    double px = point.getX();
    double py = point.getY();

    for (int i = 0; i < vertices.length; i++) {
      double xi = vertices[i].getX();
      double yi = vertices[i].getY();
      double xj = vertices[j].getX();
      double yj = vertices[j].getY();

      boolean intersect = ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);

      if (intersect) inside = !inside;
      j = i;
    }
    return inside;
  }

  @Override
  public double distanceTo(Translation2d point) {
    if (contains(point)) return 0.0;

    double minDistSq = Double.POSITIVE_INFINITY;
    int j = vertices.length - 1;

    for (int i = 0; i < vertices.length; i++) {
      minDistSq = Math.min(minDistSq, distanceSqToSegment(vertices[j], vertices[i], point));
      j = i;
    }
    return Math.sqrt(minDistSq);
  }

  @Override
  public Translation2d getCenter() {
    return center;
  }

  private static double distanceSqToSegment(Translation2d a, Translation2d b, Translation2d p) {

    double ax = a.getX();
    double ay = a.getY();
    double bx = b.getX();
    double by = b.getY();
    double px = p.getX();
    double py = p.getY();

    double dx = bx - ax;
    double dy = by - ay;

    if (dx == 0.0 && dy == 0.0) {
      double dxp = px - ax;
      double dyp = py - ay;
      return dxp * dxp + dyp * dyp;
    }

    double t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy);
    t = Math.max(0.0, Math.min(1.0, t));

    double cx = ax + t * dx;
    double cy = ay + t * dy;

    double dcx = px - cx;
    double dcy = py - cy;
    return dcx * dcx + dcy * dcy;
  }

  private static Translation2d computeCenter(Translation2d[] v) {
    double x = 0.0;
    double y = 0.0;
    for (Translation2d p : v) {
      x += p.getX();
      y += p.getY();
    }
    return new Translation2d(x / v.length, y / v.length);
  }

  /**
   * Computes the closest point on the polygon perimeter to a given point.
   *
   * @param point the reference point
   * @return the closest point on the polygon edges
   */
  public Translation2d getClosestPoint(Translation2d point) {
    Translation2d closest = null;
    double minDistanceSq = Double.MAX_VALUE;

    int j = vertices.length - 1;
    for (int i = 0; i < vertices.length; i++) {
      Translation2d p1 = vertices[j];
      Translation2d p2 = vertices[i];

      Translation2d closestOnSegment = getClosestPointOnSegment(p1, p2, point);
      double distance = point.getDistance(closestOnSegment);
      double distSq = distance * distance;

      if (distSq < minDistanceSq) {
        minDistanceSq = distSq;
        closest = closestOnSegment;
      }

      j = i;
    }

    return closest;
  }

  /**
   * Computes the closest point on a line segment to a given point.
   *
   * @param p1 the first endpoint of the segment
   * @param p2 the second endpoint of the segment
   * @param point the reference point
   * @return the closest point on the segment
   */
  private Translation2d getClosestPointOnSegment(
      Translation2d p1, Translation2d p2, Translation2d point) {
    double x1 = p1.getX();
    double y1 = p1.getY();
    double x2 = p2.getX();
    double y2 = p2.getY();
    double px = point.getX();
    double py = point.getY();

    double dx = x2 - x1;
    double dy = y2 - y1;

    if (dx == 0 && dy == 0) {
      return p1;
    }

    double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
    t = Math.max(0, Math.min(1, t));

    return new Translation2d(x1 + t * dx, y1 + t * dy);
  }

  public Translation2d[] getVertices() {
    return vertices;
  }
}
