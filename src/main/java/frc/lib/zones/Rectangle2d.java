package frc.lib.zones;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Axis-aligned rectangular zone defined by two opposite corners.
 *
 * <p>The rectangle is guaranteed to be non-rotated and cannot become a trapezoid or parallelogram.
 */
public final class Rectangle2d implements ZoneIO {

  private final double minX;
  private final double maxX;
  private final double minY;
  private final double maxY;
  private final Translation2d center;

  /**
   * Creates an axis-aligned rectangle from two opposite corners.
   *
   * @param a first corner
   * @param b opposite corner
   */
  public Rectangle2d(Translation2d a, Translation2d b) {
    this.minX = Math.min(a.getX(), b.getX());
    this.maxX = Math.max(a.getX(), b.getX());
    this.minY = Math.min(a.getY(), b.getY());
    this.maxY = Math.max(a.getY(), b.getY());

    this.center = new Translation2d((minX + maxX) * 0.5, (minY + maxY) * 0.5);
  }

  @Override
  public boolean contains(Translation2d p) {
    double x = p.getX();
    double y = p.getY();

    return x >= minX && x <= maxX && y >= minY && y <= maxY;
  }

  @Override
  public double distanceTo(Translation2d p) {
    double dx = Math.max(Math.max(minX - p.getX(), 0.0), p.getX() - maxX);
    double dy = Math.max(Math.max(minY - p.getY(), 0.0), p.getY() - maxY);
    return Math.hypot(dx, dy);
  }

  @Override
  public Translation2d getCenter() {
    return center;
  }

  /**
   * @return minimum X bound
   */
  public double getMinX() {
    return minX;
  }

  /**
   * @return maximum X bound
   */
  public double getMaxX() {
    return maxX;
  }

  /**
   * @return minimum Y bound
   */
  public double getMinY() {
    return minY;
  }

  /**
   * @return maximum Y bound
   */
  public double getMaxY() {
    return maxY;
  }

  /* ===================== */
  /*  Corner accessors     */
  /* ===================== */

  /**
   * @return bottom-left corner (minX, minY)
   */
  public Translation2d getBottomLeft() {
    return new Translation2d(minX, minY);
  }

  /**
   * @return top-right corner (maxX, maxY)
   */
  public Translation2d getTopRight() {
    return new Translation2d(maxX, maxY);
  }

  /**
   * @return bottom-right corner (maxX, minY)
   */
  public Translation2d getBottomRight() {
    return new Translation2d(maxX, minY);
  }

  /**
   * @return top-left corner (minX, maxY)
   */
  public Translation2d getTopLeft() {
    return new Translation2d(minX, maxY);
  }

  /**
   * Generic accessor for one of the defining opposite corners. Useful for flip and reconstruction.
   */
  public Translation2d getCornerA() {
    return getBottomLeft();
  }

  /** Generic accessor for the opposite defining corner. Useful for flip and reconstruction. */
  public Translation2d getCornerB() {
    return getTopRight();
  }
}
