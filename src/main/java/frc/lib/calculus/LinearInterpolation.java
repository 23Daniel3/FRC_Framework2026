package frc.lib.calculus;

/**
 * Performs piecewise linear interpolation over a fixed set of ordered points.
 *
 * <p>The interpolation assumes that the points are provided in ascending order by their {@code x}
 * values. For inputs outside the defined domain, the output is clamped to the nearest endpoint.
 *
 * <p>This implementation is optimized for performance by using a fixed array and avoiding dynamic
 * collections.
 */
public class LinearInterpolation {

  private final Point[] points;

  /**
   * Creates a linear interpolation function using the given points.
   *
   * @param points ordered interpolation points (sorted by {@code x})
   */
  public LinearInterpolation(Point... points) {
    this.points = points;
  }

  /**
   * Computes the interpolated value for the given input.
   *
   * <p>If the input is below the first point's {@code x}, the first point's {@code y} value is
   * returned. If the input is above the last point's {@code x}, the last point's {@code y} value is
   * returned.
   *
   * @param x the input value
   * @return the interpolated output value
   */
  public double calculate(double x) {

    if (x <= points[0].x) {
      return points[0].y;
    }

    for (int i = 1; i < points.length; i++) {
      if (x <= points[i].x) {
        Point start = points[i - 1];
        Point end = points[i];

        return ((x - start.x) / (end.x - start.x)) * (end.y - start.y) + start.y;
      }
    }

    return points[points.length - 1].y;
  }

  /** Represents a single interpolation point. */
  public static class Point {

    /** Input domain value. */
    public final double x;

    /** Output range value. */
    public final double y;

    /**
     * Creates a point in the interpolation domain.
     *
     * @param x input value
     * @param y output value
     */
    public Point(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }
}
