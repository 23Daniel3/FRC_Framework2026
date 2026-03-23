package frc.lib.calculus;

import frc.lib.calculus.LinearInterpolation.Point;
import frc.lib.calculus.ThrottleMap.FitMode;
import frc.lib.logger.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * A tunable wrapper around {@link ThrottleMap} that allows real-time curve adjustment using {@link
 * LoggedTunableNumber}.
 *
 * <p>This class automatically rebuilds the internal {@link ThrottleMap} whenever any tunable value
 * changes.
 *
 * <p>Features:
 *
 * <ul>
 *   <li>Automatic sorting of points by X value to ensure mathematical safety
 *   <li>Multiple constructor options (point count, arrays, or {@link Point} objects)
 *   <li>Direct integration with the robot lifecycle via {@link #calculate()}
 * </ul>
 */
public class LoggedTunableMap {

  private final String name;
  private final FitMode mode;
  private final int polynomialDegree;

  private ThrottleMap currentMap;

  private final LoggedTunableNumber[] xTunables;
  private final LoggedTunableNumber[] yTunables;
  private final LoggedTunableNumber[] allTunables;

  /** Internal helper class used for safe point sorting. */
  private static class PointPair {
    double x;
    double y;

    PointPair(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  /**
   * Creates a tunable map with a default linear ramp from 0 to 1.
   *
   * @param name the map name used for NetworkTables
   * @param numberOfPoints number of control points
   * @param useCubicSpline whether to use cubic spline interpolation
   */
  public LoggedTunableMap(String name, int numberOfPoints, boolean useCubicSpline) {
    this(
        name,
        createDefaultRamp(numberOfPoints),
        createDefaultRamp(numberOfPoints),
        useCubicSpline ? FitMode.SPLINE_CUBIC : FitMode.LINEAR,
        1);
  }

  /**
   * Creates a polynomial tunable map with a default linear ramp.
   *
   * @param name the map name
   * @param numberOfPoints number of control points
   * @param degree polynomial degree
   */
  public LoggedTunableMap(String name, int numberOfPoints, int degree) {
    this(
        name,
        createDefaultRamp(numberOfPoints),
        createDefaultRamp(numberOfPoints),
        FitMode.POLYNOMIAL_N,
        degree);
  }

  /**
   * Creates a tunable map using {@link Point} objects.
   *
   * @param name the map name
   * @param useCubicSpline whether to use cubic spline interpolation
   * @param points control points
   */
  public LoggedTunableMap(String name, boolean useCubicSpline, Point... points) {
    this(
        name,
        extractX(points),
        extractY(points),
        useCubicSpline ? FitMode.SPLINE_CUBIC : FitMode.LINEAR,
        1);
  }

  /**
   * Creates a polynomial tunable map using {@link Point} objects.
   *
   * @param name the map name
   * @param degree polynomial degree
   * @param points control points
   */
  public LoggedTunableMap(String name, int degree, Point... points) {
    this(name, extractX(points), extractY(points), FitMode.POLYNOMIAL_N, degree);
  }

  /**
   * Creates a tunable map from raw X and Y arrays.
   *
   * @param name the map name
   * @param defaultX default X values
   * @param defaultY default Y values
   * @param useCubicSpline whether to use cubic spline interpolation
   */
  public LoggedTunableMap(
      String name, double[] defaultX, double[] defaultY, boolean useCubicSpline) {
    this(name, defaultX, defaultY, useCubicSpline ? FitMode.SPLINE_CUBIC : FitMode.LINEAR, 1);
  }

  /**
   * Creates a polynomial tunable map from raw X and Y arrays.
   *
   * @param name the map name
   * @param defaultX default X values
   * @param defaultY default Y values
   * @param degree polynomial degree
   */
  public LoggedTunableMap(String name, double[] defaultX, double[] defaultY, int degree) {
    this(name, defaultX, defaultY, FitMode.POLYNOMIAL_N, degree);
  }

  private LoggedTunableMap(
      String name, double[] defaultX, double[] defaultY, FitMode mode, int degree) {

    if (defaultX.length != defaultY.length) {
      throw new IllegalArgumentException("TunableMap: X and Y arrays must have the same length.");
    }

    this.name = name;
    this.mode = mode;
    this.polynomialDegree = degree;

    int numPoints = defaultX.length;
    this.xTunables = new LoggedTunableNumber[numPoints];
    this.yTunables = new LoggedTunableNumber[numPoints];
    this.allTunables = new LoggedTunableNumber[numPoints * 2];

    for (int i = 0; i < numPoints; i++) {
      String pointLabel = "Point " + (i + 1);
      xTunables[i] = new LoggedTunableNumber(name + "/" + pointLabel + " - Input X", defaultX[i]);
      yTunables[i] = new LoggedTunableNumber(name + "/" + pointLabel + " - Output Y", defaultY[i]);

      allTunables[i * 2] = xTunables[i];
      allTunables[i * 2 + 1] = yTunables[i];
    }

    updateMapFromTunables();
  }

  /**
   * Must be called periodically.
   *
   * <p>Rebuilds the internal map if any tunable value has changed.
   */
  public void calculate() {
    LoggedTunableNumber.ifChanged(hashCode(), this::updateMapFromTunables, allTunables);
  }

  private void updateMapFromTunables() {
    List<PointPair> points = new ArrayList<>();

    for (int i = 0; i < xTunables.length; i++) {
      points.add(new PointPair(xTunables[i].get(), yTunables[i].get()));
    }

    points.sort(Comparator.comparingDouble(p -> p.x));

    double[] sortedX = points.stream().mapToDouble(p -> p.x).toArray();
    double[] sortedY = points.stream().mapToDouble(p -> p.y).toArray();

    try {
      switch (mode) {
        case SPLINE_CUBIC:
          currentMap = new ThrottleMap(sortedX, sortedY, true);
          break;
        case POLYNOMIAL_N:
          currentMap = new ThrottleMap(sortedX, sortedY, (double) polynomialDegree);
          break;
        case LINEAR:
        default:
          currentMap = new ThrottleMap(sortedX, sortedY, false);
          break;
      }
    } catch (Exception e) {
      System.err.println("[TunableMap] Failed to update map '" + name + "': " + e.getMessage());
    }
  }

  /**
   * Applies the tunable throttle curve.
   *
   * @param input input value
   * @return mapped output value
   */
  public double applyThrottle(double input) {
    return currentMap == null ? input : currentMap.applyThrottle(input);
  }

  /**
   * Applies the tunable throttle curve using the absolute input value.
   *
   * @param input input value
   * @return mapped output value
   */
  public double applyThrottleAbs(double input) {
    return currentMap == null ? input : currentMap.applyThrottleAbs(input);
  }

  private static double[] createDefaultRamp(int count) {
    if (count < 2) {
      count = 2;
    }

    double[] arr = new double[count];
    for (int i = 0; i < count; i++) {
      arr[i] = (double) i / (count - 1);
    }
    return arr;
  }

  private static double[] extractX(Point[] points) {
    double[] x = new double[points.length];
    for (int i = 0; i < points.length; i++) {
      x[i] = points[i].x;
    }
    return x;
  }

  private static double[] extractY(Point[] points) {
    double[] y = new double[points.length];
    for (int i = 0; i < points.length; i++) {
      y[i] = points[i].y;
    }
    return y;
  }
}
