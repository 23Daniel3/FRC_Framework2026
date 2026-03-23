package frc.lib.calculus;

import edu.wpi.first.math.MathUtil;
import frc.lib.calculus.LinearInterpolation.Point;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

/**
 * Flexible throttle mapping class that supports multiple curve-fitting modes:
 *
 * <ul>
 *   <li><b>Linear</b> – default piecewise linear mapping.
 *   <li><b>Cubic Spline</b> – smooth piecewise cubic interpolation between points.
 *   <li><b>Polynomial (degree N)</b> – global polynomial fit of arbitrary degree.
 * </ul>
 *
 * <p>All mappings clamp input values to the valid domain before evaluation.
 */
public class ThrottleMap {

  /** Available fitting modes. */
  public enum FitMode {
    LINEAR,
    SPLINE_CUBIC,
    POLYNOMIAL_N
  }

  private final double[] inputValues;
  private final double[] outputValues;
  private final FitMode mode;

  // One of these will be used depending on the selected mode:
  private PolynomialSplineFunction splineFunction = null;
  private PolynomialFunction polynomialFunction = null;

  // Polynomial degree (only used for POLYNOMIAL_N)
  private int polynomialDegree = 1;

  /* ============================================================
   * === CONSTRUCTORS ===========================================
   * ============================================================ */

  /** Default constructor – creates a linear (piecewise) throttle curve. */
  public ThrottleMap(double[] inputValues, double[] outputValues) {
    this.inputValues = inputValues;
    this.outputValues = outputValues;
    this.mode = FitMode.LINEAR;
    this.splineFunction = createLinearSpline();
  }

  /**
   * Simplified 2-point linear mapping constructor.
   *
   * <p>Useful when the mapping is simply a straight line between two points. Internally treated as
   * a piecewise linear spline with one segment.
   *
   * @param input1 First input value.
   * @param input2 Second input value.
   * @param output1 Output corresponding to {@code input1}.
   * @param output2 Output corresponding to {@code input2}.
   */
  public ThrottleMap(double input1, double input2, double output1, double output2) {
    this.inputValues = new double[] {input1, input2};
    this.outputValues = new double[] {output1, output2};
    this.mode = FitMode.LINEAR;
    this.splineFunction = createLinearSpline();
  }

  /**
   * Simplified 3-point linear mapping constructor.
   *
   * <p>Useful for creating a basic curve (e.g., deadband or mid-curve boost) without manually
   * passing arrays. Internally builds a piecewise linear spline with two segments.
   *
   * @param input1 First input value.
   * @param input2 Second input value.
   * @param input3 Third input value.
   * @param output1 Output corresponding to {@code input1}.
   * @param output2 Output corresponding to {@code input2}.
   * @param output3 Output corresponding to {@code input3}.
   */
  public ThrottleMap(
      double input1, double input2, double input3, double output1, double output2, double output3) {
    this.inputValues = new double[] {input1, input2, input3};
    this.outputValues = new double[] {output1, output2, output3};
    this.mode = FitMode.LINEAR;
    this.splineFunction = createLinearSpline();
  }

  public ThrottleMap(Point... points) {
    if (points.length < 2) {
      throw new IllegalArgumentException("ThrottleMap requires at least 2 points");
    }

    this.inputValues = new double[points.length];
    this.outputValues = new double[points.length];

    for (int i = 0; i < points.length; i++) {
      this.inputValues[i] = points[i].x;
      this.outputValues[i] = points[i].y;
    }

    this.mode = FitMode.LINEAR;
    this.splineFunction = createLinearSpline();
  }

  /**
   * Constructor for cubic spline interpolation.
   *
   * @param inputValues Array of input domain values.
   * @param outputValues Array of output range values.
   * @param useCubicSpline If true, uses a cubic spline interpolation.
   */
  public ThrottleMap(double[] inputValues, double[] outputValues, boolean useCubicSpline) {
    this.inputValues = inputValues;
    this.outputValues = outputValues;

    if (useCubicSpline) {
      this.mode = FitMode.SPLINE_CUBIC;
      this.splineFunction = new SplineInterpolator().interpolate(inputValues, outputValues);
    } else {
      // false → fallback to linear
      this.mode = FitMode.LINEAR;
      this.splineFunction = createLinearSpline();
    }
  }

  public ThrottleMap(boolean useCubicSpline, Point... points) {
    Point[] sorted = sortPoints(points);
    this.inputValues = extractX(sorted);
    this.outputValues = extractY(sorted);

    if (useCubicSpline) {
      this.mode = FitMode.SPLINE_CUBIC;
      this.splineFunction =
          new SplineInterpolator().interpolate(this.inputValues, this.outputValues);
    } else {
      this.mode = FitMode.LINEAR;
      this.splineFunction = createLinearSpline();
    }
  }

  /**
   * Constructor for polynomial fit of arbitrary degree.
   *
   * @param inputValues Array of input domain values.
   * @param outputValues Array of output range values.
   * @param degree Polynomial degree (e.g., 2 = quadratic, 3 = cubic, etc.)
   */
  public ThrottleMap(double[] inputValues, double[] outputValues, double degree) {
    this.inputValues = inputValues;
    this.outputValues = outputValues;
    this.mode = FitMode.POLYNOMIAL_N;
    this.polynomialDegree = (int) Math.round(degree);
    this.polynomialFunction = createPolynomialFit(inputValues, outputValues, this.polynomialDegree);
  }

  public ThrottleMap(int degree, Point... points) {
    Point[] sorted = sortPoints(points);
    this.inputValues = extractX(sorted);
    this.outputValues = extractY(sorted);
    this.mode = FitMode.POLYNOMIAL_N;
    this.polynomialDegree = degree;
    this.polynomialFunction = createPolynomialFit(this.inputValues, this.outputValues, degree);
  }

  /* ============================================================
   * === INTERNAL CURVE CREATION METHODS ========================
   * ============================================================ */

  /** Creates the default piecewise linear spline. */
  private PolynomialSplineFunction createLinearSpline() {
    PolynomialFunction[] polys = new PolynomialFunction[inputValues.length - 1];
    for (int i = 0; i < polys.length; i++) {
      double a = outputValues[i];
      double b = (outputValues[i + 1] - outputValues[i]) / (inputValues[i + 1] - inputValues[i]);
      polys[i] = new PolynomialFunction(new double[] {a, b});
    }
    return new PolynomialSplineFunction(inputValues, polys);
  }

  /** Creates a polynomial fit of degree N using least-squares fitting. */
  private PolynomialFunction createPolynomialFit(double[] x, double[] y, int degree) {
    WeightedObservedPoints obs = new WeightedObservedPoints();
    for (int i = 0; i < x.length; i++) obs.add(x[i], y[i]);
    PolynomialCurveFitter fitter = PolynomialCurveFitter.create(degree);
    double[] coeffs = fitter.fit(obs.toList());
    return new PolynomialFunction(coeffs);
  }

  /* ============================================================
   * === PUBLIC EVALUATION METHODS ==============================
   * ============================================================ */

  /** Applies the throttle mapping to a positive input. */
  public double applyThrottle(double input) {
    double clamped = MathUtil.clamp(input, inputValues[0], inputValues[inputValues.length - 1]);
    switch (mode) {
      case SPLINE_CUBIC:
      case LINEAR:
        return splineFunction.value(clamped);
      case POLYNOMIAL_N:
        return polynomialFunction.value(clamped);
      default:
        return clamped;
    }
  }

  /** Applies the throttle mapping while preserving input sign (useful for joysticks). */
  public double applyThrottleAbs(double input) {
    double magnitude = Math.abs(input);
    double mapped = applyThrottle(magnitude);
    return Math.copySign(mapped, input);
  }

  /** Returns the current fit mode for debugging. */
  public FitMode getMode() {
    return mode;
  }

  /** Returns polynomial degree if applicable (otherwise 1). */
  public int getPolynomialDegree() {
    return polynomialDegree;
  }

  private static double[] extractX(Point[] points) {
    return java.util.Arrays.stream(points).mapToDouble(p -> p.x).toArray();
  }

  private static double[] extractY(Point[] points) {
    return java.util.Arrays.stream(points).mapToDouble(p -> p.y).toArray();
  }

  private static Point[] sortPoints(Point[] points) {
    return java.util.Arrays.stream(points)
        .sorted(java.util.Comparator.comparingDouble(p -> p.x))
        .toArray(Point[]::new);
  }

  /**
   * Returns the curve data points for visualization. Index 0: Input values (X-axis) Index 1: Output
   * values (Y-axis)
   */
  public double[][] getCurveData() {
    return new double[][] {inputValues, outputValues};
  }
}
