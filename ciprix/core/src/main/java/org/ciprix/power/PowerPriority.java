package org.ciprix.power;

/**
 * How much a motor matters when the battery sags (see {@link PowerManager}). Set it in the motor
 * configuration:
 *
 * <pre>{@code
 * public static final MotorConfig MOTOR_CONFIG =
 *     new MotorConfig().currentLimit(Amps.of(25)).powerPriority(PowerPriority.LOW);
 * }</pre>
 *
 * <p>The values are the fraction of the configured current limit the motor keeps at each {@link
 * PowerManager.Level}: normal, reduced (battery below {@value PowerManager#REDUCED_BELOW_VOLTS} V)
 * and critical (below {@value PowerManager#CRITICAL_BELOW_VOLTS} V).
 */
public enum PowerPriority {
  /** Never reduced: the drivetrain and anything that must finish a movement. The default. */
  HIGH(1.0, 1.0, 1.0),

  /** Keeps 75% of its current limit when the battery sags and 50% close to a brownout. */
  NORMAL(1.0, 0.75, 0.5),

  /** Keeps 50%, then 25%: rollers, conveyors, anything that can wait a moment. */
  LOW(1.0, 0.5, 0.25);

  private final double[] scales;

  PowerPriority(double normal, double reduced, double critical) {
    this.scales = new double[] {normal, reduced, critical};
  }

  /** Fraction of the configured current limit kept at {@code level}. */
  public double scaleAt(PowerManager.Level level) {
    return scales[level.ordinal()];
  }
}
