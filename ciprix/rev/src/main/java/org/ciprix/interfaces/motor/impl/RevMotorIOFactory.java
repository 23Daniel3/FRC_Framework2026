package org.ciprix.interfaces.motor.impl;

import edu.wpi.first.wpilibj.DriverStation;
import org.ciprix.interfaces.motor.MotorIOSparkFlex;
import org.ciprix.interfaces.motor.MotorIOSparkMax;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorIO;
import org.ciprix.interfaces.motor.basic.BasicMotorIO;

/**
 * Factory for creating REV {@link MotorIO} instances. When the requested motor controller does not
 * natively support hardware-fused closed-loop with the configured external encoder, it
 * automatically wraps a {@link BasicMotorIO} inside a {@link MotorIOComposed} and emits a Driver
 * Station warning so the degradation (hardware closed-loop to 50 Hz RIO PID) is always visible.
 */
public final class RevMotorIOFactory {
  private RevMotorIOFactory() {}

  public static MotorIO createSparkMax(String name, int id, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusion(config)) {
      reportFallback(name);
      BasicMotorIO basicIO = new MotorIOSparkMax(name, id, config);
      return new MotorIOComposed(name, basicIO, config.externalEncoder, config);
    }
    return new MotorIOSparkMax(name, id, config);
  }

  public static MotorIO createSparkFlex(String name, int id, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusion(config)) {
      reportFallback(name);
      BasicMotorIO basicIO = new MotorIOSparkFlex(name, id, config);
      return new MotorIOComposed(name, basicIO, config.externalEncoder, config);
    }
    return new MotorIOSparkFlex(name, id, config);
  }

  private static boolean supportsHardwareFusion(MotorConfig config) {
    if (config.externalFusionType == null) return false;
    return config.externalFusionType == MotorConfig.FeedbackSensorType.ALTERNATE
        || config.externalFusionType == MotorConfig.FeedbackSensorType.ABSOLUTE_DATAPORT;
  }

  private static void reportFallback(String name) {
    DriverStation.reportWarning(
        "[RevMotorIOFactory] "
            + name
            + ": hardware encoder fusion not supported for this configuration."
            + " Falling back to MotorIOComposed (50 Hz RIO PID).",
        false);
  }
}
