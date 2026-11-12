package org.ciprix.interfaces.motor.impl;

import org.ciprix.interfaces.motor.MotorIOSparkFlex;
import org.ciprix.interfaces.motor.MotorIOSparkMax;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorIO;
import org.ciprix.interfaces.motor.basic.BasicMotorIO;
import org.ciprix.util.Report;

/**
 * Factory for creating REV {@link MotorIO} instances. When the requested motor controller does not
 * natively support hardware-fused closed-loop with the configured external encoder, it
 * automatically wraps a {@link BasicMotorIO} inside a {@link MotorIOComposed} and emits a Driver
 * Station warning so the degradation (hardware closed-loop to a roboRIO loop) is always visible.
 *
 * <p>In the fallback, the controller is named {@code name + "/Output"} and created with {@link
 * org.ciprix.interfaces.motor.basic.BasicMotorConfig#copyAsPowerStage()}: the output range and the
 * gains are applied once, by {@link MotorIOComposed}.
 */
public final class RevMotorIOFactory {
  private RevMotorIOFactory() {}

  public static MotorIO createSparkMax(String name, int id, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusion(config)) {
      reportFallback(name);
      BasicMotorIO power = new MotorIOSparkMax(name + "/Output", id, config.copyAsPowerStage());
      return new MotorIOComposed(name, power, config.externalEncoder, config);
    }
    return new MotorIOSparkMax(name, id, config);
  }

  public static MotorIO createSparkFlex(String name, int id, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusion(config)) {
      reportFallback(name);
      BasicMotorIO power = new MotorIOSparkFlex(name + "/Output", id, config.copyAsPowerStage());
      return new MotorIOComposed(name, power, config.externalEncoder, config);
    }
    return new MotorIOSparkFlex(name, id, config);
  }

  private static boolean supportsHardwareFusion(MotorConfig config) {
    return config.externalFusionType == MotorConfig.FeedbackSensorType.ALTERNATE
        || config.externalFusionType == MotorConfig.FeedbackSensorType.ABSOLUTE_DATAPORT;
  }

  private static void reportFallback(String name) {
    Report.warning(
        "[RevMotorIOFactory] "
            + name
            + ": hardware encoder fusion not supported for this configuration."
            + " Falling back to MotorIOComposed (closed loop on the roboRIO).");
  }
}
