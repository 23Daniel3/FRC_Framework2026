package org.ciprix.interfaces.motor.impl;

import com.ctre.phoenix6.CANBus;
import org.ciprix.interfaces.motor.MotorIOTalonFX;
import org.ciprix.interfaces.motor.MotorIOTalonFXS;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorIO;
import org.ciprix.interfaces.motor.basic.BasicMotorIO;
import org.ciprix.util.Report;

/**
 * Factory for creating CTRE {@link MotorIO} instances. When the requested motor controller does not
 * natively support hardware-fused closed-loop with the configured external encoder, it
 * automatically wraps a {@link BasicMotorIO} inside a {@link MotorIOComposed} and emits a Driver
 * Station warning so the degradation (hardware closed-loop to a roboRIO loop) is always visible.
 *
 * <p>In the fallback, the controller is named {@code name + "/Output"} and created with {@link
 * org.ciprix.interfaces.motor.basic.BasicMotorConfig#copyAsPowerStage()}: the output range and the
 * gains are applied once, by {@link MotorIOComposed}.
 */
public final class CtreMotorIOFactory {
  private CtreMotorIOFactory() {}

  public static MotorIO createTalonFX(String name, int id, CANBus canBus, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusion(config)) {
      reportFallback(name);
      BasicMotorIO power =
          new MotorIOTalonFX(name + "/Output", id, canBus, config.copyAsPowerStage());
      return new MotorIOComposed(name, power, config.externalEncoder, config);
    }
    return new MotorIOTalonFX(name, id, canBus, config);
  }

  public static MotorIO createTalonFXS(String name, int id, CANBus canBus, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusion(config)) {
      reportFallback(name);
      BasicMotorIO power =
          new MotorIOTalonFXS(name + "/Output", id, canBus, config.copyAsPowerStage());
      return new MotorIOComposed(name, power, config.externalEncoder, config);
    }
    return new MotorIOTalonFXS(name, id, canBus, config);
  }

  private static boolean supportsHardwareFusion(MotorConfig config) {
    return config.externalFusionType == MotorConfig.FeedbackSensorType.REMOTE_CANCODER
        || config.externalFusionType == MotorConfig.FeedbackSensorType.FUSED_CANCODER
        || config.externalFusionType == MotorConfig.FeedbackSensorType.SYNC_CANCODER;
  }

  private static void reportFallback(String name) {
    Report.warning(
        "[CtreMotorIOFactory] "
            + name
            + ": hardware encoder fusion not supported for this configuration."
            + " Falling back to MotorIOComposed (closed loop on the roboRIO).");
  }
}
