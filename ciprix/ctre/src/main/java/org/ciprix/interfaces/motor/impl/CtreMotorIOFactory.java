package org.ciprix.interfaces.motor.impl;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import org.ciprix.interfaces.motor.MotorIOTalonFX;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorIO;
import org.ciprix.interfaces.motor.basic.BasicMotorIO;

/**
 * Factory for creating CTRE {@link MotorIO} instances. When the requested motor controller does not
 * natively support hardware-fused closed-loop with the configured external encoder, it
 * automatically wraps a {@link BasicMotorIO} inside a {@link MotorIOComposed} and emits a Driver
 * Station warning so the degradation (hardware closed-loop to 50 Hz RIO PID) is always visible.
 */
public final class CtreMotorIOFactory {
  private CtreMotorIOFactory() {}

  public static MotorIO createTalonFX(String name, int id, CANBus canBus, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusion(config)) {
      reportFallback(name);
      BasicMotorIO basicIO = new MotorIOTalonFX(name, id, canBus, config);
      return new MotorIOComposed(name, basicIO, config.externalEncoder, config);
    }
    return new MotorIOTalonFX(name, id, canBus, config);
  }

  private static boolean supportsHardwareFusion(MotorConfig config) {
    if (config.externalFusionType == null) return false;
    return config.externalFusionType == MotorConfig.FeedbackSensorType.REMOTE_CANCODER
        || config.externalFusionType == MotorConfig.FeedbackSensorType.FUSED_CANCODER
        || config.externalFusionType == MotorConfig.FeedbackSensorType.SYNC_CANCODER;
  }

  private static void reportFallback(String name) {
    DriverStation.reportWarning(
        "[CtreMotorIOFactory] "
            + name
            + ": hardware encoder fusion not supported for this configuration."
            + " Falling back to MotorIOComposed (50 Hz RIO PID).",
        false);
  }
}
