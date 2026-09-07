package frc.lib.interfaces.motor.impl;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.interfaces.motor.MotorIOSparkFlex;
import frc.lib.interfaces.motor.MotorIOSparkMax;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.interfaces.motor.advanced.MotorConfig;
import frc.lib.interfaces.motor.advanced.MotorIO;
import frc.lib.interfaces.motor.basic.BasicMotorIO;

/**
 * Factory for creating {@link MotorIO} instances. When the requested motor controller does not
 * natively support hardware-fused closed-loop with the configured external encoder, it
 * automatically wraps a {@link BasicMotorIO} inside a {@link MotorIOComposed} and emits a Driver
 * Station warning so the degradation (hardware closed-loop → 50 Hz RIO PID) is always visible.
 */
public class MotorIOFactory {

  public static MotorIO createSparkMax(String name, int id, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusionSpark(config)) {
      reportFallback(name);
      BasicMotorIO basicIO = new MotorIOSparkMax(name, id, config);
      return new MotorIOComposed(name, basicIO, config.externalEncoder, config);
    }
    return new MotorIOSparkMax(name, id, config);
  }

  public static MotorIO createSparkFlex(String name, int id, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusionSpark(config)) {
      reportFallback(name);
      BasicMotorIO basicIO = new MotorIOSparkFlex(name, id, config);
      return new MotorIOComposed(name, basicIO, config.externalEncoder, config);
    }
    return new MotorIOSparkFlex(name, id, config);
  }

  public static MotorIO createTalonFX(String name, int id, CANBus canBus, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusionTalon(config)) {
      reportFallback(name);
      BasicMotorIO basicIO = new MotorIOTalonFX(name, id, canBus, config);
      return new MotorIOComposed(name, basicIO, config.externalEncoder, config);
    }
    return new MotorIOTalonFX(name, id, canBus, config);
  }

  private static boolean supportsHardwareFusionSpark(MotorConfig config) {
    if (config.externalFusionType == null) return false;
    return config.externalFusionType == MotorConfig.FeedbackSensorType.ALTERNATE
        || config.externalFusionType == MotorConfig.FeedbackSensorType.ABSOLUTE_DATAPORT;
  }

  private static boolean supportsHardwareFusionTalon(MotorConfig config) {
    if (config.externalFusionType == null) return false;
    return config.externalFusionType == MotorConfig.FeedbackSensorType.REMOTE_CANCODER
        || config.externalFusionType == MotorConfig.FeedbackSensorType.FUSED_CANCODER
        || config.externalFusionType == MotorConfig.FeedbackSensorType.SYNC_CANCODER;
  }

  /**
   * Emits a Driver Station warning when the factory falls back to a 50 Hz RIO software PID loop
   * instead of the motor controller's native hardware closed-loop (250 Hz – 1 kHz). Review {@link
   * MotorConfig#externalFusionType} to suppress this if the degradation is intentional.
   */
  private static void reportFallback(String name) {
    DriverStation.reportWarning(
        "[MotorIOFactory] "
            + name
            + ": hardware encoder fusion not supported for this configuration."
            + " Falling back to MotorIOComposed (50 Hz RIO PID).",
        false);
  }
}
