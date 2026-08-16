package frc.lib.interfaces.motor.impl;

import com.ctre.phoenix6.CANBus;
import frc.lib.interfaces.motor.MotorIOSparkFlex;
import frc.lib.interfaces.motor.MotorIOSparkMax;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.interfaces.motor.advanced.MotorConfig;
import frc.lib.interfaces.motor.advanced.MotorIO;
import frc.lib.interfaces.motor.basic.BasicMotorIO;

/**
 * Factory for creating MotorIO instances, automatically injecting software composition when a
 * hardware fallback isn't natively supported by the motor controller.
 */
public class MotorIOFactory {

  public static MotorIO createSparkMax(String name, int id, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusionSpark(config)) {
      BasicMotorIO basicIO = new MotorIOSparkMax(name, id, config);
      return new MotorIOComposed(name, basicIO, config.externalEncoder, config);
    }
    return new MotorIOSparkMax(name, id, config);
  }

  public static MotorIO createSparkFlex(String name, int id, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusionSpark(config)) {
      BasicMotorIO basicIO = new MotorIOSparkFlex(name, id, config);
      return new MotorIOComposed(name, basicIO, config.externalEncoder, config);
    }
    return new MotorIOSparkFlex(name, id, config);
  }

  public static MotorIO createTalonFX(String name, int id, CANBus canBus, MotorConfig config) {
    if (config.hasExternalEncoder() && !supportsHardwareFusionTalon(config)) {
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
}
