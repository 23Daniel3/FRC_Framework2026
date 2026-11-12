package org.ciprix.interfaces.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;

/**
 * SPARK Flex (NEO Vortex, or any brushless motor through the Flex dock). Everything is implemented
 * in {@link MotorIOSparkBase}; this class only creates the controller and knows its external
 * encoder port, which {@code withAlternateEncoder(cpr)} selects as the feedback sensor.
 *
 * <pre>{@code
 * MotorIO kicker = new MotorIOSparkFlex("Shooter/Kicker", 17, CONFIG_KICKER);
 * }</pre>
 */
public class MotorIOSparkFlex extends MotorIOSparkBase {

  public MotorIOSparkFlex(String name, int id, BasicMotorConfig config) {
    this(name, id, MotorType.kBrushless, MotorConfig.fromBasic(config));
  }

  public MotorIOSparkFlex(String name, int id, MotorConfig config) {
    this(name, id, MotorType.kBrushless, config);
  }

  public MotorIOSparkFlex(String name, int id, MotorType type, BasicMotorConfig config) {
    this(name, id, type, MotorConfig.fromBasic(config));
  }

  public MotorIOSparkFlex(String name, int id, MotorType type, MotorConfig config) {
    this(name, new SparkFlex(id, type), config);
  }

  private MotorIOSparkFlex(String name, SparkFlex spark, MotorConfig config) {
    super(name, spark, SparkFlexConfig::new, externalEncoderPort(spark), config);
  }

  private static SecondaryEncoderPort externalEncoderPort(SparkFlex spark) {
    return new SecondaryEncoderPort() {
      @Override
      public void configure(
          SparkBaseConfig config,
          int countsPerRevolution,
          double positionConversionFactor,
          double velocityConversionFactor) {
        ((SparkFlexConfig) config)
            .externalEncoder
            .countsPerRevolution(countsPerRevolution)
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
      }

      @Override
      public RelativeEncoder open() {
        return spark.getExternalEncoder();
      }
    };
  }
}
