package org.ciprix.interfaces.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.ciprix.interfaces.encoder.EncoderConfig;
import org.ciprix.interfaces.encoder.EncoderIO;
import org.ciprix.interfaces.encoder.EncoderIOSparkAlternate;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;

/**
 * SPARK MAX (NEO, NEO 550, or brushed motors). Everything is implemented in {@link
 * MotorIOSparkBase}; this class only creates the controller and knows its alternate encoder port.
 *
 * <pre>{@code
 * MotorIO pivot = new MotorIOSparkMax("Intake/Pivot", 19, CONFIG_PIVOT);
 * }</pre>
 */
public class MotorIOSparkMax extends MotorIOSparkBase {

  private final SparkMax spark;

  public MotorIOSparkMax(String name, int id, BasicMotorConfig config) {
    this(name, id, MotorType.kBrushless, MotorConfig.fromBasic(config));
  }

  public MotorIOSparkMax(String name, int id, MotorConfig config) {
    this(name, id, MotorType.kBrushless, config);
  }

  public MotorIOSparkMax(String name, int id, MotorType type, BasicMotorConfig config) {
    this(name, id, type, MotorConfig.fromBasic(config));
  }

  public MotorIOSparkMax(String name, int id, MotorType type, MotorConfig config) {
    this(name, new SparkMax(id, type), config);
  }

  private MotorIOSparkMax(String name, SparkMax spark, MotorConfig config) {
    super(name, spark, SparkMaxConfig::new, alternateEncoderPort(spark), config);
    this.spark = spark;
  }

  private static SecondaryEncoderPort alternateEncoderPort(SparkMax spark) {
    return new SecondaryEncoderPort() {
      @Override
      public void configure(
          SparkBaseConfig config,
          int countsPerRevolution,
          double positionConversionFactor,
          double velocityConversionFactor) {
        ((SparkMaxConfig) config)
            .alternateEncoder
            .countsPerRevolution(countsPerRevolution)
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
      }

      @Override
      public RelativeEncoder open() {
        return spark.getAlternateEncoder();
      }
    };
  }

  /**
   * A read-only {@link EncoderIO} view of the quadrature encoder on the alternate encoder port. The
   * controller object is shared, so no CAN id is duplicated.
   */
  public EncoderIO getAlternateEncoderIO() {
    return new EncoderIOSparkAlternate(name + "_AltEnc", new EncoderConfig(), spark);
  }
}
