package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.function.Supplier;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.advanced.MotorConfig.FeedbackSensorType;
import org.ciprix.interfaces.motor.advanced.MotorConfig.GravityType;
import org.ciprix.util.Report;

/**
 * Writes a {@link MotorConfig} into the configuration groups that Talon FX and Talon FXS share, so
 * both motor classes translate the configuration in exactly the same way.
 */
final class TalonConfigs {

  /** Phoenix 6 Talons have three gain slots. */
  static final int TALON_SLOTS = 3;

  /** Time allowed to the boot configuration, per attempt. */
  static final double BOOT_TIMEOUT_SECONDS = 0.25;

  /** Time allowed to a change made while the robot runs (kept short: it blocks the loop). */
  static final double RUNTIME_TIMEOUT_SECONDS = 0.02;

  static final int BOOT_ATTEMPTS = 5;

  private TalonConfigs() {}

  static void writeOutput(MotorOutputConfigs output, boolean inverted, boolean brake) {
    output.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    output.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
  }

  static void writeCurrentLimits(
      CurrentLimitsConfigs limits, double statorAmps, double supplyAmps) {
    limits.StatorCurrentLimit = statorAmps;
    limits.StatorCurrentLimitEnable = true;
    limits.SupplyCurrentLimit = supplyAmps;
    limits.SupplyCurrentLimitEnable = true;
  }

  /** Supply limit of the configuration: the explicit one, or the stator limit. */
  static double supplyLimitAmps(MotorConfig config) {
    return config.supplyCurrentLimit != null
        ? Math.abs(config.supplyCurrentLimit.in(Amps))
        : Math.abs(config.currentLimit.in(Amps));
  }

  /** Output range as peak voltages, relative to the nominal voltage. */
  static void writePeaks(VoltageConfigs voltage, double min, double max, double nominalVolts) {
    voltage.PeakForwardVoltage = max * nominalVolts;
    voltage.PeakReverseVoltage = min * nominalVolts;
  }

  static void writeRamps(
      OpenLoopRampsConfigs open, ClosedLoopRampsConfigs closed, double openSec, double closedSec) {
    double openRamp = Math.max(0.0, openSec);
    double closedRamp = Math.max(0.0, closedSec);
    open.VoltageOpenLoopRampPeriod = openRamp;
    open.DutyCycleOpenLoopRampPeriod = openRamp;
    closed.VoltageClosedLoopRampPeriod = closedRamp;
    closed.DutyCycleClosedLoopRampPeriod = closedRamp;
  }

  static void writeSoftLimits(SoftwareLimitSwitchConfigs limits, MotorConfig config) {
    limits.ForwardSoftLimitEnable = config.softLimitEnabled;
    limits.ReverseSoftLimitEnable = config.softLimitEnabled;
    if (config.softLimitEnabled) {
      limits.ForwardSoftLimitThreshold = config.maxPosition.in(Rotations);
      limits.ReverseSoftLimitThreshold = config.minPosition.in(Rotations);
    }
  }

  /**
   * Continuous position (turret, swerve azimuth). Phoenix 6 wraps over exactly one rotation of the
   * mechanism, so other ranges are reported and ignored.
   */
  static void writeWrap(ClosedLoopGeneralConfigs general, MotorConfig config, String name) {
    if (!config.positionWrap) {
      return;
    }
    double range = Math.abs(config.maxPosition.in(Rotations) - config.minPosition.in(Rotations));
    if (Math.abs(range - 1.0) < 1e-6) {
      general.ContinuousWrap = true;
    } else {
      Report.warning(
          "Motor "
              + name
              + ": a Talon wraps over exactly 1 rotation of the mechanism; positionWrap over "
              + range
              + " rotations ignored.");
    }
  }

  static void writeMotion(MotionMagicConfigs motion, double cruiseRps, double accelerationRps2) {
    motion.MotionMagicCruiseVelocity = Math.max(0.0, cruiseRps);
    motion.MotionMagicAcceleration = Math.max(0.0, accelerationRps2);
  }

  static GravityTypeValue gravity(MotorConfig config) {
    return config.gravityType == GravityType.ARM
        ? GravityTypeValue.Arm_Cosine
        : GravityTypeValue.Elevator_Static;
  }

  static void writeGains(Slot0Configs slot, MotorConfig config, int i) {
    slot.kP = config.kP[i];
    slot.kI = config.kI[i];
    slot.kD = config.kD[i];
    slot.kS = config.kS[i];
    slot.kV = config.kV[i];
    slot.kA = config.kA[i];
    slot.kG = config.kG[i];
    slot.GravityType = gravity(config);
  }

  static void writeGains(Slot1Configs slot, MotorConfig config, int i) {
    slot.kP = config.kP[i];
    slot.kI = config.kI[i];
    slot.kD = config.kD[i];
    slot.kS = config.kS[i];
    slot.kV = config.kV[i];
    slot.kA = config.kA[i];
    slot.kG = config.kG[i];
    slot.GravityType = gravity(config);
  }

  static void writeGains(Slot2Configs slot, MotorConfig config, int i) {
    slot.kP = config.kP[i];
    slot.kI = config.kI[i];
    slot.kD = config.kD[i];
    slot.kS = config.kS[i];
    slot.kV = config.kV[i];
    slot.kA = config.kA[i];
    slot.kG = config.kG[i];
    slot.GravityType = gravity(config);
  }

  /** Motion Magic cruise velocity of a slot, in rotations per second. */
  static double cruise(MotorConfig config, int slot) {
    return config.maxMotionMaxVelocity[slot].in(RotationsPerSecond);
  }

  /** Motion Magic acceleration of a slot, in rotations per second squared. */
  static double acceleration(MotorConfig config, int slot) {
    return config.maxMotionMaxAcceleration[slot].in(RotationsPerSecondPerSecond);
  }

  /** True when the robot configured gains or a profile on slot 3, which a Talon does not have. */
  static boolean usesSlot3(MotorConfig config) {
    return config.kP[3] != 0
        || config.kI[3] != 0
        || config.kD[3] != 0
        || config.kS[3] != 0
        || config.kV[3] != 0
        || config.kA[3] != 0
        || config.kG[3] != 0
        || cruise(config, 3) != 0;
  }

  /**
   * Sensor to mechanism ratio from the position conversion factor (mechanism = sensor x factor).
   */
  static double sensorToMechanismRatio(MotorConfig config) {
    return config.positionConversionFactor != 0.0 ? 1.0 / config.positionConversionFactor : 1.0;
  }

  /** True for the feedback types that read a CANcoder directly from the Talon. */
  static boolean isCANcoder(FeedbackSensorType type) {
    return type == FeedbackSensorType.REMOTE_CANCODER
        || type == FeedbackSensorType.FUSED_CANCODER
        || type == FeedbackSensorType.SYNC_CANCODER;
  }

  /**
   * CAN id of the CANcoder used as feedback: {@code remoteSensorId}, or the id of the {@link
   * org.ciprix.interfaces.encoder.EncoderIOCANcoder} given to {@code withExternalEncoder}. -1 when
   * there is none.
   */
  static int remoteSensorId(MotorConfig config) {
    if (config.remoteSensorId >= 0) {
      return config.remoteSensorId;
    }
    if (config.externalEncoder
        instanceof org.ciprix.interfaces.encoder.EncoderIOCANcoder cancoder) {
      return cancoder.getDeviceId();
    }
    return -1;
  }

  /**
   * The feedback type the Talon uses: {@code feedbackType}, or a CANcoder fused through {@code
   * withExternalEncoder}. Types a Talon FX cannot read fall back to the rotor sensor (reported).
   */
  static FeedbackSensorType resolveSensor(MotorConfig config, String name, boolean allowDataPort) {
    FeedbackSensorType type = config.feedbackType;
    if (!isCANcoder(type) && isCANcoder(config.externalFusionType)) {
      type = config.externalFusionType;
    }
    if (isCANcoder(type)) {
      if (remoteSensorId(config) < 0) {
        Report.error(
            "Motor "
                + name
                + ": "
                + type
                + " needs the CAN id of the CANcoder (withRemoteCANcoder); using the motor"
                + " sensor.");
        return FeedbackSensorType.INTERNAL;
      }
      return type;
    }
    if (type == FeedbackSensorType.INTERNAL) {
      return type;
    }
    if (allowDataPort
        && (type == FeedbackSensorType.ALTERNATE || type == FeedbackSensorType.ABSOLUTE_DATAPORT)) {
      return type;
    }
    Report.warning(
        "Motor "
            + name
            + ": feedback "
            + type
            + " does not exist on this Talon; using its sensor.");
    return FeedbackSensorType.INTERNAL;
  }

  /** Applies a configuration at boot, retrying, and reports when it never succeeds. */
  static void applyAtBoot(String name, Supplier<StatusCode> apply) {
    StatusCode status = StatusCode.OK;
    for (int attempt = 0; attempt < BOOT_ATTEMPTS; attempt++) {
      status = apply.get();
      if (status.isOK()) {
        return;
      }
    }
    Report.error(
        "Motor "
            + name
            + ": configuration failed after "
            + BOOT_ATTEMPTS
            + " attempts ("
            + status
            + "). Check the CAN wiring, the id and the bus.");
  }
}
