package frc.lib.util.security.antitipping;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import frc.lib.calculus.ExponentialMovingAverage; // Importação da sua classe utilitária
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Advanced Anti-Tipping controller using State-Space-like separation.
 *
 * <p>Pipeline:
 *
 * <ol>
 *   <li>{@code estimateState}: Processes acceleration (from IMU) and tilt vectors.
 *   <li>{@code computeRisk}: Evaluates a weighted risk factor [0, 1].
 *   <li>{@code computeCorrection}: Determines counter-acting velocity if tipping.
 *   <li>{@code applyLimits}: Dynamically scales drivetrain acceleration limits based on risk.
 * </ol>
 */
public class AntiTippingAccel {

  // =========================================================================
  // 1. CONFIGURATION (Tuning Parameters)
  // =========================================================================

  /**
   * Configuration object for Anti-Tipping. Create this separately and pass it via {@link
   * #setConfig(AntiTippingConfig)}.
   */
  public static class AntiTippingConfig {
    public Angle tippingThreshold = Degrees.of(10.0);
    public LinearVelocity maxCorrectionSpeed = MetersPerSecond.of(2.0);

    // kP: Correction speed per degree of tilt (m/s / deg)
    public double kP = 0.2;

    public boolean dynamicAccelerationEnabled = true;
    public boolean reactiveOnlyMode = false;

    // Safety floors for dynamic limiting
    public double minAccelFraction = 0.2;
    public double minAngularAccelFraction = 0.2;

    // Risk Weights
    public double inclinationWeight = 1.0;
    public double linAccelWeight = 1.0;
    public double speedWeight = 0.5;
    public double alignmentWeight = 1.2;
    public double riskSensitivity = 2.0;

    // Center of Mass Estimates
    public Distance comHeight = Meters.of(0.15);
    public Distance comForwardOffset = Meters.of(0.0);
    public Distance comLateralOffset = Meters.of(0.0);
    public Distance characteristicLength = Meters.of(0.5);

    // Smoothing rates for limit application
    public LinearAcceleration accelChangeRate = MetersPerSecondPerSecond.of(2.0);
    public AngularAcceleration angularAccelChangeRate = RadiansPerSecondPerSecond.of(3.0);

    // Parametros fisicos e de loop para evitar hardcode
    public LinearAcceleration gravitySafetyThreshold = MetersPerSecondPerSecond.of(3.0);
    public LinearVelocity nominalRobotSpeed = MetersPerSecond.of(4.5);
    public LinearAcceleration defaultMaxAccel = MetersPerSecondPerSecond.of(5.0);
    public AngularAcceleration defaultMaxAngularAccel = RadiansPerSecondPerSecond.of(10.0);
    public double angularRiskScalar = 0.8;
    public Time loopPeriod = Seconds.of(0.02);

    // Filtro para dados ruidosos do acelerometro
    public double accelFilterAlpha = 0.15; // Valor entre 0 e 1 (menor = mais suave/lento)
  }

  // =========================================================================
  // 2. STATE (Observability/Logging)
  // =========================================================================

  /**
   * Represents the calculated state of the anti-tipping system for a single loop cycle. This record
   * is ideal for structured logging (AdvantageKit).
   */
  public record AntiTippingState(
      Angle pitch,
      Angle roll,
      Rotation2d tiltDirection,
      double inclinationMagnitudeDeg,
      LinearAcceleration estimatedAx,
      LinearAcceleration estimatedAy,
      double riskScore,
      boolean isTipping,
      ChassisSpeeds correctionSpeeds,
      LinearAcceleration appliedMaxAccel,
      AngularAcceleration appliedMaxAngularAccel) {}

  // =========================================================================
  // 3. LOGIC & MEMORY
  // =========================================================================

  private AntiTippingConfig config;
  private AntiTippingState lastState;

  // Filters for noisy acceleration data
  private ExponentialMovingAverage axFilter;
  private ExponentialMovingAverage ayFilter;

  // Memory for smoothing limits
  private double lastAppliedMaxAccMpss = Double.NaN;
  private double lastAppliedMaxAngAccRadpss = Double.NaN;

  public AntiTippingAccel(AntiTippingConfig initialConfig) {
    this.config = initialConfig;
    // Inicializa filtros com o alpha da config
    this.axFilter = new ExponentialMovingAverage(config.accelFilterAlpha);
    this.ayFilter = new ExponentialMovingAverage(config.accelFilterAlpha);
  }

  /**
   * Updates the configuration at runtime. Call this when tuning values change.
   *
   * @param newConfig The new configuration object.
   */
  public void setConfig(AntiTippingConfig newConfig) {
    // Se o alpha mudou, recria os filtros (opcional, mas recomendado para garantir consistência)
    if (this.config.accelFilterAlpha != newConfig.accelFilterAlpha) {
      this.axFilter = new ExponentialMovingAverage(newConfig.accelFilterAlpha);
      this.ayFilter = new ExponentialMovingAverage(newConfig.accelFilterAlpha);
    }
    this.config = newConfig;
  }

  /**
   * Main control loop method.
   *
   * @param robotRotation Current IMU orientation.
   * @param measuredAx Acceleration X directly from IMU (Pigeon).
   * @param measuredAy Acceleration Y directly from IMU (Pigeon).
   * @param drivetrain The drivetrain subsystem (for reading speed and setting limits).
   * @return Optional correction speeds if tipping is active.
   */
  public Optional<ChassisSpeeds> calculate(
      Rotation3d robotRotation,
      LinearAcceleration measuredAx,
      LinearAcceleration measuredAy,
      Drivetrain drivetrain) {
    // 1. Estimate State (Physics & Filtering)
    // Passamos os valores medidos para serem filtrados e empacotados
    var currentState = estimateState(robotRotation, measuredAx, measuredAy);

    // 2. Compute Risk (0.0 to 1.0)
    double risk = computeRisk(currentState, drivetrain.getRobotVelocity());

    // 3. Compute Correction (Active counter-flow)
    ChassisSpeeds correction = computeCorrection(currentState);

    // 4. Apply Limits (Preventative dynamic scaling)
    applyLimits(risk, drivetrain);

    // 5. Finalize State for Logging
    // We recreate the state record to include the final calculated risk and correction
    lastState =
        new AntiTippingState(
            currentState.pitch,
            currentState.roll,
            currentState.tiltDirection,
            currentState.incMagDeg,
            currentState.estimatedAx,
            currentState.estimatedAy,
            risk,
            currentState.isTipping,
            correction,
            MetersPerSecondPerSecond.of(lastAppliedMaxAccMpss),
            RadiansPerSecondPerSecond.of(lastAppliedMaxAngAccRadpss));

    logState();

    return lastState.isTipping ? Optional.of(correction) : Optional.empty();
  }

  // --- Internal Steps ---

  /** Step 1: Reads sensors, filters acceleration and calculates tilt geometry. */
  private InternalPhysicsState estimateState(
      Rotation3d rot, LinearAcceleration rawAx, LinearAcceleration rawAy) {

    // Apply Exponential Moving Average filter to noisy IMU acceleration
    double filteredAxVal = axFilter.calculate(rawAx.in(MetersPerSecondPerSecond));
    double filteredAyVal = ayFilter.calculate(rawAy.in(MetersPerSecondPerSecond));

    LinearAcceleration ax = MetersPerSecondPerSecond.of(filteredAxVal);
    LinearAcceleration ay = MetersPerSecondPerSecond.of(filteredAyVal);

    // Tilt Geometry
    Angle pitch = Radians.of(rot.getY());
    Angle roll = Radians.of(rot.getX());

    // We use hypot on degrees for linearity in calculation, though Radians are cleaner math
    double incMagDeg = Math.hypot(pitch.in(Degrees), roll.in(Degrees));
    boolean tipping = incMagDeg > config.tippingThreshold.in(Degrees);

    Rotation2d tiltDir = new Rotation2d(Math.atan2(roll.in(Radians), pitch.in(Radians)));

    return new InternalPhysicsState(pitch, roll, tiltDir, incMagDeg, tipping, ax, ay);
  }

  /** Step 2: Calculates the risk factor based on tilt, accel, and velocity alignment. */
  private double computeRisk(InternalPhysicsState state, ChassisSpeeds robotVel) {
    if (config.reactiveOnlyMode) return 0.0;

    // A. Inclination Risk
    double normInclination = state.incMagDeg / Math.max(0.1, config.tippingThreshold.in(Degrees));

    // B. Acceleration Risk (Projected onto tilt vector)
    double tiltCos = state.tiltDirection.getCos();
    double tiltSin = state.tiltDirection.getSin();
    double projAccel =
        state.estimatedAx.in(MetersPerSecondPerSecond) * tiltCos
            + state.estimatedAy.in(MetersPerSecondPerSecond) * tiltSin;

    // Using config.gravitySafetyThreshold instead of hardcoded 3.0
    double normAccel =
        Math.abs(projAccel)
            * Math.max(0.01, config.comHeight.in(Meters))
            / config.gravitySafetyThreshold.in(MetersPerSecondPerSecond);

    // C. Velocity & Alignment Risk
    double velMag = Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);
    double alignment = 0.0;
    if (velMag > 1e-3) {
      alignment =
          (robotVel.vxMetersPerSecond / velMag) * tiltCos
              + (robotVel.vyMetersPerSecond / velMag) * tiltSin;
    }
    double alignFactor = Math.max(0.0, alignment);
    // Using config.nominalRobotSpeed instead of hardcoded 4.5
    double normSpeed = velMag / config.nominalRobotSpeed.in(MetersPerSecond);

    // D. Directional Bias (COM Offsets)
    double fwdBias =
        MathUtil.clamp(
            config.comForwardOffset.in(Meters) / config.characteristicLength.in(Meters), -1, 1);
    double latBias =
        MathUtil.clamp(
            config.comLateralOffset.in(Meters) / config.characteristicLength.in(Meters), -1, 1);

    // Lógica: Se o componente de velocidade (forwardComponent) estiver alinhado com o viés
    // (fwdBias),
    // isso é perigoso. Multiplicamos para aumentar o risco.
    double forwardComponent = (velMag > 1e-3) ? (robotVel.vxMetersPerSecond / velMag) : 0.0;
    double lateralComponent = (velMag > 1e-3) ? (robotVel.vyMetersPerSecond / velMag) : 0.0;

    double frontEffect = 1.0 + (fwdBias * forwardComponent); // Se bias + e vel +, aumenta risco
    double sideEffect = 1.0 + (Math.abs(latBias) * Math.abs(lateralComponent));

    double dirFactor = MathUtil.clamp(frontEffect * sideEffect, 0.5, 2.0);

    // Agora o rawRisk usa o dirFactor
    double rawRisk =
        ((config.inclinationWeight * normInclination)
                + (config.linAccelWeight * normAccel)
                + (config.speedWeight * normSpeed * alignFactor * config.alignmentWeight))
            * dirFactor;

    // Usando config.riskSensitivity ao invés de hardcode 2.0
    return MathUtil.clamp(rawRisk / config.riskSensitivity, 0.0, 1.0);
  }

  /** Step 3: Computes the corrective velocity vector. */
  private ChassisSpeeds computeCorrection(InternalPhysicsState state) {
    if (!state.isTipping) {
      return new ChassisSpeeds();
    }

    double speedMps = config.kP * state.incMagDeg;
    speedMps =
        MathUtil.clamp(
            speedMps,
            -config.maxCorrectionSpeed.in(MetersPerSecond),
            config.maxCorrectionSpeed.in(MetersPerSecond));

    // Simplification: Standard swerve "Anti-Tip" usually pushes the wheels IN the direction of tilt
    // to "catch" the fall
    // or drives the chassis UNDER the COG.
    // If tilting forward (+Pitch), we drive forward (+X).
    Translation2d recoverVec =
        new Translation2d(0.0, 1.0).rotateBy(state.tiltDirection).times(speedMps);

    // Note: This coordinate transform assumes Field-Centric or Robot-Centric consistency.
    // Ideally, anti-tip is robot-relative.
    return new ChassisSpeeds(recoverVec.getX(), -recoverVec.getY(), 0.0);
  }

  /** Step 4: Applies dynamic limits to the drivetrain. */
  private void applyLimits(double risk, Drivetrain drivetrain) {
    if (!config.dynamicAccelerationEnabled || config.reactiveOnlyMode) return;

    double nominalMaxAcc = config.defaultMaxAccel.in(MetersPerSecondPerSecond);
    // Ideally: double nominalMaxAcc = drivetrain.getConfig().maxLinearAccel;
    // Since we don't have access to Drivetrain config structure here, we assume:
    try {
      nominalMaxAcc = drivetrain.getMaxAcceleration().in(MetersPerSecondPerSecond);
    } catch (Exception e) {
      /* Fallback uses config value */
    }

    double scale = MathUtil.clamp(1.0 - risk, config.minAccelFraction, 1.0);
    double targetMaxAcc = nominalMaxAcc * scale;

    // Smooth the change
    double dt = config.loopPeriod.in(Seconds); // Using config loop time
    double maxChange = config.accelChangeRate.in(MetersPerSecondPerSecond) * dt;

    if (Double.isNaN(lastAppliedMaxAccMpss)) lastAppliedMaxAccMpss = nominalMaxAcc;

    double nextAcc =
        MathUtil.clamp(
            targetMaxAcc, lastAppliedMaxAccMpss - maxChange, lastAppliedMaxAccMpss + maxChange);

    drivetrain.setMaxAcceleration(MetersPerSecondPerSecond.of(nextAcc));
    lastAppliedMaxAccMpss = nextAcc;

    // Repeat for Angular
    double nominalMaxAngAcc = config.defaultMaxAngularAccel.in(RadiansPerSecondPerSecond);
    try {
      nominalMaxAngAcc = drivetrain.getMaxAngularAcceleration().in(RadiansPerSecondPerSecond);
    } catch (Exception e) {
    }

    // Using config.angularRiskScalar instead of hardcoded 0.8
    double scaleAng =
        MathUtil.clamp(
            1.0 - (risk * config.angularRiskScalar), config.minAngularAccelFraction, 1.0);
    double targetMaxAngAcc = nominalMaxAngAcc * scaleAng;

    double maxAngChange = config.angularAccelChangeRate.in(RadiansPerSecondPerSecond) * dt;

    if (Double.isNaN(lastAppliedMaxAngAccRadpss)) lastAppliedMaxAngAccRadpss = nominalMaxAngAcc;

    double nextAngAcc =
        MathUtil.clamp(
            targetMaxAngAcc,
            lastAppliedMaxAngAccRadpss - maxAngChange,
            lastAppliedMaxAngAccRadpss + maxAngChange);

    drivetrain.setMaxAngularAcceleration(RadiansPerSecondPerSecond.of(nextAngAcc));
    lastAppliedMaxAngAccRadpss = nextAngAcc;
  }

  private void logState() {
    if (lastState == null) return;
    String root = "Subsystems/Drivetrain/AntiTip/";
    Logger.recordOutput(root + "Risk", lastState.riskScore);
    Logger.recordOutput(root + "IsTipping", lastState.isTipping);
    Logger.recordOutput(root + "InclinationDeg", lastState.inclinationMagnitudeDeg);
    Logger.recordOutput(root + "Limit/MaxAccel", lastState.appliedMaxAccel);
    Logger.recordOutput(root + "Correction/Vx", lastState.correctionSpeeds.vxMetersPerSecond);
    Logger.recordOutput(root + "Correction/Vy", lastState.correctionSpeeds.vyMetersPerSecond);
    Logger.recordOutput(root + "EstimatedAccel/X", lastState.estimatedAx);
    Logger.recordOutput(root + "EstimatedAccel/Y", lastState.estimatedAy);
  }

  /** Helper record to pass data between pipeline steps without polluting global state */
  private record InternalPhysicsState(
      Angle pitch,
      Angle roll,
      Rotation2d tiltDirection,
      double incMagDeg,
      boolean isTipping,
      LinearAcceleration estimatedAx,
      LinearAcceleration estimatedAy) {}

  /** Reset estimator history (call ever on enable) DO NOT FORGET DUMP DANIEL. */
  public void reset() {
    if (axFilter != null) axFilter.reset();
    if (ayFilter != null) ayFilter.reset();
    lastAppliedMaxAccMpss = Double.NaN;
  }
}
